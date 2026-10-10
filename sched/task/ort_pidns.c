/****************************************************************************
 * sched/task/ort_pidns.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * [ORT-A §98] 容器 pid 命名空间**第一刀**（自省面）：
 *
 *   命名空间 = **同一个容器 root 下的全部任务组**（与 §87/§96③ 闸的
 *   "容器"谓词同源：A 侧设根容器；M 侧绑域不参与本轮，如实）。
 *   某组**首次**进入命名空间时取一个本地号（1 起递增），此后：
 *     · getpid（用户态读 tg_info->ta_pid）→ 本地号；
 *     · getppid → 父组同命名空间 ⇒ 父组本地号；否则 0（照 POSIX
 *       命名空间口径：命名空间外的父 = 0）；
 *     · kill/tgkill/sigqueue 的 pid 参数**本地号优先**解析——命中本
 *       命名空间成员即换成全局号走原路；未命中按原全局号走原闸
 *       （跨组 → §96③ 一样 -EPERM）。
 *
 *   边界如实（第一刀）：嵌套 spawn 的回传号、waitpid 号面仍全局；
 *   命名空间**不隔离**内核对象生命周期（无 pid 翻译表 —— 本地号是
 *   组属性，组亡随亡）。
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_ORT_CONTAINER

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <nuttx/sched.h>
#include <nuttx/kmalloc.h>
#include <nuttx/debug.h>

#include "sched/sched.h"
#include "task/task.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ORT_PIDNS_MAX     8       /* 同时存在的容器命名空间上限（原型） */
#define ORT_PIDNS_ROOTLEN 64

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ort_pidns_s
{
  char     pd_root[ORT_PIDNS_ROOTLEN];  /* 命名空间键：容器 root 串 */
  uint32_t pd_next;                     /* 下一个本地号（1 起） */
  uint16_t pd_refs;                     /* 引用（组数）计数 */
  bool     pd_used;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct ort_pidns_s g_ort_pidns[ORT_PIDNS_MAX];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ort_pidns_join
 *
 * Description:
 *   组进入命名空间：按 root 串查/建命名空间，取本地号返回。
 *   表满返回 0（调用方按"未入命名空间 = 全局语义"降级，fail-open
 *   到既有行为——不制造半隔离）。
 *
 ****************************************************************************/

uint32_t ort_pidns_join(FAR const char *root)
{
  irqstate_t flags;
  int i;
  int free = -1;
  uint32_t lpid = 0;

  if (root == NULL || root[0] == '\0')
    {
      return 0;
    }

  flags = enter_critical_section();

  for (i = 0; i < ORT_PIDNS_MAX; i++)
    {
      if (g_ort_pidns[i].pd_used)
        {
          if (strncmp(g_ort_pidns[i].pd_root, root, ORT_PIDNS_ROOTLEN) == 0)
            {
              lpid = ++g_ort_pidns[i].pd_next;
              g_ort_pidns[i].pd_refs++;
              break;
            }
        }
      else if (free < 0)
        {
          free = i;
        }
    }

  if (lpid == 0 && free >= 0)
    {
      strlcpy(g_ort_pidns[free].pd_root, root, ORT_PIDNS_ROOTLEN);
      g_ort_pidns[free].pd_used = true;
      g_ort_pidns[free].pd_refs = 1;
      lpid = ++g_ort_pidns[free].pd_next;
    }

  leave_critical_section(flags);
  return lpid;
}

/****************************************************************************
 * Name: ort_pidns_put
 *
 * Description:
 *   组离开命名空间（组销毁时）。最后一个引用离开即回收。
 *
 ****************************************************************************/

void ort_pidns_put(FAR const char *root)
{
  irqstate_t flags;
  int i;

  if (root == NULL || root[0] == '\0')
    {
      return;
    }

  flags = enter_critical_section();

  for (i = 0; i < ORT_PIDNS_MAX; i++)
    {
      if (g_ort_pidns[i].pd_used &&
          strncmp(g_ort_pidns[i].pd_root, root, ORT_PIDNS_ROOTLEN) == 0)
        {
          if (g_ort_pidns[i].pd_refs > 0 &&
              --g_ort_pidns[i].pd_refs == 0)
            {
              g_ort_pidns[i].pd_used = false;
              g_ort_pidns[i].pd_next = 0;
            }

          break;
        }
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: ort_pid_resolve
 *
 * Description:
 *   信号入口（kill/tgkill/sigqueue）的号解析**第一刀**：调用者是容器
 *   成员且 pid>0 命中**本命名空间**成员的本地号 ⇒ 换成其全局 pid；
 *   否则原样返回（后续按原全局号走既有闸 —— 跨组 -EPERM 不变）。
 *   遍历按 PIDHASH（任务数小，临界区内直扫）。
 *
 ****************************************************************************/

pid_t ort_pid_resolve(pid_t pid)
{
  FAR struct tcb_s *rtcb = this_task();
  FAR const char *root;
  irqstate_t flags;
  pid_t found = pid;
  int i;

  if (pid <= 0 || rtcb == NULL || rtcb->group == NULL)
    {
      return pid;
    }

  root = rtcb->group->tg_ort_root;
  if (root == NULL || root[0] == '\0')
    {
      return pid;                 /* 非容器：全局语义原样 */
    }

  flags = enter_critical_section();

  /* PIDHASH 表直扫（pid 唯一、无冲突；表大小即上界 —— 同
   * nxsched_get_tcb 的口径） */

  for (i = 0; i < g_npidhash; i++)
    {
      FAR struct tcb_s *tcb = g_pidhash[i];

      if (tcb != NULL && tcb->group != NULL &&
          tcb->group->tg_ort_lpid == (uint32_t)pid &&
          tcb->group->tg_ort_root != NULL &&
          strncmp(tcb->group->tg_ort_root, root, ORT_PIDNS_ROOTLEN) == 0)
        {
          found = tcb->pid;
          break;
        }
    }

  leave_critical_section(flags);
  return found;
}

#endif /* CONFIG_ORT_CONTAINER */
