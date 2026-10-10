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
 *   ---- 第二刀（§99）：**进程管理号面**收进命名空间 ----
 *     · spawn 回传号：exec_spawn 出口本地化（活体查表，见
 *       ort_pid_localize）—— 容器内 spawn 拿到的就是本地号；
 *     · waitpid：入口本地号优先解析（ort_pid_resolve 复用）；回收
 *       号回传本地化 —— 子 tcb 回收时已亡，本地号靠**创建时留存**
 *       在 child_status 的 ch_ort_lpid（nxtask_save_parent 处填），
 *       见 ort_wait_localize。
 *   两向都带"调用者是**容器成员**"谓词 —— 谓词 = §86/§96③ 同款
 *   **tg_ort_re_root**（不是"root 非空"：设根者 orting 自己带 root
 *   但 re_root=false，必须保持全局语义；§99 r1 实锤错谓词把 orting
 *   的 spawn 记账换成"本地 1"）。谓词不过一律原样，零影响。
 *   边界如实：无 retains（NOCLDWAIT）路径回传全局号；线程号面
 *   （gettid）仍全局。
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

  /* [§99] 容器谓词 = §86/§96③ 同款：**已重挂（tg_ort_re_root）**才算
   * 容器成员。只用"root 非空"会把**设根者自己**（orting：带 root 传播
   * 给孩子、但 re_root=false 走全局路径）也算进来 —— §99 r1 实锤：
   * orting 读 spawn 回传号被换成子本地号 1（25 处 `spawned pid=1`），
   * 故障线段配对全线崩。 */

  if (!rtcb->group->tg_ort_re_root)
    {
      return pid;                 /* 非容器成员：全局语义原样 */
    }

  root = rtcb->group->tg_ort_root;
  if (root == NULL || root[0] == '\0')
    {
      return pid;                 /* 保险：成员必带 root */
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

/****************************************************************************
 * Name: ort_pid_localize
 *
 * Description:
 *   [§99 第二刀] **全局号 → 本地号**（活体查表版）：spawn 族把子进程号
 *   交还调用者时用（子 tcb 尚在）。谓词与 ort_pid_resolve 对偶：调用者
 *   是**容器成员（tg_ort_re_root）**且子组同源（root 相同）且子组有
 *   本地号。任一不满足原样返回 —— 容器外/内核调用点零影响。
 *
 ****************************************************************************/

pid_t ort_pid_localize(pid_t gpid)
{
  FAR struct tcb_s *rtcb = this_task();
  FAR struct tcb_s *ctcb;
  FAR const char *root;
  irqstate_t flags;

  if (gpid <= 0 || rtcb == NULL || rtcb->group == NULL)
    {
      return gpid;
    }

  /* [§99] 谓词同 resolve：只认**已重挂**的容器成员（设根者 orting 自己
   * 不算 —— 它 spawn 容器后要读全局号做记账）。 */

  if (!rtcb->group->tg_ort_re_root)
    {
      return gpid;
    }

  root = rtcb->group->tg_ort_root;
  if (root == NULL || root[0] == '\0')
    {
      return gpid;
    }

  flags = enter_critical_section();
  ctcb = nxsched_get_tcb(gpid);
  if (ctcb != NULL && ctcb->group != NULL &&
      ctcb->group->tg_ort_lpid != 0 &&
      ctcb->group->tg_ort_root != NULL &&
      strncmp(ctcb->group->tg_ort_root, root, ORT_PIDNS_ROOTLEN) == 0)
    {
      gpid = (pid_t)ctcb->group->tg_ort_lpid;
    }

  leave_critical_section(flags);
  return gpid;
}

/****************************************************************************
 * Name: ort_wait_localize
 *
 * Description:
 *   [§99 第二刀] **全局号 → 本地号**（见证号版）：waitpid 回收时子 tcb
 *   已亡，查不了表 —— 本地号来自子组**创建时**留存的 ch_ort_lpid
 *  （nxtask_save_parent 处填，随 child_status 条目在 reparent 时一并
 *   迁移）。谓词：调用者是**容器成员（tg_ort_re_root）**且见证号非 0
 *  （0 = 子未曾入命名空间 / 见证缺失），否则原样返回。
 *
 *   注：不另比子 root —— 子组 root 随派生继承天然同源；reparent 只把
 *   孩子交给 init/监督者（root 空），那一路本函数不生效（调用者在
 *   空间外）。
 *
 ****************************************************************************/

pid_t ort_wait_localize(pid_t gpid, uint32_t lpid)
{
  FAR struct tcb_s *rtcb = this_task();

  if (lpid == 0 || rtcb == NULL || rtcb->group == NULL)
    {
      return gpid;
    }

  /* [§99] 谓词同 resolve：只认**已重挂**的容器成员 —— 设根者 orting
   * 回收容器时按全局号回传（§99 r1 实锤：错谓词下 orting 拿回本地 1）。 */

  if (!rtcb->group->tg_ort_re_root)
    {
      return gpid;
    }

  return (pid_t)lpid;
}

#endif /* CONFIG_ORT_CONTAINER */
