/****************************************************************************
 * sched/task/ort_pidns.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * [ORT-A §98] 容器 pid 命名空间**第一刀**（自省面）：
 *
 *   某组**首次**进入命名空间时取一个本地号（1 起递增），此后：
 *     · getpid（用户态读 tg_info->ta_pid）→ 本地号；
 *     · getppid → 父组同命名空间 ⇒ 父组本地号；否则 0（照 POSIX
 *       命名空间口径：命名空间外的父 = 0）；
 *     · kill/tgkill/sigqueue 的 pid 参数**本地号优先**解析——命中本
 *       命名空间成员即换成全局号走原路；未命中按原全局号走原闸
 *       （跨组 → §96③ 一样 -EPERM）。
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
 *   边界如实：无 retains（NOCLDWAIT）路径回传全局号。
 *
 *   ---- 第三刀（§102）：**命名空间实例化** ----
 *   §98 起命名空间以 **root 串**为键（"同 root 全部任务组"）—— 同 root
 *   的**并发多容器共用**一个空间（各自 init 成了 1,2,3…，不是各自 1）。
 *   本刀改为**实例键**（tg_ort_nsid，全局单调）：
 *     · **设根者派生 = 新实例**（entrypoint 恒为本地 1）；
 *     · 容器成员的派生 = 继承父实例（内部 spawn/线程照旧）。
 *   号解析（resolve/localize/wait_localize）按 **nsid 相等** 匹配，
 *   不再比 root 串。getpid/getppid/号面语义与第一二刀逐字不变；
 *   变化只在"同 root 并发"这一格（由"共用空间"修正为"各自空间"）。
 *   主线程 gettid 视图（tid==pid）在 tls_initinfo 处（见该文件）；
 *   pthread 线程号面仍全局（边界，与 tgkill 的 tid 面留后续刀）。
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_ORT_CONTAINER

#include <stdint.h>
#include <stdbool.h>
#include <nuttx/sched.h>
#include <nuttx/kmalloc.h>
#include <nuttx/debug.h>

#include "sched/sched.h"
#include "task/task.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ORT_PIDNS_MAX     8       /* 同时存在的容器命名空间实例上限（原型） */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ort_pidns_s
{
  uint32_t pd_nsid;                     /* 实例号（全局单调，非 0） */
  uint32_t pd_next;                     /* 下一个本地号（1 起） */
  uint16_t pd_refs;                     /* 引用（组数）计数 */
  bool     pd_used;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct ort_pidns_s g_ort_pidns[ORT_PIDNS_MAX];
static uint32_t g_ort_nsid_next;        /* 实例号发号器（只增不复用 ——
                                         * 陈旧号绝不与后来者混淆） */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ort_pidns_new
 *
 * Description:
 *   [§102] 设根者派生 = **新容器实例**：建实例、取首个本地号（=1 的
 *   不变量由此保证）、回填实例号。表满返回 0（调用方按全局语义降级，
 *   fail-open —— 不制造半隔离）。
 *
 ****************************************************************************/

uint32_t ort_pidns_new(FAR uint32_t *nsid_out)
{
  irqstate_t flags;
  int i;
  int free = -1;
  uint32_t lpid = 0;

  flags = enter_critical_section();

  for (i = 0; i < ORT_PIDNS_MAX; i++)
    {
      if (!g_ort_pidns[i].pd_used)
        {
          free = i;
          break;
        }
    }

  if (free >= 0)
    {
      g_ort_pidns[free].pd_used = true;
      g_ort_pidns[free].pd_nsid = ++g_ort_nsid_next;
      g_ort_pidns[free].pd_refs = 1;
      lpid = ++g_ort_pidns[free].pd_next;    /* 前置自增：0 是哨兵 */
      *nsid_out = g_ort_pidns[free].pd_nsid;
    }

  leave_critical_section(flags);
  return lpid;
}

/****************************************************************************
 * Name: ort_pidns_join
 *
 * Description:
 *   [§102] 容器成员的派生 = 加入父实例：按实例号取号。未命中返回 0
 *  （调用方按全局语义降级）。
 *
 ****************************************************************************/

uint32_t ort_pidns_join(uint32_t nsid)
{
  irqstate_t flags;
  uint32_t lpid = 0;
  int i;

  if (nsid == 0)
    {
      return 0;
    }

  flags = enter_critical_section();

  for (i = 0; i < ORT_PIDNS_MAX; i++)
    {
      if (g_ort_pidns[i].pd_used && g_ort_pidns[i].pd_nsid == nsid)
        {
          lpid = ++g_ort_pidns[i].pd_next;
          g_ort_pidns[i].pd_refs++;
          break;
        }
    }

  leave_critical_section(flags);
  return lpid;
}

/****************************************************************************
 * Name: ort_pidns_put
 *
 * Description:
 *   组离开命名空间（组销毁时）。最后一个引用离开即回收实例。
 *
 ****************************************************************************/

void ort_pidns_put(uint32_t nsid)
{
  irqstate_t flags;
  int i;

  if (nsid == 0)
    {
      return;
    }

  flags = enter_critical_section();

  for (i = 0; i < ORT_PIDNS_MAX; i++)
    {
      if (g_ort_pidns[i].pd_used && g_ort_pidns[i].pd_nsid == nsid)
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
 *   信号入口（kill/tgkill/sigqueue）的号解析：调用者是容器成员且
 *   pid>0 命中**本实例**成员的本地号 ⇒ 换成其全局 pid；否则原样返回
 *   （后续按原全局号走既有闸 —— 跨组 -EPERM 不变）。遍历按 PIDHASH
 *   （任务数小，临界区内直扫）。
 *
 *   容器谓词 = §86/§96③ 同款 **tg_ort_re_root**（§99 r1 实锤教训：
 *   "root 非空"会把设根者 orting 自己算进来）。
 *
 ****************************************************************************/

pid_t ort_pid_resolve(pid_t pid)
{
  FAR struct tcb_s *rtcb = this_task();
  irqstate_t flags;
  pid_t found = pid;
  int i;

  if (pid <= 0 || rtcb == NULL || rtcb->group == NULL)
    {
      return pid;
    }

  if (!rtcb->group->tg_ort_re_root || rtcb->group->tg_ort_nsid == 0)
    {
      return pid;                 /* 非容器成员：全局语义原样 */
    }

  flags = enter_critical_section();

  /* PIDHASH 表直扫（pid 唯一、无冲突；表大小即上界 —— 同
   * nxsched_get_tcb 的口径） */

  for (i = 0; i < g_npidhash; i++)
    {
      FAR struct tcb_s *tcb = g_pidhash[i];

      if (tcb != NULL && tcb->group != NULL &&
          tcb->group->tg_ort_nsid == rtcb->group->tg_ort_nsid &&
          tcb->group->tg_ort_lpid == (uint32_t)pid)
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
 *   **全局号 → 本地号**（活体查表版）：spawn 族把子进程号交还调用者时
 *   用（子 tcb 尚在）。谓词：调用者是容器成员，且子组在**同一实例**
 *   且有本地号。任一不满足原样返回 —— 容器外/内核调用点零影响。
 *
 ****************************************************************************/

pid_t ort_pid_localize(pid_t gpid)
{
  FAR struct tcb_s *rtcb = this_task();
  FAR struct tcb_s *ctcb;
  irqstate_t flags;

  if (gpid <= 0 || rtcb == NULL || rtcb->group == NULL)
    {
      return gpid;
    }

  if (!rtcb->group->tg_ort_re_root || rtcb->group->tg_ort_nsid == 0)
    {
      return gpid;
    }

  flags = enter_critical_section();
  ctcb = nxsched_get_tcb(gpid);
  if (ctcb != NULL && ctcb->group != NULL &&
      ctcb->group->tg_ort_nsid == rtcb->group->tg_ort_nsid &&
      ctcb->group->tg_ort_lpid != 0)
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
 *   **全局号 → 本地号**（见证号版）：waitpid 回收时子 tcb 已亡，查不了
 *   表 —— 本地号来自子组**创建时**留存的 ch_ort_lpid（nxtask_save_parent
 *   处填，随 child_status 条目在 reparent 时一并迁移）。谓词：调用者是
 *   容器成员且见证号非 0（0 = 子未曾入命名空间 / 见证缺失）。
 *
 ****************************************************************************/

pid_t ort_wait_localize(pid_t gpid, uint32_t lpid)
{
  FAR struct tcb_s *rtcb = this_task();

  if (lpid == 0 || rtcb == NULL || rtcb->group == NULL)
    {
      return gpid;
    }

  if (!rtcb->group->tg_ort_re_root || rtcb->group->tg_ort_nsid == 0)
    {
      return gpid;
    }

  return (pid_t)lpid;
}

#endif /* CONFIG_ORT_CONTAINER */
