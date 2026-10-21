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
#include <stdio.h>
#include <string.h>
#include <nuttx/sched.h>
#include <nuttx/kmalloc.h>
#include <nuttx/debug.h>
#include <nuttx/mqueue.h>
#include <nuttx/wqueue.h>

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

/* [§117] M 域键 IPC 命名空间：每域组引用 + 代际。enc 取值 1..255
 * （tg_ort_domain 的编码）。新绑定 gen++；延迟 GC worker 只在代际未变
 * （期间没有新实例绑定该域）时执行。 */

#define ORT_IPC_DOMAIN_MAX 256

static uint16_t g_ipc_dom_refs[ORT_IPC_DOMAIN_MAX];
static uint32_t g_ipc_dom_gen[ORT_IPC_DOMAIN_MAX];

struct ort_dom_gc_s
{
  struct work_s work;
  uint8_t  enc;
  uint32_t gen;
};

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
 * Returned Value:
 *   true = 本调用释放了实例的**最后一个**引用（[§115] 调用者可据此
 *   回收该实例的 mqueue 命名空间）；false = 还有别的组在实例内。
 *
 ****************************************************************************/

bool ort_pidns_put(uint32_t nsid)
{
  irqstate_t flags;
  bool last = false;
  int i;

  if (nsid == 0)
    {
      return false;
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
              last = true;
            }

          break;
        }
    }

  leave_critical_section(flags);
  return last;
}

/****************************************************************************
 * Name: ort_pidns_next_tid
 *
 * Description:
 *   [§109] 给 pthread **线程**分配实例内本地 tid（与 lpid **共用同一
 *   计数器** —— Linux 口径：tid 与 pid 同空间）。表满/未入返回 0
 *   （调用方降级为全局号，fail-open）。
 *
 ****************************************************************************/

uint32_t ort_pidns_next_tid(uint32_t nsid)
{
  irqstate_t flags;
  uint32_t tid = 0;
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
          tid = ++g_ort_pidns[i].pd_next;
          break;
        }
    }

  leave_critical_section(flags);
  return tid;
}

/****************************************************************************
 * Name: ort_pid_resolve
 *
 * Description:
 *   号解析（信号三入口 + 调度入口 + pthread 句柄面共用）：调用者是
 *   容器成员且 pid>0 命中**本实例**成员的号 ⇒ 换成其全局 pid；
 *   否则原样返回（后续按原全局号走既有闸 —— 跨组 -EPERM 不变）。
 *   遍历按 PIDHASH（任务数小，临界区内直扫）。
 *
 *   [§109] 匹配面扩到 **tcb 级**：组的本地号（组长/进程号）**或**
 *   线程的本地 tid（pthread 句柄）—— 句柄面（pthread_join/kill/
 *   detach 的 pthread_t、调度入口的 tid 参数、tgkill 的 tid 参数）
 *   因此统一走这一处。
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
   * nxsched_get_tcb 的口径）。组号与线程号同空间（同一计数器），
   * 二者不会撞号。 */

  for (i = 0; i < g_npidhash; i++)
    {
      FAR struct tcb_s *tcb = g_pidhash[i];

      if (tcb != NULL && tcb->group != NULL &&
          tcb->group->tg_ort_nsid == rtcb->group->tg_ort_nsid &&
          (tcb->group->tg_ort_lpid == (uint32_t)pid ||
           tcb->ort_ltid == (uint32_t)pid))
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

/****************************************************************************
 * Name: ort_tid_localize
 *
 * Description:
 *   [§109] **全局号 → 本地线程号**（活体查表，tcb 级 ort_ltid 版）：
 *   pthread_create 把新线程的句柄交还调用者时用。组长 tcb 的
 *   ort_ltid = 组 lpid（同一计数器），所以对组长返回值与组本地号一致。
 *   任一谓词不过原样返回 —— 容器外零影响。
 *
 ****************************************************************************/

pid_t ort_tid_localize(pid_t gpid)
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
      ctcb->ort_ltid != 0)
    {
      gpid = (pid_t)ctcb->ort_ltid;
    }

  leave_critical_section(flags);
  return gpid;
}

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

/****************************************************************************
 * [§117] M 域键 IPC 命名空间的生命周期（组引用 + 延迟 GC）
 *
 *   域键 `ort.d<enc>.` 在 CG 重启时复用（§114 绑定前清理保证不跨实例
 *   泄漏），但旧队列会留到"下一次绑定该域"。本机制给每个域记**组引用**
 *   （绑域 +1、§113 继承 +1、组销毁 -1）：归零 = 实例彻底死亡 → LPWORK
 *   异步清命名空间。新绑定先 gen++，使此前排队的 worker 跳过（绑定自身
 *   的 §114 清理已把旧队清掉），避免延迟清理误伤新实例。
 *
 *   enc 即 tg_ort_domain（0 = 未绑；n>0 = 域 n-1）。
 ****************************************************************************/

static void ort_ipc_domain_gc_worker(FAR void *arg)
{
  FAR struct ort_dom_gc_s *p = (FAR struct ort_dom_gc_s *)arg;
  irqstate_t flags;
  bool current;
  char prefix[32];

  flags = enter_critical_section();
  current = (g_ipc_dom_gen[p->enc] == p->gen);
  leave_critical_section(flags);

  if (current && p->enc != 0)
    {
      snprintf(prefix, sizeof(prefix), "ort.d%u.", (unsigned)p->enc);
      nxmq_purge_prefix(prefix);
    }

  kmm_free(p);
}

void ort_ipc_domain_bind(uint8_t enc)
{
  irqstate_t flags;

  if (enc == 0)
    {
      return;
    }

  flags = enter_critical_section();
  g_ipc_dom_gen[enc]++;
  if (g_ipc_dom_refs[enc] < 0xffff)
    {
      g_ipc_dom_refs[enc]++;
    }

  leave_critical_section(flags);
}

void ort_ipc_domain_ref(uint8_t enc)
{
  irqstate_t flags;

  if (enc == 0)
    {
      return;
    }

  flags = enter_critical_section();
  if (g_ipc_dom_refs[enc] < 0xffff)
    {
      g_ipc_dom_refs[enc]++;
    }

  leave_critical_section(flags);
}

void ort_ipc_domain_unref(uint8_t enc)
{
  FAR struct ort_dom_gc_s *p;
  irqstate_t flags;
  bool zero = false;
  uint32_t gen = 0;

  if (enc == 0)
    {
      return;
    }

  flags = enter_critical_section();
  if (g_ipc_dom_refs[enc] > 0 && --g_ipc_dom_refs[enc] == 0)
    {
      zero = true;
      gen = g_ipc_dom_gen[enc];
    }

  leave_critical_section(flags);

  if (!zero)
    {
      return;
    }

  /* LPWORK 在未配置低优先队列的 SKU 上被 wqueue.h 重定向到 HPWORK
   * （同一 qid 语义），两 SKU 都可用。 */

  p = kmm_malloc(sizeof(*p));
  if (p == NULL)
    {
      _alert("ORT: ipc domain gc alloc failed: d%u (leak)\n",
             (unsigned)enc);
      return;
    }

  memset(p, 0, sizeof(*p));
  p->enc = enc;
  p->gen = gen;

  if (work_queue(LPWORK, &p->work, ort_ipc_domain_gc_worker, p, 0) < 0)
    {
      _alert("ORT: ipc domain gc queue failed: d%u (leak)\n",
             (unsigned)enc);
      kmm_free(p);
    }
}

#endif /* CONFIG_ORT_CONTAINER */
