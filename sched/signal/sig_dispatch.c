/****************************************************************************
 * sched/signal/sig_dispatch.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <inttypes.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/debug.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/signal.h>
#include <nuttx/queue.h>

#include "sched/sched.h"
#include "group/group.h"
#include "semaphore/semaphore.h"
#include "signal/signal.h"
#include "mqueue/mqueue.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_ENABLE_ALL_SIGNALS
struct sig_arg_s
{
  pid_t pid;
  bool need_restore;
};

#ifdef CONFIG_SMP
/* [ORT §79 常驻烙印记] 每次 up_schedule_sigaction 的来源点（site）/CPU/
 * 目标 pid —— 与 arm_syscall.c 的帧形态绊线配套：绊线复响时直接带
 * 四元组（帧态/臂来源）进排查，不再重新设计装置。 */

volatile uint32_t g_ort_arm_site;
volatile int      g_ort_arm_cpu;
volatile int      g_ort_arm_pid;

/* [ORT] 异步臂装载的静态槽（每目标 CPU 一套）—— 手册 §三·补七十九。
 *
 *   为什么需要它们：**ISR/异常上下文**里向"另一核上 RUNNING"的任务投
 *   信号，不能走同步 nxsched_smp_call_single()（其契约禁止在中断上下文
 *   等待，sched_smp.c 的 DEBUGASSERT）；而"延迟到目标自己的下次内核
 *   边界"也不行 —— **用户在跑的任务 xcp.regs 常态是 NULL 哨兵**
 *   （arm_doirq/arm_syscall 返回路径明写 "about to become invalid"），
 *   对 NULL 做帧搬运会访存 NULL−XCPTCONTEXT_SIZE（实测 DFAR=0xfffffeb8
 *   → panic）。正解 = **异步** smp_call：payload 仍然在**目标核的 IRQ
 *   上下文**里跑 —— 那时目标核刚把 xcp.regs 设成它的活帧（arm_doirq
 *   入口），搬运合法；而发送方**不等**，ISR 合法。
 *
 *   在途不变量（原型）：同一任务由 stcb->sigdeliver 守卫串行；跨任务
 *   同目标 CPU 的并发由 busy 标志 fail-safe（静默跳过 —— 唤醒是尽力
 *   而为的通知，故障事件本体在队列里，不靠唤醒送达）。本原型的异步
 *   目标是监督者（单任务），不变量成立。
 */
static struct smp_call_data_s g_ort_sig_call[CONFIG_SMP_NCPUS];
static struct sig_arg_s      g_ort_sig_arg[CONFIG_SMP_NCPUS];
static bool                  g_ort_sig_busy[CONFIG_SMP_NCPUS];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_SMP
static int sig_handler(FAR void *cookie)
{
  FAR struct sig_arg_s *arg = cookie;
  FAR struct tcb_s *tcb;
  irqstate_t flags;

  flags = enter_critical_section();

#ifdef CONFIG_SMP
  /* [ORT] §79：归还异步臂装载的静态槽（同步路径无副作用——
   * 幂等）。放在存活检查**之前**：目标中途死掉也不留 busy 悬挂。 */

  g_ort_sig_busy[this_cpu()] = false;
#endif

  tcb = nxsched_get_tcb(arg->pid);

  if (!tcb || tcb->task_state == TSTATE_TASK_INVALID ||
      (tcb->flags & TCB_FLAG_EXIT_PROCESSING) != 0)
    {
      /* There is no TCB with this pid or, if there is, it is not a task. */

      leave_critical_section(flags);
      return -ESRCH;
    }

  if (arg->need_restore)
    {
      tcb->flags &= ~TCB_FLAG_CPU_LOCKED;
    }

  if (tcb->sigdeliver)
    {
      /* [ORT §79 缺陷②] 臂装载前校验：**当前帧必须是用户态帧**。
       *
       *   本载荷可能在目标**正处于投递装置内部**（SYS 模式、专属栈）
       *   时执行 —— 那时 xcp.regs 是投递帧（SP=专属栈/内核区），把它
       *   当用户帧搬运会让用户处理器的调用帧建到内核区，用户 stub
       *   （arm_signal_handler.S）首条 `push {lr}` 即触 MMU 权限拒。
       *   实测（缩比装置）：bad usp=0x40058110 saved_cpsr=0x…7f(SYS)。
       *   （双载荷为什么会出现：sig_dispatch 的 `!sigdeliver` 守卫是
       *   **跨核非原子的 check-then-set** —— ISR 臂（异步载荷）与
       *   EXIT 事件臂（同步载荷）在风暴里可同时过闸。）
       *   非用户帧 → 跳过臂装载：信号留在队列（尽力而为语义；故障
       *   事件本体在队列里，不靠唤醒送达）。 */

      if (tcb->xcp.regs != NULL &&
          (tcb->xcp.regs[REG_CPSR] & 0x1f) == 0x10)   /* PSR_MODE_USR */
        {
          g_ort_arm_site = 1;      /* [ORT §79] sig_handler 载荷 */
          g_ort_arm_cpu  = this_cpu();
          g_ort_arm_pid  = tcb->pid;
          up_schedule_sigaction(tcb);
        }
    }

  leave_critical_section(flags);
  return OK;
}
#endif

/****************************************************************************
 * Name: nxsig_queue_action
 *
 * Description:
 *   Queue a signal action for delivery to a task.
 *
 * Returned Value:
 *   Returns 0 (OK) on success or a negated errno value on failure.
 *
 * Assumptions:
 *   Called in critical section
 *
 ****************************************************************************/

static int nxsig_queue_action(FAR struct tcb_s *stcb,
                              FAR sigactq_t *sigact,
                              FAR siginfo_t *info)
{
  FAR sigq_t    *sigq;
  int            ret = OK;

  DEBUGASSERT(stcb != NULL && stcb->group != NULL);

  /* Check if a valid signal handler is available and if the signal is
   * unblocked. NOTE: There is no default action.
   */

  if ((sigact) && (sigact->act.sa_u._sa_sigaction))
    {
      /* Allocate a new element for the signal queue. NOTE:
       * nxsig_alloc_pendingsigaction will force a system crash if it is
       * unable to allocate memory for the signal data.
       */

      sigq = nxsig_alloc_pendingsigaction();
      if (!sigq)
        {
          ret = -ENOMEM;
        }
      else
        {
          /* Populate the new signal queue element */

          sigq->action.sighandler = sigact->act.sa_u._sa_sigaction;
          sigq->mask = sigact->act.sa_mask;
          if ((sigact->act.sa_flags & SA_NODEFER) == 0)
            {
              sigaddset(&sigq->mask, info->si_signo);
            }

          memcpy(&sigq->info, info, sizeof(siginfo_t));
          sigq->info.si_user = sigact->act.sa_user;

          /* Put it at the end of the pending signals list */

          sq_addlast((FAR sq_entry_t *)sigq, &(stcb->sigpendactionq));

          /* Then schedule execution of the signal handling action on the
           * recipient's thread. SMP related handling will be done in
           * up_schedule_sigaction()
           */

          if (!stcb->sigdeliver)
            {
#ifdef CONFIG_SMP
              int cpu = stcb->cpu;
              int me  = this_cpu();

              /* [ORT] 上游缺陷本地修复（第 4 处，手册 §三·补七十六）：
               *
               *   目标任务正在**另一个核**上 RUNNING 时，这里走**同步**
               *   nxsched_smp_call_single() —— 而它有一个明写的契约
               *   "Cannot wait in interrupt context"（sched_smp.c 的
               *   DEBUGASSERT(!up_interrupt_context())）。从**中断/
               *   异常上下文**投信号（如 ORT 的收容路径在 abort 向量里
               *   通知监督者）踩中该断言；release 构建下更糟：会在
               *   ISR 里 nxsem_wait 死等。
               *
               *   触发实测（§76 并发容器）：4 个容器同时在 4 个核上跑，
               *   监督者自己也在跑——早崩容器的"醒信号"投给它时正好
               *   命中"跨核 RUNNING"分支 → 断言（serial 流程不炸，因为
               *   监督者那时阻塞在 waitpid、不走同步分支）。
               *
               *   修法：中断上下文里不走同步 smp_call，落回**延迟投递**
               *   （up_schedule_sigaction 只搬运寄存器上下文，ISR 安全；
               *   投递在目标的下一处内核边界发生——对"醒信号"这类
               *   尽力而为的通知语义足够）。 */

              stcb->sigdeliver = nxsig_deliver;
              if (cpu != me && stcb->task_state == TSTATE_TASK_RUNNING)
                {
                  if (!up_interrupt_context())
                    {
                      struct sig_arg_s arg;

                      if ((stcb->flags & TCB_FLAG_CPU_LOCKED) != 0)
                        {
                          arg.need_restore   = false;
                        }
                      else
                        {
                          arg.need_restore   = true;
                          stcb->flags        |= TCB_FLAG_CPU_LOCKED;
                        }

                      arg.pid = stcb->pid;
                      nxsched_smp_call_single(stcb->cpu, sig_handler, &arg);
                    }
                  else if (!g_ort_sig_busy[cpu])
                    {
                      /* [ORT] ISR 上下文：异步臂装载（详见文件头的
                       * 静态槽说明）。payload 在目标核 IRQ 上下文跑，
                       * 搬运借它刚设好的活帧；发送方不等。 */

                      g_ort_sig_busy[cpu]             = true;
                      g_ort_sig_arg[cpu].pid          = stcb->pid;
                      g_ort_sig_arg[cpu].need_restore = true;
                      stcb->flags                    |= TCB_FLAG_CPU_LOCKED;
                      nxsched_smp_call_init(&g_ort_sig_call[cpu], sig_handler,
                                            &g_ort_sig_arg[cpu]);
                      nxsched_smp_call_single_async(cpu, &g_ort_sig_call[cpu]);
                    }
                }
              else
#endif
                {
                  stcb->sigdeliver = nxsig_deliver;
                  if (stcb == this_task() && !up_interrupt_context())
                    {
                      /* In this case just deliver the signal now. */

                      (stcb->sigdeliver)(stcb);
                      stcb->sigdeliver = NULL;
                    }
                  else
                    {
#ifdef CONFIG_SMP
                      /* [ORT §79] else 分支直呼（诊断戳只在 SMP 构建存在
                       * —— 2026-10-09 §86 补编 M 时发现：非 SMP 分支
                       * 也在编，三个戳变量却是 CONFIG_SMP 守卫定义的，
                       * M 两 SKU 自 §79 起编译不过。A 侧真值分支逐字
                       * 保留，语义零变化） */

                      g_ort_arm_site = 2;
                      g_ort_arm_cpu  = this_cpu();
                      g_ort_arm_pid  = stcb->pid;
#endif
                      up_schedule_sigaction(stcb);
                    }
                }
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Name: nxsig_dispatch_kernel_action
 ****************************************************************************/

static void nxsig_dispatch_kernel_action(FAR struct tcb_s *stcb,
                                         FAR siginfo_t *info)
{
  FAR struct task_group_s *group = stcb->group;
  FAR sigactq_t *sigact;

  sigact = nxsig_find_action(group, info->si_signo);
  if (sigact && (sigact->act.sa_flags & SA_KERNELHAND))
    {
      info->si_user = sigact->act.sa_user;
      (sigact->act.sa_sigaction)(info->si_signo, info, NULL);
    }
}
#endif /* CONFIG_ENABLE_ALL_SIGNALS */

/****************************************************************************
 * [ORT §87] 信号面隔离：容器只准向**自己组**投信号。
 *
 *   谁算容器：tg_ort_re_root（§86 起，binfmt 派生传播时置位的"容器
 *   标记"）。非容器（监督者/init/nsh/内核线程组）全程原语义。
 *
 *   为什么三条入口都要过它：kill/tgkill/sigqueue 是用户态仅有的三条
 *   投递路径（POSIX 语义允许"任何进程给任何进程发信号"，容器化后
 *   这正是要收掉的：容器不能被拿去打监督者、init、别的容器）。
 *
 *   内核内部不经此闸：故障/退出唤醒已改**直投**（arm_ortcommon 的
 *   ort_wake_supervisor），定时器/AIO 的目标天然同组。返回语义与
 *   原路径对齐：目标不存在 → -ESRCH；跨组 → -EPERM。
 ****************************************************************************/

#if defined(CONFIG_ORT_CONTAINER) && defined(CONFIG_BUILD_KERNEL)
int ort_sig_gate(pid_t pid)
{
  FAR struct tcb_s *rtcb = this_task();
  FAR struct tcb_s *stcb;

  if (rtcb == NULL || rtcb->group == NULL ||
      !rtcb->group->tg_ort_re_root)
    {
      return OK;
    }

  stcb = nxsched_get_tcb(pid);
  if (stcb == NULL)
    {
      return -ESRCH;
    }

  return stcb->group == rtcb->group ? OK : -EPERM;
}
#endif

/****************************************************************************
 * Name: nxsig_alloc_pendingsignal
 *
 * Description:
 *   Allocate a pending signal list entry
 *
 * Assumptions:
 *   Called with g_sigpendingsignal locked
 *
 ****************************************************************************/

static FAR sigpendq_t *nxsig_alloc_pendingsignal(void)
{
  FAR sigpendq_t *sigpend;

  /* Try to get the pending signal structure from the free list */

  sigpend = (FAR sigpendq_t *)sq_remfirst(&g_sigpendingsignal);
  if (!sigpend && up_interrupt_context())
    {
      /* If no pending signal structure is available in the free list,
       * then try the special list of structures reserved for
       * interrupt handlers
       */

      sigpend = (FAR sigpendq_t *)sq_remfirst(&g_sigpendingirqsignal);
    }

  return sigpend;
}

/****************************************************************************
 * Name: nxsig_find_pendingsignal
 *
 * Description:
 *   Find a specified element in the pending signal list
 *
 * Assumptions:
 *   Called with group->tg_sigpendingq locked
 *
 ****************************************************************************/

static FAR sigpendq_t *
nxsig_find_pendingsignal(FAR struct task_group_s *group, int signo)
{
  FAR sigpendq_t *sigpend = NULL;
  irqstate_t flags;

  DEBUGASSERT(group != NULL);

  /* Determining whether a signal is reliable or unreliable */

  if (SIGRTMIN <= signo && signo <= SIGRTMAX)
    {
      return sigpend;
    }

  /* Pending signals can be added from interrupt level. */

  flags = spin_lock_irqsave(&group->tg_lock);

  /* Search the list for a action pending on this signal */

  for (sigpend = (FAR sigpendq_t *)group->tg_sigpendingq.head;
       (sigpend && sigpend->info.si_signo != signo);
       sigpend = sigpend->flink);

  spin_unlock_irqrestore(&group->tg_lock, flags);
  return sigpend;
}

/****************************************************************************
 * Name: nxsig_add_pendingsignal
 *
 * Description:
 *   Add the specified signal to the signal pending list. NOTE: This
 *   function will queue only one entry for each pending signal. This
 *   was done intentionally so that a run-away sender cannot consume
 *   all of memory.
 *
 * Assumptions:
 *   Called with tg_sigpendingq locked
 *
 ****************************************************************************/

static FAR sigpendq_t *nxsig_add_pendingsignal(FAR struct tcb_s *stcb,
                                               FAR siginfo_t *info,
                                               bool group_dispatch)
{
  FAR struct task_group_s *group;
  FAR sigpendq_t *sigpend;
  irqstate_t flags;

  DEBUGASSERT(stcb != NULL && stcb->group != NULL);
  group = stcb->group;

  /* Check if the signal is already pending for the group */

  sigpend = nxsig_find_pendingsignal(group, info->si_signo);
  if (sigpend != NULL)
    {
      /* The signal is already pending... retain only one copy */

      memcpy(&sigpend->info, info, sizeof(siginfo_t));
    }

  /* No... There is nothing pending in the group for this signo */

  else
    {
      /* Allocate a new pending signal entry */

      sigpend = nxsig_alloc_pendingsignal();
      if (sigpend != NULL)
        {
          /* Put the signal information into the allocated structure */

          memcpy(&sigpend->info, info, sizeof(siginfo_t));

          /* Mark the tcb which need to receive the signal. If any
           * thread in the group may receive it, set it to NULL
           */

          sigpend->tcb = group_dispatch ? NULL : stcb;

          /* Add the structure to the group pending signal list */

          flags = spin_lock_irqsave(&group->tg_lock);
          sq_addlast((FAR sq_entry_t *)sigpend, &group->tg_sigpendingq);
          spin_unlock_irqrestore(&group->tg_lock, flags);
        }
    }

  DEBUGASSERT(sigpend);

  return sigpend;
}

/****************************************************************************
 * Name: nxsig_alloc_dyn_pending
 *
 * Description:
 *   Dynamically allocate more pending signal and pending sigaction
 *   structures, if there are no more left. Note that this leaves the
 *   the critical section for the allocation. During that time it is
 *   it is possible that structures are freed, or another signalling thread
 *   allocates more structures. This is not an issue, any extra pending
 *   structures are freed after they get used.
 *
 * Assumptions:
 *   Called with g_sigpendingsignal and g_sigpendingaction locked by
 *   critical section.
 *
 ****************************************************************************/

static int nxsig_alloc_dyn_pending(FAR irqstate_t *flags)
{
  int ret = OK;
  bool alloc_signal = sq_empty(&g_sigpendingsignal);
#ifdef CONFIG_ENABLE_ALL_SIGNALS
  bool alloc_sigact = sq_empty(&g_sigpendingaction);

  if (alloc_signal || alloc_sigact)
#else
  if (alloc_signal)
#endif
    {
      FAR sigpendq_t *sigpend = NULL;
#ifdef CONFIG_ENABLE_ALL_SIGNALS
      FAR sigq_t *sigq = NULL;
#endif

      /* We can't do memory allocations in idle task or interrupt */

      if (up_interrupt_context() || sched_idletask())
        {
          return -EAGAIN;
        }

      /* Leave critical section for the duration of heap operations */

      leave_critical_section(*flags);

      /* Allocate more pending signals if there are no more */

      if (alloc_signal)
        {
          sigpend = kmm_malloc(sizeof(sigpendq_t));
        }

#ifdef CONFIG_ENABLE_ALL_SIGNALS
      /* Allocate more pending signal actions if there are no more */

      if (alloc_sigact)
        {
          sigq = kmm_malloc(sizeof(sigq_t));
        }
#endif

      /* Restore critical section and add the allocated structures to
       * the free pending queues
       */

      *flags = enter_critical_section();

      if (alloc_signal)
        {
          if (sigpend)
            {
              sigpend->type = SIG_ALLOC_DYN;
              sq_addfirst((sq_entry_t *)sigpend, &g_sigpendingsignal);
            }
          else
            {
              ret = -EAGAIN;
            }
        }

#ifdef CONFIG_ENABLE_ALL_SIGNALS
      if (alloc_sigact)
        {
          if (sigq)
            {
              sigq->type = SIG_ALLOC_DYN;
              sq_addfirst((sq_entry_t *)sigq, &g_sigpendingaction);
            }
          else
            {
              ret = -EAGAIN;
            }
        }
#endif
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_tcbdispatch
 *
 * Description:
 *   All signals received the task (whatever the source) go through this
 *   function to be processed. This function is responsible for:
 *
 *   - Determining if the signal is blocked.
 *   - Queuing and dispatching signal actions
 *   - Unblocking tasks that are waiting for signals
 *   - Queuing pending signals.
 *
 *   This function will deliver the signal to the task associated with
 *   the specified TCB. This function should *not* typically be used
 *   to dispatch signals since it will *not* follow the group signal
 *   deliver algorithms.
 *
 * Returned Value:
 *   Returns 0 (OK) on success or a negated errno value on failure.
 *
 ****************************************************************************/

int nxsig_tcbdispatch(FAR struct tcb_s *stcb, siginfo_t *info,
                      bool group_dispatch)
{
  FAR struct tcb_s *rtcb = this_task();
  irqstate_t flags;
  int masked;
  int ret = OK;
  FAR sigpendq_t *sigpend = NULL;
#ifdef CONFIG_ENABLE_ALL_SIGNALS
  FAR sigactq_t *sigact;
#endif

  sinfo("TCB=%p pid=%d signo=%d code=%d value=%d masked=%s\n",
        stcb, stcb->pid, info->si_signo, info->si_code,
        info->si_value.sival_int,
        sigismember(&stcb->sigprocmask, info->si_signo) == 1 ? "YES" : "NO");

  DEBUGASSERT(stcb != NULL && info != NULL);

  /* Return ESRCH when thread was in exit processing */

  if ((stcb->flags & TCB_FLAG_EXIT_PROCESSING) != 0)
    {
      return -ESRCH;
    }

  /* Don't actually send a signal for signo 0. */

  if (info->si_signo == 0)
    {
      return OK;
    }

  /************************** MASKED SIGNAL ACTIONS *************************/

  flags = enter_critical_section();

  /* Make sure that there is always at least one sigpednq and sigq structure
   * available, in case one needs to be queued later. Note that this breaks
   * the critical section if it needs to allocate any new structures. So it
   * needs to be done here before using the task state or sigprocmask.
   */

  ret = nxsig_alloc_dyn_pending(&flags);
  if (ret < 0)
    {
      leave_critical_section(flags);
      return ret;
    }

  masked = nxsig_ismember(&stcb->sigprocmask, info->si_signo);

#ifdef CONFIG_LIB_SYSCALL
  /* Check if the signal is masked OR if the signal is received while we are
   * processing a system call -- in either case, it will be added to the
   * list of pending signals. Unmasked user signal actions will be deferred
   * while we process the system call.
   *
   * If a thread calls a blocking system call, the thread will still be
   * unblocked when the signal occurs (see OTHER SIGNAL HANDLING below), but
   * any associated user signal action will be deferred until the system
   * call returns. For example, if the application calls sem_wait(), the
   * following would occur:
   *
   *   1. System call entry logic will block user signal handling and call
   *      sem_wait() in kernel mode.
   *   2. sem_wait() will block,
   *   3. The receipt of the signal will cause any signal action to pend
   *      but will unblock sem_wait(),
   *   4. The sem_wait() system call will awaken and return EINTR,
   *   5. The pending signal action will occur after the sem_wait() system
   *      call returns to user mode.
   *
   * Syscall handlers (and logic-in-general within the OS) should not use
   * signal handlers.
   */

  if ((masked == 1) || (stcb->flags & TCB_FLAG_SYSCALL) != 0)
#else
  /* Check if the signal is masked. In that case, it will be added to the
   * list of pending signals.
   */

  if (masked == 1)
#endif
    {
      /* Check if the task is waiting for this pending signal. If so, then
       * unblock it. This must be performed in a critical section because
       * signals can be queued from the interrupt level.
       */

      if (stcb->task_state == TSTATE_WAIT_SIG &&
          (masked == 0 ||
           (nxsig_ismember(&stcb->sigwaitmask, info->si_signo) == 1)))
        {
          if (stcb->sigunbinfo != NULL)
            {
              memcpy(stcb->sigunbinfo, info, sizeof(siginfo_t));
            }

          sigemptyset(&stcb->sigwaitmask);
          wd_cancel(&stcb->waitdog);

          /* Remove the task from waiting list */

          dq_rem((FAR dq_entry_t *)stcb, list_waitingforsignal());

          /* Add the task to ready-to-run task list and
           * perform the context switch if one is needed
           */

          if (nxsched_add_readytorun(stcb))
            {
              up_switch_context(this_task(), rtcb);
            }

#if defined(CONFIG_LIB_SYSCALL) && defined(CONFIG_ENABLE_ALL_SIGNALS)
          /* Must also add signal action if in system call */

          if (masked == 0)
            {
              sigpend = nxsig_add_pendingsignal(stcb, info, group_dispatch);
            }
#endif
        }

      /* Its not one we are waiting for... Add it to the list of pending
       * signals.
       */

      else
        {
          sigpend = nxsig_add_pendingsignal(stcb, info, group_dispatch);
        }
    }

  /************************* UNMASKED SIGNAL ACTIONS ************************/

  else
    {
#ifdef CONFIG_ENABLE_ALL_SIGNALS
      /* Find if there is a group sigaction associated with this signal */

      sigact = nxsig_find_action(stcb->group, info->si_signo);

      /* Queue any sigaction's requested by this task. */

      ret = nxsig_queue_action(stcb, sigact, info);
#endif
      /* Deliver of the signal must be performed in a critical section */

      /* Check if the task is waiting for an unmasked signal. If so, then
       * unblock it. This must be performed in a critical section because
       * signals can be queued from the interrupt level.
       */

      if (stcb->task_state == TSTATE_WAIT_SIG)
        {
          if (stcb->sigunbinfo != NULL)
            {
              memcpy(stcb->sigunbinfo, info, sizeof(siginfo_t));
            }

          sigemptyset(&stcb->sigwaitmask);
          wd_cancel(&stcb->waitdog);

          /* Remove the task from waiting list */

          dq_rem((FAR dq_entry_t *)stcb, list_waitingforsignal());

          /* Add the task to ready-to-run task list and
           * perform the context switch if one is needed
           */

          if (nxsched_add_readytorun(stcb))
            {
              up_switch_context(this_task(), rtcb);
            }
        }

      /* If the task neither was waiting for the signal nor had a signal
       * handler attached to the signal, then the default action is
       * simply to ignore the signal
       */
    }

  /************************* OTHER SIGNAL HANDLING **************************/

  /* Performed only if the signal is unmasked. These actions also must
   * happen within a system call.
   */

  if (masked == 0)
    {
      /* If the task is blocked waiting for a semaphore, then that task must
       * be unblocked when a signal is received.
       */

      if (stcb->task_state == TSTATE_WAIT_SEM)
        {
          nxsem_wait_irq(stcb, EINTR);
        }

#if !defined(CONFIG_DISABLE_MQUEUE) || !defined(CONFIG_DISABLE_MQUEUE_SYSV)
      /* If the task is blocked waiting on a message queue, then that task
       * must be unblocked when a signal is received.
       */

      else if (stcb->task_state == TSTATE_WAIT_MQNOTEMPTY ||
          stcb->task_state == TSTATE_WAIT_MQNOTFULL)
        {
          nxmq_wait_irq(stcb, EINTR);
        }
#endif

#ifdef CONFIG_SIG_SIGSTOP_ACTION
      /* If the task was stopped by SIGSTOP or SIGTSTP, then unblock the task
       * if SIGCONT is received.
       */

      else if (stcb->task_state == TSTATE_TASK_STOPPED &&
          info->si_signo == SIGCONT)
        {
#ifdef HAVE_GROUP_MEMBERS
          group_continue(stcb);
#else
          /* Remove the task from waiting list */

          dq_rem((FAR dq_entry_t *)stcb, list_stoppedtasks());

          /* Add the task to ready-to-run task list and
           * perform the context switch if one is needed
           */

          if (nxsched_add_readytorun(stcb))
            {
              up_switch_context(this_task(), rtcb);
            }
#endif
        }
#endif
    }

  leave_critical_section(flags);

  /* Dispatch kernel action, if needed, in case a pending signal was added */

  if (sigpend != NULL)
    {
#ifdef CONFIG_ENABLE_ALL_SIGNALS
      nxsig_dispatch_kernel_action(stcb, &sigpend->info);
#endif
    }

  /* In case nxsig_ismember failed due to an invalid signal number */

  if (masked < 0)
    {
      ret = -EINVAL;
    }

  return ret;
}

/****************************************************************************
 * Name: nxsig_dispatch
 *
 * Description:
 *   This is the front-end for nxsig_tcbdispatch that should be typically
 *   be used to dispatch a signal. If HAVE_GROUP_MEMBERS is defined,
 *   then function will follow the group signal delivery algorithms:
 *
 *   This front-end does the following things before calling
 *   nxsig_tcbdispatch.
 *
 *     With HAVE_GROUP_MEMBERS defined:
 *     - Get the TCB associated with the pid.
 *     - If the TCB was found, get the group from the TCB.
 *     - If the PID has already exited, lookup the group that that was
 *       started by this task.
 *     - Use the group to pick the TCB to receive the signal
 *     - Call nxsig_tcbdispatch with the TCB
 *
 *     With HAVE_GROUP_MEMBERS *not* defined
 *     - Get the TCB associated with the pid.
 *     - Call nxsig_tcbdispatch with the TCB
 *
 * Returned Value:
 *   Returns 0 (OK) on success or a negated errno value on failure.
 *
 ****************************************************************************/

int nxsig_dispatch(pid_t pid, FAR siginfo_t *info, bool thread)
{
#ifdef HAVE_GROUP_MEMBERS
  if (!thread)
    {
      /* Find the group by process PID and call group signal() to send the
       * signal to the correct group member.
       */

      FAR struct task_group_s *group = task_getgroup(pid);
      if (group != NULL)
        {
          return group_signal(group, info);
        }
    }
  else
#endif
    {
      /* Get the TCB associated with the thread TID */

      FAR struct tcb_s *stcb = nxsched_get_tcb(pid);
      if (stcb != NULL)
        {
          return nxsig_tcbdispatch(stcb, info, false);
        }
    }

  return -ESRCH;
}
