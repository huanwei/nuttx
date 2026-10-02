/****************************************************************************
 * arch/arm/src/armv7-a/arm_ort.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * [ORT] ARMv7-A（MMU）侧的容器支持
 *
 * ★ 与 MPU 平台的根本区别：
 *   MMU 下每个进程有独立地址空间，隔离是**天然**的 ——
 *   不需要 per-switch 的 region 编程，也不需要"内存域"。
 *
 *   所以本文件里 ort_memdomain_switch() 根本不存在（切换钩子在
 *   sched_switchcontext.c 里由 CONFIG_ORT_MEMDOMAIN 守卫，
 *   MMU 平台不打开那个选项）。
 *
 *   这里只做两件事：
 *     ① 给监督者一个"绑域"接口（语义上无操作，保持跨平台一致的
 *        supervisor 代码可以不改）
 *     ② 把"用户态故障 → 终止进程而不是 panic 内核"这条策略集中在一处
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_ORT_CONTAINER

#include <stdint.h>
#include <errno.h>
#include <debug.h>
#include <nuttx/debug.h>

#include <sys/prctl.h>

#include <nuttx/sched.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/signal.h>

#include "arm_ortcommon.h"
#include "arm.h"

#include "signal/signal.h"         /* nxsig_isdefault() */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_ARCH_KERNEL_STACK

/****************************************************************************
 * Name: ort_fault_on_kstack
 *
 * Description:
 *   在**本进程的内核栈**上运行 ort_handle_user_fault()。
 *
 *   ★ 为什么必须换栈（见手册 §三·补十一）：
 *
 *   `up_schedule_sigaction()` 会把寄存器帧**原地**下移 XCPTCONTEXT_SIZE：
 *
 *       tcb->xcp.regs = (uint32_t)tcb->xcp.regs - XCPTCONTEXT_SIZE;
 *       memcpy(tcb->xcp.regs, tcb->xcp.saved_regs, XCPTCONTEXT_SIZE);
 *
 *   - **系统调用路径安全**：处理器跑在**内核栈**上，帧在**用户栈**上，
 *     两张栈不相干，下移落到用户栈的空闲区。
 *   - **abort 向量不安全**：`arm_vectordata` 把帧建在 SYS 栈上 ——
 *     而 SYS 栈就是用户栈（SYS/USR 共用 SP）。于是异常处理器的调用帧
 *     和帧**共用同一张栈**，下移区间 `[regs-XS, regs)` 正好压住
 *     处理器自己的返回地址。
 *
 *   实测症状：无 panic、无输出、**整机复位** ——
 *   因为返回地址被踩后执行流直接跳到垃圾地址。
 *
 *   内核栈只在 SVC 期间使用，而 abort 向量进来时已 `cpsid if` 关中断，
 *   不存在重入。asm 里保存/恢复 sp，调用者可以正常继续。
 *
 ****************************************************************************/

static bool ort_fault_on_kstack(FAR struct tcb_s *tcb,
                                uintptr_t pc, uintptr_t addr)
{
  uintptr_t ret;

  if (tcb->xcp.kstack == NULL)
    {
      /* 没分配内核栈（非 KERNEL 构建）——退回原路径。
       * 那条路径只在 KERNEL 构建下才有上面这个问题。
       */

      return ort_handle_user_fault(pc, addr);
    }

  __asm__ __volatile__
    (
     "mov  r4, sp\n"
     "mov  sp, %[ksp]\n"
     "mov  r0, %[pc]\n"
     "mov  r1, %[addr]\n"
     "blx  %[fn]\n"
     "mov  sp, r4\n"
     "mov  %[ret], r0\n"
     : [ret]  "=r" (ret)
     : [pc]   "r"   (pc),
       [addr] "r"   (addr),
       [ksp]  "r"   ((uintptr_t)tcb->xcp.kstack + ARCH_KERNEL_STACKSIZE),
       [fn]   "r"   (ort_handle_user_fault)
     : "r0", "r1", "r4", "lr", "memory", "cc"
    );

  return ret != 0;
}

#else

#  define ort_fault_on_kstack(tcb, pc, addr) \
          ort_handle_user_fault(pc, addr)

#endif /* CONFIG_ARCH_KERNEL_STACK */

/****************************************************************************
 * Name: ort_sig_kill
 *
 * Description:
 *   给故障进程投递信号，并保证「默认终止动作」在**内核栈**上就地完成。
 *
 *   ★ 为什么不能一律走 nxsig_kill()（见手册 §三·补十二）：
 *
 *   SIGSEGV 的默认动作（CONFIG_SIG_DEFAULT）是 nxsig_abnormal_termination()，
 *   它结尾是 `_exit(EXIT_FAILURE)` —— 在**调用者脚下的那张栈**上执行。
 *
 *   而在异常上下文里，nxsig_queue_action() 会因为 up_interrupt_context()
 *   为真而选择 up_schedule_sigaction()：把寄存器帧搬到用户栈上、
 *   把 PC 设成 arm_sigdeliver。于是整条终止路径最终跑在**用户栈**上，
 *   `_exit()` 一执行就把 trampoline 正踩着的那张用户栈拆掉了 ——
 *   实测表现为取指跳飞到 0x2、arm_undefinedinsn panic。
 *
 *   nxsig_isdefault() 恰好就是 nxsig_deliver() 用来区分
 *   「内核态处理器」和「用户态处理器」的那个判据：
 *
 *     - true  → 处理器是内核里的默认动作，**绝不会**把控制权交给用户代码；
 *     - false → 进程自己装的处理器，nxsig_deliver() 会走
 *               up_signal_dispatch() 切回用户态去跑。
 *
 *   所以只有前者可以在内核栈上直接投递。后者必须照旧走 trampoline：
 *   用户代码一旦在内核栈、SYS 模式下执行，就是实打实的特权提升。
 *
 ****************************************************************************/

#ifdef CONFIG_SIG_DEFAULT
static int ort_sig_kill(FAR struct tcb_s *ftcb, int signo)
{
  irqstate_t flags;
  int ret;

  if (!nxsig_isdefault(ftcb, signo) || !up_interrupt_context())
    {
      /* 用户自己的处理器 / 不在异常上下文 —— 走原来的路。 */

      return nxsig_kill(ftcb->pid, signo);
    }

  /* 临时摘掉中断上下文标志，让 nxsig_queue_action() 走「直接投递」分支
   * （stcb == this_task() && !up_interrupt_context()）：
   * 默认动作于是在当前这张（内核）栈上跑完，_exit() 不会拆掉自己的栈。
   *
   * 正常情况下 nxsig_kill() 不返回（_exit() 已经换到别的任务去了），
   * 下面的恢复语句只在投递失败时才会执行。
   */

  flags = up_irq_save();
  up_set_interrupt_context(false);

  ret = nxsig_kill(ftcb->pid, signo);

  up_set_interrupt_context(true);
  up_irq_restore(flags);
  return ret;
}

#else

#  define ort_sig_kill(ftcb, signo) nxsig_kill((ftcb)->pid, (signo))

#endif /* CONFIG_SIG_DEFAULT */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ort_handle_user_fault_kstack
 *
 * Description:
 *   ort_handle_user_fault() 的入口包装：abort 处理器必须调这个，
 *   直接调 ort_handle_user_fault() 会在 KERNEL 构建下踩掉自己的调用帧。
 *
 ****************************************************************************/

bool ort_handle_user_fault_kstack(FAR struct tcb_s *tcb,
                                  uintptr_t pc, uintptr_t addr)
{
  return ort_fault_on_kstack(tcb, pc, addr);
}

/****************************************************************************
 * Name: ort_container_domain
 *
 * Description:
 *   ARMv7-A 上「容器」= 进程 + 它的地址空间，没有"内存域"这个维度。
 *   恒返回 -1（未绑定）。
 *
 ****************************************************************************/

int ort_container_domain(FAR struct task_group_s *group)
{
  UNUSED(group);
  return -1;
}

/****************************************************************************
 * Name: ort_container_bind
 *
 * Description:
 *   ARMv7-A 上「绑域」是**语义上的无操作**。
 *
 *   为什么仍然接受而不是返回 -ENOSYS：
 *     上层（监督者 ortsup / prctl）是跨平台共用的。让它在这里拿到
 *     一个"成功"比到处判断平台要干净 —— 因为 MMU 下容器的边界
 *     已经由地址空间给出了，确实不需要再做任何事。
 *
 *   但权限检查仍然要做：不然"只有监督者能绑"这条约束在两个平台上
 *   就不一致了，将来移植时会变成坑。
 *
 ****************************************************************************/

int ort_container_bind(pid_t pid, int domain)
{
  FAR struct tcb_s *tcb;

  if (nxsched_self()->pid != ort_supervisor_pid())
    {
      return -EPERM;
    }

  /* 仍然校验目标存在 —— 让调用者能发现"绑了个不存在的容器" */

  if (domain >= 0)
    {
      tcb = nxsched_get_tcb(pid);
      if (tcb == NULL)
        {
          return -ESRCH;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: ort_handle_user_fault
 *
 * Description:
 *   处理一次**用户态**故障：通知监督者，并保证该进程被终止。
 *
 *   ★ 这里集中了全部策略，abort 处理器只负责"判别是不是用户态"
 *     和"把 pc/addr 传进来"。
 *
 * Input Parameters:
 *   pc   - 触发故障的指令地址
 *   addr - 被非法访问的地址
 *
 * Returned Value:
 *   true  = 已处理，调用者应正常从异常返回（进程会被信号终止）
 *   false = 无法隔离，调用者应当 panic（fail-stop，而不是假装没事）
 *
 ****************************************************************************/

bool ort_handle_user_fault(uintptr_t pc, uintptr_t addr)
{
  FAR struct tcb_s *ftcb = nxsched_self();
  FAR struct task_group_s *fgroup;
  uint32_t faults;
  int ret = -ESRCH;

  if (ftcb == NULL || ftcb->group == NULL)
    {
      _alert("ORT: user fault with no container -> fail-stop\n");
      return false;
    }

  fgroup = ftcb->group;
  faults = ++fgroup->tg_ort_faults;

  _alert("ORT: USER FAULT pid=%d tgid=%d pc=%08" PRIxPTR " addr=%08" PRIxPTR
         " -> terminate process\n", ftcb->pid, fgroup->tg_pid, pc, addr);

  /* ── 第零步：先通知监督者 ────────────────────────────────────────
   *
   * 必须用**独立通道**，不能靠进程自己注册的 SIGSEGV 处理器：
   * 那个处理器是进程可控的（SIG_IGN 在 NuttX 里等于把动作删掉），
   * 一删监督者就瞎了。
   */

  ort_fault_notify(ftcb->pid, pc, addr, faults);

  /* ── 第一步：投递 SIGSEGV，给进程/监督者一个可观测点 ────────────── */

  if (faults == 1)
    {
      ret = ort_sig_kill(ftcb, SIGSEGV);
    }

  /* ── 第二步：保证进程一定会死（监督者的终止权）──────────────────
   *
   * 为什么 SIGSEGV 不足以终止进程：
   *   POSIX 允许忽略 SIGSEGV（结果未定义），NuttX 亦然 ——
   *   进程只要 sigaction(SIGSEGV, SIG_IGN)，或注册一个"打印一下就返回"
   *   的处理器，异常返回后就会回到同一条故障指令再次 fault ——
   *   **无限循环卡死 CPU**。
   *
   * 为什么 SIGKILL 可以依赖：
   *   CONFIG_SIG_DEFAULT 给它设了 SIG_FLAG_NOCATCH，
   *   sigaction(SIGKILL, SIG_IGN) 返回 -EINVAL；其默认动作由内核在
   *   任务启动时安装，进程删不掉。
   */

  if (faults > 1 || ftcb->sigdeliver == NULL)
    {
      _alert("ORT: escalating pid=%d to SIGKILL (faults=%" PRIu32 ")\n",
             ftcb->pid, faults);
      ret = ort_sig_kill(ftcb, SIGKILL);
    }

  /* 最后兜底：连 SIGKILL 都投不出去（CONFIG_SIG_DEFAULT 没开）
   * 说明任何信号都不会被处理，异常返回后必然无限 fault。
   * 此时唯一诚实的做法是 fail-stop。
   */

  if (ret < 0 || ftcb->sigdeliver == NULL)
    {
      _alert("ORT: cannot terminate pid=%d (ret=%d) -> fail-stop\n"
             "     (CONFIG_SIG_DEFAULT=y 是 ORT 的必需配置)\n",
             ftcb->pid, ret);
      return false;
    }

  return true;
}

#endif /* CONFIG_ORT_CONTAINER */
