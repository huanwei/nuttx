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
 * Name: ort_caps
 *
 * Description:
 *   申报本平台**实际强制执行**的 ORT 能力位。
 *
 ****************************************************************************/

uint32_t ort_caps(void)
{
  /* ★ 这里返回 **0**：ORT-A 不提供「容器自行处理故障」能力。
   *
   *   不是"还没做"，是**做不到**（实测，见 §三·补十二）：
   *   有用户处理器时 nxsig_queue_action() 只能选 up_schedule_sigaction()，
   *   控制权必然回到建立在**用户栈**上的 arm_sigdeliver；
   *   而 abort 向量也把寄存器帧建在用户栈上（SYS 栈 == 用户栈）。
   *   实测症状：取指跳飞到 0x6 → arm_undefinedinsn panic → 整机复位。
   *
   *   所以容器注册的处理器被识别出来后，内核直接升级 SIGKILL ——
   *   没有通知机会。监督者不受影响：它走 ORT_SIGFAULT 独立通道。
   *
   *   申报 0 而不是"假装支持"，是刻意的：
   *   准入检查必须能问出真实行为，否则这个位就没有意义。
   *
   *   ⚠️ 这个返回值与 ort_handle_user_fault() 里那行
   *      `if (faults == 1 && ort_sig_isdefault(...))` 是**同一件事的两面**：
   *      那边跳过用户处理器，所以这边没有 FAULT_HANDLER 能力。
   *      改一边必须改另一边 —— 否则这个位就会开始说谎。
   */

  /* ★ 2026-10-04 起返回 ORT_CAP_FAULT_HANDLER：**与 ORT-M 一致**。
   *
   *   这里原先返回 0 —— 不是"还没做"，是**做不到**（§三·补十二 实测）：
   *   有用户处理器时控制权会回到建立在**用户栈**上的 arm_sigdeliver，
   *   而 abort 向量也把帧建在用户栈上 ⇒ 三者挤一张栈 ⇒ 整机复位。
   *
   *   那条路已在 §三·补四十 修好，障碍不存在了。
   *   而现在**必须打开**：对标 Wind River —— 它的 RTP 能收到自己的异常
   *   （异常产生同步信号、在当前上下文立即执行）。默认不给不是优势，
   *   是差距；真正的能力是"**能给，且给了也带不走别人**"。
   *
   *   ★ 能力的边界（与 ort_handle_user_fault 里那段是同一件事的两面，
   *     改一边必须改另一边）：处理器是**通知，不是恢复** ——
   *     返回后再踩同一条故障指令 → 升级 SIGKILL。容器仍然要死。
   *     监督者的终止权不变，它另有 ORT_SIGFAULT 独立通道。
   */

  return ORT_CAP_FAULT_HANDLER;
}

/****************************************************************************
 * Name: ort_domain_capacity
 *
 * Description:
 *   本架构**功能上**能给的域数（手册 §三·补五十四）。
 *
 *   ★ 为什么是 ORT_STATE_SLOTS（8）而不是编码上限 255：
 *     域号在 ARMv7-A 上不参与强制，它唯一的作用是**状态槽索引**
 *     （STATE_PUT/GET/SEQ 全按域号取槽）。255 是 uint8 编码容量，
 *     不是功能容量 —— "能绑但发布不了状态"的域对部署方是陷阱，
 *     不该报成容量。两个 SKU 的容量含义因此一致：
 *     **域号能被完整功能支撑的个数**。
 *
 ****************************************************************************/

int ort_domain_capacity(void)
{
  return ORT_STATE_SLOTS;
}

/****************************************************************************
 * Name: ort_container_domain
 *
 * Description:
 *   ARMv7-A 上「容器」= 进程 + 它的地址空间，没有"内存域"这个维度 ——
 *   域号在这里只是个**标签**，不参与强制。
 *
 *   但标签仍然是必要的：容器靠它判"我准入完了没有"（跨平台共用的
 *   准入等待），部署层靠它把容器与 manifest 对上。
 *   所以这里如实返回 ort_container_bind() 记下的值，
 *   而不是恒返回 -1。
 *
 *   未绑定返回 -1（编码 0）—— 与 MPU 侧的语义一致。
 *
 ****************************************************************************/

int ort_container_domain(FAR struct task_group_s *group)
{
  if (group == NULL || group->tg_ort_domain == 0)
    {
      return -1;
    }

  return (int)group->tg_ort_domain - 1;
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

  /* 校验目标存在 —— 让调用者能发现"绑了个不存在的容器" */

  tcb = nxsched_get_tcb(pid);
  if (tcb == NULL || tcb->group == NULL)
    {
      return -ESRCH;
    }

  /* ★ 预算之外（含此前的 255 编码上限）→ -EINVAL，不改动既有绑定。
   *   判**预算** = min(容量, 配额)：容量管物理，配额管部署（§三·补五十四）。
   *   旧的"编码上限 254"随之被预算取代 —— 预算 ≤ 容量（8）≤ 254，
   *   比它更严，也更有意义。 */

  if (domain < 0 || domain >= ort_domain_budget())
    {
      return -EINVAL;
    }

  /* ★ 域号在 ARMv7-A 上**不参与强制**（隔离由地址空间天然给出），
   *   但仍然要**记录**下来。为什么不能像原来那样返回 OK 就走：
   *
   *   容器靠 `while (prctl(PR_GET_ORT_DOMAIN) < 0)` 判"我准入完了没有" ——
   *   这是**跨平台共用**的准入等待（见 ortsup 的 ort_container_main）。
   *   如果 MMU 侧因为"域是空的"就永远返回 -1，同一份容器代码在 ORT-A 上
   *   会一直等到超时然后退出 —— 而监督者那边看起来只是"容器起不来"。
   *
   *   部署层也需要它：把运行中的容器和 manifest 里的那一项对上。
   *
   *   编码沿用 tg_ort_domain 的既有契约（0 = 未绑定，域 n 存为 n+1），
   *   与 MPU 侧完全一致 —— 见 include/nuttx/sched.h 里那段说明。
   */

  tcb->group->tg_ort_domain = (uint8_t)(domain + 1);
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

  /* ── 第一步：投递 SIGSEGV，给进程一个可观测点 ─────────────────────
   *
   * ★ 只对**没装自己的 SIGSEGV 处理器**的进程投。
   *
   *   原因不是策略洁癖，而是这条路在当前配置下走不通：
   *   有用户处理器时，nxsig_queue_action() 只能选 up_schedule_sigaction()，
   *   把控制权交给建立在**用户栈**上的 arm_sigdeliver trampoline ——
   *   而 abort 向量恰恰把寄存器帧也建在用户栈上（SYS 栈 == 用户栈）。
   *   实测：取指跳飞到 0x6、arm_undefinedinsn panic、整机复位。
   *
   *   换句话说：**硬件故障不可能安全地"交给进程自己处理"**，
   *   这也正是 ORT 的立场 —— 容器的终止权在监督者/内核手里，
   *   进程无权通过装个处理器来把内核拖下水。
   *
   *   没装处理器的进程（绝大多数）仍然拿到 SIGSEGV，
   *   其默认动作就是终止 —— 那一步是安全的（见 ort_sig_kill）。
   */

  /* ★ 2026-10-04 起**无条件投递**（与 ORT-M 一致）。
   *   原先只对'没装处理器'的进程投 —— 当时的理由是真的：
   *   那条路走不通（§三·补二十七 实测整机复位）。它已在
   *   §三·补四十 修好，所以跳过不再必要，而且**与对标不符**
   *   （Wind River 的 RTP 能收到自己的异常）。
   *
   *   ★ 给了也不危险，靠三条不变量（缺一不可）：
   *     1. 处理器**不能阻止终止** —— 返回后再踩同一条故障指令，
   *        下一步立即升级 SIGKILL（实测）；
   *     2. 处理器跑在**用户态、自己的域内**；
   *     3. 监督者走 **ORT_SIGFAULT 独立通道**，不依赖容器的处理器。
   */
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
      _alert("ORT: escalating pid=%d to SIGKILL (faults=%" PRIu32
             ", sigdeliver=%p)\n", ftcb->pid, faults, ftcb->sigdeliver);
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
