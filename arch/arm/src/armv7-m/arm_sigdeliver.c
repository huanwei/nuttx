/****************************************************************************
 * arch/arm/src/armv7-m/arm_sigdeliver.c
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

#include <stdint.h>
#include <string.h>
#include <sched.h>
#include <assert.h>

#include <nuttx/debug.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "sched/sched.h"
#include "arm_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_sigdeliver
 *
 * Description:
 *   This is the a signal handling trampoline.  When a signal action was
 *   posted.  The task context was mucked with and forced to branch to this
 *   location with interrupts disabled.
 *
 ****************************************************************************/

void arm_sigdeliver(void)
{
  struct tcb_s *rtcb = this_task();
  uint32_t *regs = rtcb->xcp.saved_regs;
  uint32_t *new_regs;
  uint32_t desired_sp;
  uint32_t implied_sp;

#ifdef CONFIG_SMP
  /* In the SMP case, we must terminate the critical section while the signal
   * handler executes, but we also need to restore the irqcount when the
   * we resume the main thread of the task.
   */

  int16_t saved_irqcount;
#endif

  board_autoled_on(LED_SIGNAL);

  sinfo("rtcb=%p sigdeliver=%p sigpendactionq.head=%p\n",
        rtcb, rtcb->sigdeliver, rtcb->sigpendactionq.head);
  DEBUGASSERT(rtcb->sigdeliver != NULL);

retry:
#ifdef CONFIG_SMP
  /* In the SMP case, up_schedule_sigaction(0) will have incremented
   * 'irqcount' in order to force us into a critical section.  Save the
   * pre-incremented irqcount.
   */

  saved_irqcount = rtcb->irqcount;
  DEBUGASSERT(saved_irqcount >= 0);

  /* Now we need call leave_critical_section() repeatedly to get the irqcount
   * to zero, freeing all global spinlocks that enforce the critical section.
   */

  while (rtcb->irqcount > 0)
    {
      leave_critical_section((uint8_t)regs[REG_BASEPRI]);
    }
#endif /* CONFIG_SMP */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  /* Then make sure that interrupts are enabled.  Signal handlers must always
   * run with interrupts enabled.
   */

  up_irq_enable();
#endif

  /* Deliver the signal */

  (rtcb->sigdeliver)(rtcb);

  /* Output any debug messages BEFORE restoring errno (because they may
   * alter errno), then disable interrupts again and restore the original
   * errno that is needed by the user logic (it is probably EINTR).
   *
   * I would prefer that all interrupts are disabled when
   * arm_fullcontextrestore() is called, but that may not be necessary.
   */

  sinfo("Resuming\n");

#ifdef CONFIG_SMP
  /* Restore the saved 'irqcount' and recover the critical section
   * spinlocks.
   */

  DEBUGASSERT(rtcb->irqcount == 0);
  while (rtcb->irqcount < saved_irqcount + 1)
    {
      enter_critical_section();
    }
#endif

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  up_irq_save();
#endif

  if (!sq_empty(&rtcb->sigpendactionq) &&
      (rtcb->flags & TCB_FLAG_SIGNAL_ACTION) == 0)
    {
#ifdef CONFIG_SMP
      leave_critical_section((uint8_t)regs[REG_BASEPRI]);
#endif
      goto retry;
    }

  /* Modify the saved return state with the actual saved values in the
   * TCB.  This depends on the fact that nested signal handling is
   * not supported.  Therefore, these values will persist throughout the
   * signal handling action.
   *
   * Keeping this data in the TCB resolves a security problem in protected
   * and kernel mode:  The regs[] array is visible on the user stack and
   * could be modified by a hostile program.
   */

  rtcb->sigdeliver = NULL;  /* Allows next handler to be scheduled */

  /* Then restore the correct state for this thread of
   * execution.
   */

  board_autoled_off(LED_SIGNAL);
#ifdef CONFIG_SMP
  /* We need to keep the IRQ lock until task switching */

  leave_critical_section(up_irq_save());
#endif

  /* If the signal handler modified SP (REG_R13), relocate the saved
   * context so that the hardware exception return produces the correct SP.
   *
   * On ARMv7-M, the exception return path sets PSP to the HW frame address
   * and hardware computes final SP = PSP + frame_size.  The implied SP is
   * determined by the physical location of the context, not by REG_R13.
   * To honor a modified SP, we memmove the entire context frame to the
   * address where the end of the frame equals the desired SP.
   */

  /* ★★ [ORT] 恢复必须用**投递时那一个帧** —— 也就是函数开头捕获的 `regs`。
   *
   *   原来的结尾是 `rtcb->xcp.regs = rtcb->xcp.saved_regs;`，即**重新读**
   *   一次这个字段。但 `saved_regs` 会在投递过程中被改写：
   *   `up_schedule_sigaction()` 每次排信号都会写它（`saved_regs = xcp.regs`），
   *   而 `retry` 路径下它**在同一趟 arm_sigdeliver 里**就会被重入。
   *   于是：
   *
   *     - 上面那段 `desired_sp/implied_sp` 用的是**开头**的 `regs`；
   *     - 真正恢复用的却是**结尾**的 `saved_regs`；
   *     - 两者可以是**两个不同的帧**，那段 SP 判断因此失去意义，
   *       恢复也可能落到一个内容已被后续栈活动覆盖的帧上。
   *
   *   ARMv7-M 的异常返回是**硬件按 PSP 弹栈**的：帧地址与帧里记的 SP
   *   必须自洽。拿到一个不自洽的帧，弹出来的 PC 就是栈上的任意值。
   *
   *   实测（mps2-an500，标准场景，改动前 2/4~4/4 复现）：
   *     局部 regs=0x60c08dc0（内容自洽，PC 在用户代码、SP==implied）
   *     xcp.regs = saved_regs = 0x60c08c60（另一个帧）
   *     崩溃 PC = 0x60c08d38 = 0x60c08c60 + XCPTCONTEXT_SIZE
   *     CFSR.INVSTATE —— 跳到了偶数地址。
   *
   *   openvela 的 fork 记录过同一现象（"a new up_schedule_sigaction could
   *   overlay saved_regs with in-process regs"），他们的修法是限制投递时机；
   *   这里改成**只用一个帧**，更小且不需要额外的状态位。
   *
   *   不变量：恢复用的帧、SP 判断用的帧、以及两个 TCB 字段，**同一个**。 */

  desired_sp = regs[REG_R13];
  implied_sp = (uint32_t)regs + XCPTCONTEXT_SIZE;

  if (desired_sp != implied_sp)
    {
      new_regs = (uint32_t *)(desired_sp - XCPTCONTEXT_SIZE);
      memmove(new_regs, regs, XCPTCONTEXT_SIZE);
      regs = new_regs;
    }

  rtcb->xcp.saved_regs = regs;
  rtcb->xcp.regs       = regs;
  arm_fullcontextrestore();
}
