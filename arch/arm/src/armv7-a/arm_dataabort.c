/****************************************************************************
 * arch/arm/src/armv7-a/arm_dataabort.c
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
#include <assert.h>

#include <nuttx/debug.h>
#include <nuttx/irq.h>

#include "mmu.h"
#include "sched/sched.h"
#include "arm_internal.h"

#ifdef CONFIG_ORT_MMU
#  include "arm_ortcommon.h"
#  include "arm.h"
#endif

#ifdef CONFIG_LEGACY_PAGING
#  include <nuttx/page.h>
#  include "arm.h"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_dataabort
 *
 * Input Parameters:
 *   regs - The standard, ARM register save array.
 *
 * If CONFIG_LEGACY_PAGING is selected in the NuttX configuration file, then
 * these additional input values are expected:
 *
 *   dfar - Fault address register.  On a data abort, the ARM MMU places the
 *     miss virtual address (MVA) into the DFAR register.  This is the
 *     address of the data which, when accessed, caused the fault.
 *   dfsr - Fault status register.  On a data a abort, the ARM MMU places an
 *     encoded four-bit value, the fault status, along with the four-bit
 *     encoded domain number, in the data DFSR
 *
 * Description:
 *   This is the data abort exception handler. The ARM data abort exception
 *   occurs when a memory fault is detected during a data transfer.
 *
 ****************************************************************************/

#ifdef CONFIG_LEGACY_PAGING
uint32_t *arm_dataabort(uint32_t *regs, uint32_t dfar, uint32_t dfsr)
{
  struct tcb_s *tcb = this_task();
  uint32_t *saveregs;
  bool savestate;

  savestate = up_interrupt_context();
  saveregs = tcb->xcp.regs;
  tcb->xcp.regs = regs;
  up_set_interrupt_context(true);

  /* In the NuttX on-demand paging implementation, only the read-only, .text
   * section is paged.  However, the ARM compiler generated PC-relative data
   * fetches from within the .text sections.  Also, it is customary to locate
   * read-only data (.rodata) within the same section as .text so that it
   * does not require copying to RAM. Misses in either of these case should
   * cause a data abort.
   *
   * We are only interested in data aborts due to page translations faults.
   * Sections should already be in place and permissions should already be
   * be set correctly (to read-only) so any other data abort reason is a
   * fatal error.
   */

  pginfo("DFSR: %08x DFAR: %08x\n", dfsr, dfar);
  if (FSR_FAULT(dfsr) == FSR_FAULT_DEBUG)
    {
      arm_dbgmonitor(0, (void *)dfar, regs);
    }
  else if((dfsr & FSR_MASK) != FSR_PAGE)
    {
      goto segfault;
    }

  /* Check the (virtual) address of data that caused the data abort. When
   * the exception occurred, this address was provided in the DFAR register.
   * (It has not yet been saved in the register context save area).
   */

  else if (dfar < PG_PAGED_VBASE || dfar >= PG_PAGED_VEND)
    {
      goto segfault;
    }
  else
    {
      pginfo("VBASE: %08x VEND: %08x\n", PG_PAGED_VBASE, PG_PAGED_VEND);

      /* Save the offending data address as the fault address in the TCB of
       * the currently task.  This fault address is also used by the prefetch
       * abort handling; this will allow common paging logic for both
       * prefetch and data aborts.
       */

      tcb->xcp.dfar = regs[REG_R15];

      /* Call pg_miss() to schedule the page fill.  A consequences of this
       * call are:
       *
       * (1) The currently executing task will be blocked and saved on
       *     on the g_waitingforfill task list.
       * (2) An interrupt-level context switch will occur so that when
       *     this function returns, it will return to a different task,
       *     most likely the page fill worker thread.
       * (3) The page fill worker task has been signalled and should
       *     execute immediately when we return from this exception.
       */

      pg_miss();
    }

  /* Restore the previous value of saveregs. */

  up_set_interrupt_context(savestate);
  tcb->xcp.regs = saveregs;
  return regs;

segfault:
  _alert("Data abort. PC: %08" PRIx32 " DFAR: %08" PRIx32 " DFSR: %08"
         PRIx32 "\n", regs[REG_PC], dfar, dfsr);
  PANIC_WITH_REGS("panic", regs);
  return regs; /* To keep the compiler happy */
}

#else /* CONFIG_LEGACY_PAGING */

uint32_t *arm_dataabort(uint32_t *regs, uint32_t dfar, uint32_t dfsr)
{
  struct tcb_s *tcb = this_task();

  tcb->xcp.regs = regs;
  up_set_interrupt_context(true);

  /* Crash -- possibly showing diagnostic debug information. */

#ifdef CONFIG_ORT_MMU
  /* [ORT] 判别故障是否来自用户态。
   *
   * ★ ARMv7-A 的判据比 ARMv7-M 简单得多：CPSR 的模式位直接告诉我们
   *   异常发生时的模式，不需要拿 PC 去比代码区范围。
   *
   * 为什么要区分：
   *   内核代码的 data abort 是真 bug，应当 panic；
   *   用户进程越界只应终止该进程 ——
   *   否则一个容器越界会把整机带走，违反「单容器故障隔离」的核心卖点。
   *   （BUILD_KERNEL 下每个进程有独立地址空间，隔离本来就有；
   *     缺的只是"别 panic"这一步。）
   */

  if ((regs[REG_CPSR] & PSR_MODE_MASK) == PSR_MODE_USR)
    {
      /* ★ 必须走 _kstack 版本：KERNEL 构建下 abort 向量把寄存器帧建在
       *   用户栈上，而异常处理器的调用帧也在同一张栈上 ——
       *   直接调会让 up_schedule_sigaction() 的原地帧下移踩掉
       *   调用链自己的返回地址（实测：无输出、整机复位）。
       *   见 arm_ort.c 的 ort_fault_on_kstack()。
       */

      if (ort_handle_user_fault_kstack(tcb, regs[REG_PC], dfar))
        {
          /* 正常异常返回 → 信号投递 → 进程被终止 */

          up_set_interrupt_context(false);

          /* ★ 返回 tcb->xcp.regs 而不是入参 regs。
           *
           *   信号投递（up_schedule_sigaction）会把寄存器帧**搬到别处**
           *   并把 PC 设成 arm_sigdeliver；矢量代码要用搬走后的那一份
           *   （arm_vectors.S: "It will differ if a context switch is
           *   required"）。
           *
           *   返回旧的 regs 会让进程回到故障指令上再 fault 一次 ——
           *   无限循环。若信号没被排上，两者本来就相等，返回它也无害。
           */

          return tcb->xcp.regs;
        }

      /* 无法隔离 —— 落到下面 panic（fail-stop） */
    }
#endif

  _alert("Data abort. PC: %08" PRIx32 " DFAR: %08" PRIx32 " DFSR: %08"
         PRIx32 "\n", regs[REG_PC], dfar, dfsr);

  if (FSR_FAULT(dfsr) == FSR_FAULT_DEBUG)
    {
      arm_dbgmonitor(0, (void *)dfar, regs);
    }
  else
    {
      PANIC_WITH_REGS("panic", regs);
    }

  up_set_interrupt_context(false);
  return regs; /* To keep the compiler happy */
}

#endif /* CONFIG_LEGACY_PAGING */
