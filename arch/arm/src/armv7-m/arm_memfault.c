/****************************************************************************
 * arch/arm/src/armv7-m/arm_memfault.c
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

#include <assert.h>
#include <nuttx/debug.h>
#include <inttypes.h>

#include <arch/irq.h>
#ifdef CONFIG_BUILD_PROTECTED
#  include <signal.h>
#  include <nuttx/sched.h>
#  include <nuttx/signal.h>
#  include <nuttx/userspace.h>
#endif

#include "nvic.h"
#include "arm_internal.h"
#include "arm_ortcommon.h"

#ifdef CONFIG_ORT_MEMDOMAIN
#  include "arm_memdomain.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_MEMFAULT
#  define mfalert(format, ...) _alert(format, ##__VA_ARGS__)
#else
#  define mfalert(x...)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_memfault
 *
 * Description:
 *   This is Memory Management Fault exception handler.  Normally we get
 *   here when the Cortex M3 MPU is enabled and an MPU fault is detected.
 *   However, I understand that there are other error conditions that can
 *   also generate memory management faults.
 *
 ****************************************************************************/

int arm_memfault(int irq, void *context, void *arg)
{
  uint32_t cfsr = getreg32(NVIC_CFAULTS);

  /* Dump some memory management fault info */

  mfalert("PANIC!!! Memory Management Fault:\n");
  mfalert("\tIRQ: %d context: %p\n", irq, context);
  mfalert("\tCFSR: %08x MMFAR: %08x\n",
          getreg32(NVIC_CFAULTS), getreg32(NVIC_MEMMANAGE_ADDR));
  mfalert("\tBASEPRI: %08x PRIMASK: %08x IPSR: %08"
          PRIx32 " CONTROL: %08" PRIx32 "\n",
          getbasepri(), getprimask(), getipsr(), getcontrol());

  mfalert("Memory Management Fault Reason:\n");
  if (cfsr & NVIC_CFAULTS_IACCVIOL)
    {
      mfalert("\tInstruction access violation\n");
    }

  if (cfsr & NVIC_CFAULTS_DACCVIOL)
    {
      mfalert("\tData access violation\n");
    }

  if (cfsr & NVIC_CFAULTS_MUNSTKERR)
    {
      mfalert("\tMemManage fault on unstacking\n");
    }

  if (cfsr & NVIC_CFAULTS_MSTKERR)
    {
      mfalert("\tMemManage fault on stacking\n");
    }

  if (cfsr & NVIC_CFAULTS_MLSPERR)
    {
      mfalert("\tFloating-point lazy state preservation error\n");
    }

  /* In some scenarios (e.g. testing, debugging, etc.) where we want to
   * ignore the memory management fault and proceed, we can set the parameter
   * arg to 0xffffffff to skip the Memory Management Fault exception
   */

  if (arg == (void *)0xffffffff)
    {
      uint32_t *regs = context;
      uint16_t insn;
      mfalert("Skip the memory management fault and proceed\n");

      /* regs[REG_PC] advance by 2/4 bytes depends on whether the encoded
       * faulty instructions are 16-bit/32-bit thumb instructions
       */

      insn = (*(volatile uint16_t *)(regs[REG_PC]) >> 11) & 0x1f;

      if (insn == 0x1d || insn == 0x1e || insn == 0x1f)
        {
          regs[REG_PC] += 4;
        }
      else
        {
          regs[REG_PC] += 2;
        }

      /* Clear the MMFSR and MMFAR register */

      putreg32(0xff, NVIC_CFAULTS);
      putreg32(0, NVIC_MEMMANAGE_ADDR);

      return OK;
    }

/* [ORT] 这块多出来的判据要 **两个开关同时开**：BUILD_PROTECTED 是它的
 * 语境（用户态存在），ORT_CONTAINER 是它的依赖（调 ort_contain_* 与
 * ORT_FAULT_*）。只开 PROTECTED 的构建（如 nucleo-f767zi:knsh 裸板）
 * 走下面的上游兜底 panic —— 之前只有"两个都开"的 M 配置编过，
 * 单开 PROTECTED 会编译失败（2026-10-05 建 F767 时暴露）。 */

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_ORT_CONTAINER)
  /* [ORT] 判别故障是否来自用户态。
   *
   * 判据：faulting PC 落在用户代码区（USERSPACE->us_textstart..us_textend）。
   *
   * 为什么必须区分：
   *   内核代码的 memfault 是真 bug，应当 panic；
   *   用户态（容器）越界只应终止该任务 ——
   *   否则一个容器越界会把 SystemPrivTask 一起带走，
   *   违反「单容器故障隔离、不影响整机」的核心卖点。
   */

  if (USERSPACE->us_textstart != 0)
    {
      FAR uint32_t *regs = (FAR uint32_t *)context;
      uintptr_t pc = (uintptr_t)regs[REG_PC];

      if (pc >= USERSPACE->us_textstart && pc < USERSPACE->us_textend)
        {
          FAR struct tcb_s *ftcb = nxsched_self();
          uintptr_t addr = (uintptr_t)getreg32(NVIC_MEMMANAGE_ADDR);

          _alert("ORT: USER TASK MEMFAULT pid=%d pc=%08" PRIxPTR
                 " addr=%08" PRIxPTR " -> terminate task\n",
                 ftcb != NULL ? ftcb->pid : -1, pc, addr);

          /* 先清 fault 状态，避免异常返回时重新触发同一个 fault。
           * 必须在读走 MMFAR 之后 —— 写 0 到 MMFAR 会丢掉故障地址。
           */

          putreg32(0xff, NVIC_CFAULTS);
          putreg32(0, NVIC_MEMMANAGE_ADDR);

          /* ★ 处置（通知监督者 → SIGSEGV → 升级 SIGKILL）与 usagefault
           *   共用一份，见 arm/arm_ortcommon.c 的 ort_contain_user_fault()。
           *
           *   为什么不再各写一份：undefined instruction 那条路原先就是
           *   因为"只有 memfault 这一条被处理过"才成了洞（手册 §三·补三十七）。
           *   两处判据不一致正是那笔债的成因，不能再制造一次。
           */

          switch (ort_contain_user_fault(ftcb, pc, addr))
            {
              case ORT_FAULT_NO_CONTAINER:
                _alert("ORT: user memfault with no container -> fail-stop\n");
                up_irq_save();
                PANIC_WITH_REGS("user memfault: no container", context);

              case ORT_FAULT_UNDELIVERABLE:
                up_irq_save();
                PANIC_WITH_REGS("user memfault: undeliverable", context);

              default:
                break;
            }

          return OK;
        }
    }
#endif

  up_irq_save();
  PANIC_WITH_REGS("panic", context);
  return OK; /* Won't get here */
}
