/****************************************************************************
 * arch/arm/src/armv7-m/arm_usagefault.c
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
#include <assert.h>
#include <inttypes.h>
#include <nuttx/debug.h>
#include <nuttx/sched.h>
#include <nuttx/userspace.h>

#include <arch/irq.h>

#include "nvic.h"
#include "arm_internal.h"
#include "arm_ortcommon.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_USAGEFAULT
#  define ufalert(format, ...) _alert(format, ##__VA_ARGS__)
#else
#  define ufalert(x...)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_usagefault
 *
 * Description:
 *   This is Usage Fault exception handler.  It also catches SVC call
 *   exceptions that are performed in bad contexts.
 *
 ****************************************************************************/

int arm_usagefault(int irq, void *context, void *arg)
{
  uint32_t cfsr = getreg32(NVIC_CFAULTS);

  /* Dump some usage fault info */

  ufalert("PANIC!!! Usage Fault:\n");
  ufalert("\tIRQ: %d regs: %p\n", irq, context);
  ufalert("\tBASEPRI: %08x PRIMASK: %08x IPSR: %08"
          PRIx32 " CONTROL: %08" PRIx32 "\n",
          getbasepri(), getprimask(), getipsr(), getcontrol());
  ufalert("\tCFSR: %08" PRIx32 " HFSR: %08" PRIx32 " DFSR: %08"
          PRIx32 " BFAR: %08" PRIx32 " AFSR: %08" PRIx32 "\n",
          cfsr, getreg32(NVIC_HFAULTS), getreg32(NVIC_DFAULTS),
          getreg32(NVIC_BFAULT_ADDR), getreg32(NVIC_AFAULTS));

  ufalert("Usage Fault Reason:\n");
  if (cfsr & NVIC_CFAULTS_UNDEFINSTR)
    {
      ufalert("\tUndefined instruction\n");
    }

  if (cfsr & NVIC_CFAULTS_INVSTATE)
    {
      ufalert("\tInvalid state\n");
    }

  if (cfsr & NVIC_CFAULTS_INVPC)
    {
      ufalert("\tInvalid PC load, "
              "caused by an invalid PC load by EXC_RETURN\n");
    }

  if (cfsr & NVIC_CFAULTS_NOCP)
    {
      ufalert("\tNo Coprocessor\n");
    }

  if (cfsr & NVIC_CFAULTS_STKOF)
    {
      ufalert("\tStack Overflow\n");
    }

  if (cfsr & NVIC_CFAULTS_UNALIGNED)
    {
      ufalert("\tUnaligned access\n");
    }

  if (cfsr & NVIC_CFAULTS_DIVBYZERO)
    {
      ufalert("\tDivide by zero\n");
    }

#ifdef CONFIG_ORT_MEMDOMAIN
  /* [ORT] 与 arm_memfault.c **完全同一条判据**：故障 PC 落在用户代码区
   * （USERSPACE->us_textstart..us_textend）→ 用户态（容器）故障，
   * 只终止该任务，不 panic 内核。
   *
   * ★ 这一条原先**完全没有** —— 见手册 §三·补三十七：
   *   容器执行一条未定义指令（`udf`）就能把整机打停机，
   *   而这不需要恶意 —— 一个被写坏的函数指针指到自己的数据上就够了。
   *
   * ★ 三条故障向量（memfault / busfault / usagefault）必须语义相同，
   *   否则"容器能不能带走整机"取决于它撞上的是哪一条 ——
   *   这正是这笔债的成因。
   */

  if (USERSPACE->us_textstart != 0)
    {
      FAR uint32_t *regs = (FAR uint32_t *)context;
      uintptr_t pc = (uintptr_t)regs[REG_PC];

      if (pc >= USERSPACE->us_textstart && pc < USERSPACE->us_textend)
        {
          FAR struct tcb_s *ftcb = nxsched_self();
          enum ort_fault_action_e act;

          _alert("ORT: USER TASK USAGEFAULT pid=%d pc=%08" PRIxPTR
                 " -> terminate task\n",
                 ftcb != NULL ? ftcb->pid : -1, pc);

          /* 先清 fault 状态，避免异常返回时重新触发同一个 fault。
           * 未定义指令没有对应的 FAR 寄存器，PC 就是全部信息。 */

          putreg32(NVIC_CFAULTS_USGFAULTSR_MASK, NVIC_CFAULTS);

          /* 处置与 memfault 共用一份（见 arm_ortcommon.c）。 */

          act = ort_contain_user_fault(ftcb, pc, pc);

          if (act == ORT_FAULT_CONTAINED)
            {
              /* 正常异常返回 → PendSV → 信号投递 → 容器被终止 */

              return OK;
            }

          _alert("ORT: user usagefault %s -> fail-stop\n",
                 act == ORT_FAULT_NO_CONTAINER ? "with no container"
                                               : "undeliverable");

          up_irq_save();
          PANIC_WITH_REGS("user usagefault", context);
        }
    }
#endif

  up_irq_save();
  PANIC_WITH_REGS("panic", context);
  return OK;
}
