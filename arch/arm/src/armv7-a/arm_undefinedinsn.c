/****************************************************************************
 * arch/arm/src/armv7-a/arm_undefinedinsn.c
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

#include <nuttx/arch.h>
#include <sched/sched.h>

#include "arm.h"
#include "arm_internal.h"

/* [ORT §91 顺带] ort_handle_user_fault_kstack 的声明在
 * common/arm_ortcommon.h —— 其余调用点（dataabort/prefetchabort）都
 * include 了它，本文件漏了，一直以**隐式声明**编译（-Wimplicit 警告；
 * ARM AAPCS 下 bool/int 同走 r0 所以行为侥幸正确）。补上，消除侥幸。 */

#include "arm_ortcommon.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_undefinedinsn
 ****************************************************************************/

uint32_t *arm_undefinedinsn(uint32_t *regs)
{
  struct tcb_s *tcb = this_task();

  tcb->xcp.regs = regs;
  up_set_interrupt_context(true);

#ifdef CONFIG_ORT_MMU
  /* [ORT] 与 arm_dataabort.c / arm_prefetchabort.c 同一条规矩：
   * 用户态故障只终止进程，不 panic 内核。
   *
   * ★ 这一条原先**被漏掉了** —— data abort 与 prefetch abort 两条都被
   *   特意加过 ORT 处理，只有这里没有用户态判别、无条件 panic。
   *   后果（实测，见手册 §三·补三十七）：**容器执行一条未定义指令
   *   就能把整机打停机**。而这不需要恶意 —— 容器里一个被写坏的
   *   函数指针指到自己的数据上就够了。
   *
   *   判据与另外两条完全一样：CPSR 的模式位是不是 USR。
   *   三个向量必须语义相同，否则"容器能不能带走整机"就取决于
   *   它撞上的是哪一条。
   *
   *   第三个参数（被非法访问的地址）在这里就是这条指令自身的地址 ——
   *   未定义指令没有对应的 FAR 寄存器，PC 就是全部信息。
   */

  if ((regs[REG_CPSR] & PSR_MODE_MASK) == PSR_MODE_USR)
    {
      /* 同 arm_dataabort.c：必须走 _kstack 版本，并返回搬走后的帧 */

      if (ort_handle_user_fault_kstack(tcb, regs[REG_PC], regs[REG_PC]))
        {
          up_set_interrupt_context(false);
          return tcb->xcp.regs;
        }
    }
#endif

  if (regs[REG_PC] >= (uint32_t)_stext && regs[REG_PC] < (uint32_t)_etext)
    {
      _alert("Undefined instruction at 0x%" PRIx32 ": 0x%" PRIx32 "\n",
             regs[REG_PC], *(uint32_t *)regs[REG_PC]);
    }
  else
    {
      _alert("Undefined instruction at 0x%" PRIx32 "\n", regs[REG_PC]);
    }

  PANIC_WITH_REGS("panic", regs);
  return regs; /* To keep the compiler happy */
}
