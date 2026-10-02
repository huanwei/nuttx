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

#ifdef CONFIG_BUILD_PROTECTED
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
          int ret = -ESRCH;

          _alert("ORT: USER TASK MEMFAULT pid=%d pc=%08" PRIxPTR
                 " addr=%08" PRIx32 " -> terminate task\n",
                 ftcb != NULL ? ftcb->pid : -1, pc,
                 getreg32(NVIC_MEMMANAGE_ADDR));

          /* 先清 fault 状态，避免异常返回时重新触发同一个 fault */

          putreg32(0xff, NVIC_CFAULTS);
          putreg32(0, NVIC_MEMMANAGE_ADDR);

          if (ftcb == NULL)
            {
              _alert("ORT: user memfault with no TCB -> fail-stop\n");
              up_irq_save();
              PANIC_WITH_REGS("user memfault: no tcb", context);
            }

          /* ── 第一步：投递 SIGSEGV，给容器/监督者一个可观测点 ──────────
           *
           * 为什么用信号而不是直接 nxtask_exit()：
           *   1. 不能从异常处理器直接调用 nxtask_exit() —— 它期望在任务上下文执行，
           *      在异常返回路径上调用会导致上下文切换无法完成（实测系统挂起）
           *   2. 信号由 NuttX 在「返回用户态时」投递，时机正确
           *   3. 应用可注册 SIGSEGV 处理器 —— 使「容器越界」成为可感知事件，
           *      直接对接 ORT 降级状态机的 onFailure 策略
           *
           * 只在「首次故障」时发：若处理完 SIGSEGV 又回到故障指令，
           * 再发一遍没有意义（见第二步）。
           */

          if (ftcb->xcp.fault_count == 0)
            {
              ret = nxsig_kill(ftcb->pid, SIGSEGV);
            }

          ftcb->xcp.fault_count++;

          /* ── 第二步：保证容器一定会死（★ 监督者的终止权）──────────────
           *
           * 为什么 SIGSEGV 不足以终止容器：
           *   POSIX 允许忽略 SIGSEGV（结果未定义），NuttX 亦然 ——
           *   sig_action.c 只对 SIG_FLAG_NOCATCH 的信号返回 -EINVAL，
           *   而 SIGSEGV 没设这个标志。容器有两种办法逃过终止：
           *
           *   (a) sigaction(SIGSEGV, SIG_IGN)
           *       NuttX 把 SIG_IGN 规范化成「从 tg_sigactionq 里删掉这个动作」
           *       （sig_action.c "Handle the case where no sigaction is
           *       supplied (SIG_IGN)"），于是 nxsig_find_action() 返回 NULL，
           *       nxsig_queue_action() 整段跳过 —— 什么都没投出去。
           *       → 表现为 sigdeliver == NULL，第一次 fault 就升级
           *
           *   (b) 注册一个「打印一下就返回」的处理器
           *       SIGSEGV 正常投递、处理器正常返回，然后异常返回**回到同一条
           *       故障指令**上再次 fault —— 无限循环卡死 CPU。
           *       → 表现为 fault_count 涨到 2，第二次 fault 升级
           *
           * 为什么 SIGKILL 可以依赖：
           *   CONFIG_SIG_DEFAULT 给 SIGKILL 设了 SIG_FLAG_NOCATCH，
           *   sigaction(SIGKILL, SIG_IGN) 返回 -EINVAL —— 容器改不掉它。
           *   而且它的默认动作（nxsig_abnormal_termination）由内核在任务
           *   启动时安装，容器也删不掉。
           */

          if (ftcb->xcp.fault_count > 1 || ftcb->sigdeliver == NULL)
            {
              _alert("ORT: escalating pid=%d to SIGKILL (faults=%d)\n",
                     ftcb->pid, ftcb->xcp.fault_count);
              ret = nxsig_kill(ftcb->pid, SIGKILL);
            }

          /* ★ 不跳过 faulting 指令。
           *
           * 为什么「跳到下一条指令」是错的（实测踩过）：
           *   nxsig_kill() → nxsig_queue_action() 一旦发现任务存在信号动作，
           *   就会调用 up_schedule_sigaction() 触发 PendSV；PendSV 上
           *   up_schedule_sigaction() 会把保存的上下文整体复制一份、把 PC 改写成
           *   arm_sigdeliver。任务永远不会再回到这条 faulting 指令 ——
           *   跳过指令不但多余，还会让任务带着被截断的状态继续跑。
           *
           *   另外，手工解码 Thumb 指令长度（16/32 位）本身就不可靠。
           */

          /* 最后兜底：连 SIGKILL 都投不出去（CONFIG_SIG_DEFAULT 没开）
           * 说明任何信号都不会被处理，异常返回后必然无限 fault。
           * 此时唯一诚实的做法是 fail-stop，而不是假装没事继续跑。
           */

          if (ret < 0 || ftcb->sigdeliver == NULL)
            {
              _alert("ORT: cannot terminate pid=%d (ret=%d) -> fail-stop\n"
                     "     (CONFIG_SIG_DEFAULT=y 是 ORT 的必需配置)\n",
                     ftcb->pid, ret);
              up_irq_save();
              PANIC_WITH_REGS("user memfault: undeliverable", context);
            }

          /* 正常异常返回 → PendSV → 信号投递 → 容器被终止 */

          return OK;
        }
    }
#endif

  up_irq_save();
  PANIC_WITH_REGS("panic", context);
  return OK; /* Won't get here */
}
