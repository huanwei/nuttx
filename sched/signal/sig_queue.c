/****************************************************************************
 * sched/signal/sig_queue.c
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
#include <nuttx/compiler.h>

#include <signal.h>
#include <nuttx/debug.h>
#include <sched.h>
#include <errno.h>

#include <nuttx/signal.h>

#include "sched/sched.h"
#include "signal/signal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_queue
 *
 * Description:
 *   This function sends the signal specified by signo with the signal
 *   parameter value to the process specified by pid.
 *
 *   If the receiving process has the signal blocked via the sigprocmask,
 *   the signal will pend until it is unmasked. Only one pending signal (per
 *   signo) is retained.  This is consistent with POSIX which states, "If
 *   a subsequent occurrence of a pending signal is generated, it is
 *   implementation defined as to whether the signal is delivered more than
 *   once.
 *
 *   This is an internal OS interface.  It is functionally equivalent to
 *   sigqueue() except that it does not modify the errno value.
 *
 * Input Parameters:
 *   pid - Process ID of task to receive signal
 *   signo - Signal number
 *   value - Value to pass to task with signal
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 *    EGAIN  - The limit of signals which may be queued has been reached.
 *    EINVAL - sig was invalid.
 *    EPERM  - The  process  does  not  have  permission to send the
 *             signal to the receiving process.
 *    ESRCH  - No process has a PID matching pid.
 *
 ****************************************************************************/

static int nxsig_queue_common(int pid, int signo, union sigval value)
{
#ifdef CONFIG_SCHED_HAVE_PARENT
  FAR struct tcb_s *rtcb = this_task();
#endif
  siginfo_t info;

  sinfo("pid=0x%08x signo=%d value=%d\n", pid, signo, value.sival_int);

  /* Sanity checks */

  if (!GOOD_SIGNO(signo))
    {
      return -EINVAL;
    }

  /* Create the siginfo structure */

  info.si_signo           = signo;
  info.si_code            = SI_QUEUE;
  info.si_errno           = OK;
  info.si_value           = value;
#ifdef CONFIG_SCHED_HAVE_PARENT
  info.si_pid             = rtcb->pid;
  info.si_status          = OK;
#endif
  info.si_user            = NULL; /* Will be set in sig_dispatch.c */

  /* Send the signal */

  return nxsig_dispatch(pid, &info, false);
}

int nxsig_queue(int pid, int signo, union sigval value)
{
  /* [ORT §87] 跨组投递闸 —— 用户侧 sigqueue/kill 族走这里或 kill/tgkill
   * 同款闸；内核子系统对被监督对象的通知走 nxsig_queue_kernel（不过
   * 闸，见其说明）。 */

#if defined(CONFIG_ORT_CONTAINER) && defined(CONFIG_BUILD_KERNEL)
  {
    int gret = ort_sig_gate(pid);
    if (gret < 0)
      {
        return gret;
      }
  }
#endif

  return nxsig_queue_common(pid, signo, value);
}

/****************************************************************************
 * [ORT §87] 内核内部投递入口：**不过容器信号闸**。
 *
 *   为什么必须有它：ORT 的故障/退出唤醒是**跨组的内核通知**（故障者
 *   容器 → 监督者）—— 那是监督者设计的承重路径，不能被子系统自己的
 *   容器闸挡掉。语义与 nxsig_queue 逐字相同（同一 common 实现），
 *   差别只在"不查容器归属"。使用纪律：只许内核子系统调用；用户侧
 *   投递一律走 nxsig_queue / nxsig_kill / nxsig_tgkill。
 ****************************************************************************/

int nxsig_queue_kernel(int pid, int signo, union sigval value)
{
  return nxsig_queue_common(pid, signo, value);
}

/****************************************************************************
 * Name: sigqueue
 *
 * Description:
 *   This function sends the signal specified by signo with the signal
 *   parameter value to the process specified by pid.
 *
 *   If the receiving process has the signal blocked via the sigprocmask,
 *   the signal will pend until it is unmasked. Only one pending signal (per
 *   signo) is retained.  This is consistent with POSIX which states, "If
 *   a subsequent occurrence of a pending signal is generated, it is
 *   implementation defined as to whether the signal is delivered more than
 *   once."
 *
 * Input Parameters:
 *   pid - Process ID of task to receive signal
 *   signo - Signal number
 *   value - Value to pass to task with signal
 *
 * Returned Value:
 *    On  success (at least one signal was sent), zero (OK) is returned.  On
 *    any failure, -1 (ERROR) is returned and errno variable is set
 *    appropriately:
 *
 *    EGAIN  - The limit of signals which may be queued has been reached.
 *    EINVAL - sig was invalid.
 *    EPERM  - The  process  does  not  have  permission to send the
 *             signal to the receiving process.
 *    ESRCH  - No process has a PID matching pid.
 *
 ****************************************************************************/

int sigqueue(int pid, int signo, union sigval value)
{
  int ret;

  /* Let nxsig_queue() do all of the real work */

  ret = nxsig_queue(pid, signo, value);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}
