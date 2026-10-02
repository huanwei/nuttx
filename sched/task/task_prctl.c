/****************************************************************************
 * sched/task/task_prctl.c
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

#include <sys/prctl.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/sched.h>

#include "sched/sched.h"
#include "task/task.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: prctl
 *
 * Description:
 *   prctl() is called with a first argument describing what to do (with
 *   values PR_* defined above) and with additional arguments depending on
 *   the specific command.
 *
 * Returned Value:
 *   The returned value may depend on the specific command.  For PR_SET_NAME
 *   and PR_GET_NAME, the returned value of 0 indicates successful operation.
 *   On any failure, -1 is retruend and the errno value is set appropriately.
 *
 *     EINVAL The value of 'option' is not recognized.
 *     EFAULT optional arg1 is not a valid address.
 *     ESRCH  No task/thread can be found corresponding to that specified
 *       by optional arg1.
 *
 ****************************************************************************/

int prctl(int option, ...)
{
  va_list ap;
  int errcode;

  va_start(ap, option);
  switch (option)
    {
#ifdef CONFIG_ORT_MEMDOMAIN
      case PR_SET_ORT_DOMAIN:
        {
          /* [ORT] 把调用任务绑定到指定域。
           * 参数：int domain（< 0 = 解除绑定）
           * 返回值：0 成功
           *
           * 越界一律降级为「解除绑定」，由 ort_memdomain_bind() 统一处理 ——
           * sched/ 不需要知道「未绑定」是怎么编码的，也不需要知道域有几种。
           */

          ort_memdomain_bind(this_task(), va_arg(ap, int));
          return OK;
        }

      case PR_SET_ORT_SUPERVISOR:
        {
          /* [ORT] 把自己注册为监督者：内核在容器故障时通知它。
           * 只接受首次注册，重复注册返回 -EBUSY。
           */

          return ort_supervisor_set((int)this_task()->pid);
        }

      case PR_GET_ORT_FAULT:
        {
          /* [ORT] 取回最近一次容器故障记录。
           *
           * ⚠️ 原型期直接按用户指针写 —— PROTECTED 构建下内核能访问
           *    用户内存所以可行，但**没有做指针合法性校验**。
           *    正式实现必须校验（或改为内核侧环形缓冲 + 只读文件接口）。
           */

          FAR struct ort_faultrec_s *rec =
              (FAR struct ort_faultrec_s *)va_arg(ap, uintptr_t);

          if (rec == NULL)
            {
              return -EINVAL;
            }

          ort_fault_record(rec);
          return (int)rec->seq;
        }
#endif

      case PR_SET_NAME:
      case PR_GET_NAME:
      case PR_SET_NAME_EXT:
      case PR_GET_NAME_EXT:
#if CONFIG_TASK_NAME_SIZE > 0
        {
          /* Get the prctl arguments */

          FAR char *name = va_arg(ap, FAR char *);
          FAR struct tcb_s *tcb;
          int pid = 0;

          if (option == PR_SET_NAME_EXT ||
              option == PR_GET_NAME_EXT)
            {
              pid = va_arg(ap, int);
            }

          /* Get the TCB associated with the PID (handling the special case
           * of pid==0 meaning "this thread")
           */

          if (pid == 0)
            {
              tcb = this_task();
            }
          else
            {
              tcb = nxsched_get_tcb(pid);
            }

          /* An invalid pid will be indicated by a NULL TCB returned from
           * nxsched_get_tcb()
           */

          if (tcb == NULL)
            {
              serr("ERROR: Pid does not correspond to a task: %d\n", pid);
              errcode = ESRCH;
              goto errout;
            }

          /* A pointer to the task name storage must also be provided */

          if (name == NULL)
            {
              serr("ERROR: No name provide\n");
              errcode = EFAULT;
              goto errout;
            }

          /* Now get or set the task name */

          if (option == PR_SET_NAME || option == PR_SET_NAME_EXT)
            {
              /* Ensure that tcb->name will be null-terminated, truncating if
               * necessary.
               */

              strlcpy(tcb->name, name, sizeof(tcb->name));
              tcb->name[CONFIG_TASK_NAME_SIZE] = '\0';
            }
          else
            {
              /* The returned value will be null-terminated, truncating if
               * necessary.
               */

              strlcpy(name, tcb->name, sizeof(tcb->name));
              name[CONFIG_TASK_NAME_SIZE - 1] = '\0';
            }
        }
        break;
#else
        serr("ERROR: Option not enabled: %d\n", option);
        errcode = ENOSYS;
        goto errout;
#endif

      default:
        serr("ERROR: Unrecognized option: %d\n", option);
        errcode = EINVAL;
        goto errout;
    }

  /* Not reachable unless CONFIG_TASK_NAME_SIZE is > 0.  NOTE: This might
   * change if additional commands are supported.
   */

#if CONFIG_TASK_NAME_SIZE > 0
  va_end(ap);
  return OK;
#endif

errout:
  va_end(ap);
  set_errno(errcode);
  return ERROR;
}
