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
#ifdef CONFIG_ORT_CONTAINER
      case PR_SET_ORT_DOMAIN:
        {
          /* [ORT] 把**指定容器**绑定到指定域。参数：(int domain, pid_t pid)
           *
           * 注意是「容器」不是「任务」：域绑在 task_group_s 上，
           * 同一容器的所有线程共享一个域。
           *
           * ★ 只有 ORT 监督者能调用（-EPERM 拒绝其它调用者）。
           *   容器不能自己申报域 —— 域号就是内存块号，能自选就能选到
           *   别的容器的块。权限判断在 ort_container_bind() 里统一做，
           *   sched/ 不需要知道谁是监督者，也不需要知道域的编码方式。
           */

          int domain = va_arg(ap, int);
          int pid    = va_arg(ap, int);

          return ort_container_bind((pid_t)pid, domain);
        }

      case PR_GET_ORT_DOMAIN:
        {
          /* [ORT] 查询**本容器**的域。返回域号，未绑定返回 -1。
           *
           * 不需要权限：容器当然可以知道自己被分到哪个域 ——
           * 那是只读信息，知道域号也不能访问别的域。
           */

          return ort_container_domain(this_task()->group);
        }

#ifdef CONFIG_ORT_SUPERVISOR_RESET
      case PR_ORT_SUPERVISOR_RESET:
        {
          /* [ORT] ⚠️ 仅原型测试：释放监督者槽位。
           * 正式构建里本分支不存在（见 sched/Kconfig）。
           */

          return ort_supervisor_reset();
        }
#endif

      case PR_SET_ORT_SUPERVISOR:
        {
          /* [ORT] 把自己注册为监督者：内核在容器故障时通知它。
           * 只接受首次注册，重复注册返回 -EBUSY。
           *
           * ★ 资格检查（先于槽位检查）：只有**构建期白名单**
           *   （CONFIG_ORT_SUPERVISOR_TASKNAMES）里的任务名能首次注册。
           *   缺口原文是"首次注册先到先得"——任何任务抢先调用就能占住
           *   槽位；白名单把资格从"运行期先到先得"收敛为"构建期声明"。
           *
           *   ⚠️ 诚实边界：同一构建内任务名不是安全边界（能改名的任务
           *   就能过线）。完整鉴权待产品化；此处按实际强度标注。
           */

          {
            FAR const char *wn = CONFIG_ORT_SUPERVISOR_TASKNAMES;
            FAR const char *tn = this_task()->name;
            size_t tl = strlen(tn);
            bool   ok = false;

            while (*wn != '\0')
              {
                FAR const char *comma = strchr(wn, ',');
                size_t          wl    = comma ? (size_t)(comma - wn)
                                              : strlen(wn);

                if (wl == tl && strncmp(wn, tn, wl) == 0)
                  {
                    ok = true;
                    break;
                  }

                wn = comma ? comma + 1 : wn + wl;
              }

            if (!ok)
              {
                _alert("ORT: supervisor registration rejected: "
                       "task \"%s\" not in [%s]\n",
                       tn, CONFIG_ORT_SUPERVISOR_TASKNAMES);
                return -EPERM;
              }
          }

          return ort_supervisor_set((int)this_task()->pid);
        }

      case PR_GET_ORT_FAULT:
        {
          /* [ORT] 取下一条未读的容器故障事件（队列语义）。
           *
           * ⚠️ 原型期直接按用户指针写 —— PROTECTED 构建下内核能访问
           *    用户内存所以可行，但**没有做指针合法性校验**。
           *    正式实现必须校验（或改为内核侧环形缓冲 + 只读文件接口）。
           */

          FAR struct ort_faultrec_s *rec =
              (FAR struct ort_faultrec_s *)va_arg(ap, uintptr_t);

          return ort_fault_read(rec);
        }

      case PR_ORT_STATE_PUT:
        {
          /* [ORT] 发布状态快照。域号由内核从调用者的 group 取，
           * 调用者无法指定 —— 见 <sys/prctl.h> 的说明。
           *
           * ⚠️ 同 PR_GET_ORT_FAULT：原型期直接按用户指针读写，
           *    没做指针合法性校验。正式实现必须校验。
           */

          FAR const void *buf = (FAR const void *)va_arg(ap, uintptr_t);
          size_t len          = (size_t)va_arg(ap, int);

          return ort_state_put(buf, len);
        }

      case PR_ORT_STATE_GET:
        {
          FAR void *buf = (FAR void *)va_arg(ap, uintptr_t);
          size_t len    = (size_t)va_arg(ap, int);

          return ort_state_get(buf, len);
        }

      case PR_GET_ORT_STATE_SEQ:
        {
          /* [ORT] 某域发布过多少次。纯标量，不碰用户指针。
           *
           * 权限在实现里查（只有监督者能调）—— 见 ort_state_seq()。
           */

          return ort_state_seq((int)va_arg(ap, int));
        }

      case PR_SET_ORT_DEPLOY:
        {
          /* [ORT] 注册部署/O&M 代理。首次注册即钉住，同监督者规则。 */

          return ort_deploy_set((int)this_task()->pid);
        }

#ifdef CONFIG_ORT_SUPERVISOR_RESET
      case PR_ORT_TEST_FAULT:
        {
          /* [ORT] ⚠️ 仅原型测试：注入故障事件，压故障队列。见 <sys/prctl.h>。 */

          return ort_fault_inject((int)va_arg(ap, int));
        }

      case PR_ORT_DEPLOY_RESET:
        {
          ort_deploy_reset();
          return OK;
        }

      case PR_ORT_TEST_SIGNOFF:
        {
          /* [ORT] ⚠️ 仅原型测试：开关故障通知信号。见 <sys/prctl.h>。
           *
           * 与注入接口同样**对任何任务开放** —— 它是照实验的旋钮，
           * 不是安全接口；而且对照实验里拧它的任务往往不是监督者。 */

          return ort_fault_signal_set((int)va_arg(ap, int));
        }
#endif

      case PR_ORT_CFG_PUT:
        {
          /* [ORT] 代理写入 manifest 原样字节。只有钉住的代理能调。
           *
           * ⚠️ 同 PR_GET_ORT_FAULT：原型期直接按用户指针读，没做校验。 */

          FAR const void *buf = (FAR const void *)va_arg(ap, uintptr_t);
          size_t len          = (size_t)va_arg(ap, int);

          return ort_cfg_put(buf, len);
        }

      case PR_ORT_CFG_ALIVE:
        {
          /* [ORT] 代理心跳。只有钉住的代理能调。 */

          return ort_cfg_alive();
        }

      case PR_GET_ORT_CFG_SEQ:
        {
          return ort_cfg_seq();
        }

      case PR_GET_ORT_CFG_TICK:
        {
          return ort_cfg_tick();
        }

      case PR_ORT_CFG_GET:
        {
          FAR void *buf = (FAR void *)va_arg(ap, uintptr_t);
          size_t cap    = (size_t)va_arg(ap, int);

          return ort_cfg_get(buf, cap);
        }

      case PR_GET_ORT_CAPS:
        {
          /* [ORT] 本平台的能力位。直接返回掩码，不碰用户指针。
           *
           * 存在的理由：两个 SKU 的能力差异此前是**隐式**的 ——
           * 同一份 manifest 在两个 SKU 上行为不同，却没地方问。
           * 有了它，监督者就能在**准入**阶段拒绝一个自己兑现不了的
           * 声明，而不是等运行期默默降级。
           *
           * 具体有哪些位、每个位的确切含义，见 <sys/prctl.h>。
           */

          return (int)ort_caps();
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
