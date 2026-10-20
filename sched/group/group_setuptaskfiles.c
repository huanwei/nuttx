/****************************************************************************
 * sched/group/group_setuptaskfiles.c
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

#include <sched.h>
#include <assert.h>
#include <syslog.h>

#include <nuttx/fs/fs.h>
#include <nuttx/trace.h>

#include "sched/sched.h"
#include "group/group.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: group_setuptaskfiles
 *
 * Description:
 *   Configure a newly allocated TCB so that it will inherit
 *   file descriptors and streams from the parent task.
 *
 * Input Parameters:
 *   tcb     - tcb of the new task.
 *   actions - The spawn file actions
 *   cloexec - Perform O_CLOEXEC on setup task files
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int group_setuptaskfiles(FAR struct tcb_s *tcb,
                         FAR const posix_spawn_file_actions_t *actions,
                         bool cloexec)
{
  FAR struct task_group_s *group = tcb->group;
  int ret = OK;
#ifndef CONFIG_FDCLONE_DISABLE
  FAR struct tcb_s *rtcb = this_task();
  bool stdio_only = false;
#endif

  sched_trace_begin();
  DEBUGASSERT(group);
#ifndef CONFIG_DISABLE_PTHREAD
  DEBUGASSERT((tcb->flags & TCB_FLAG_TTYPE_MASK) !=
              TCB_FLAG_TTYPE_PTHREAD);
#endif

#ifndef CONFIG_FDCLONE_DISABLE
  DEBUGASSERT(rtcb->group);

  /* With the exception of kernel threads, duplicate the parent task's
   * file descriptors.
   */

  if (group != rtcb->group &&
      (tcb->flags & TCB_FLAG_TTYPE_MASK) != TCB_FLAG_TTYPE_KERNEL)
    {
#ifdef CONFIG_ORT_CONTAINER
      /* [ORT §112] 继承 fd 收口（A 侧咽喉；M 的绑域容器在准入返回
       * 时自清，见 ort_wait_admission）：
       *   · 父**不**重挂、子重挂 = host→容器跨越 → 只复制 stdio
       *     （设备/宿主文件/目录 fd 一律不跨越；stdio 是契约面）。
       *   · 父也重挂 = 容器内部派生 → 保持完整继承（POSIX 语义）。
       * 任一容器组复制完成后置 fdsafe，准入清扫不再重复。 */

      if (group->tg_ort_re_root && !rtcb->group->tg_ort_re_root)
        {
          stdio_only = true;
        }
#endif

      ret = fdlist_copy(&rtcb->group->tg_fdlist,
                        &group->tg_fdlist, actions, cloexec, stdio_only);

#ifdef CONFIG_ORT_CONTAINER
      if (ret >= 0 && group->tg_ort_re_root)
        {
          group->tg_ort_fdsafe = true;

          if (stdio_only)
            {
              syslog(LOG_ERR,
                     "ORT: fdguard: stdio-only lpid=%u\n",
                     (unsigned)group->tg_ort_lpid);
            }
        }
#endif
    }

  if (ret >= 0 && actions != NULL &&
      (tcb->flags & TCB_FLAG_TTYPE_MASK) != TCB_FLAG_TTYPE_KERNEL)
    {
      ret = spawn_file_actions(tcb, actions);
    }
#endif

  sched_trace_end();
  return ret;
}
