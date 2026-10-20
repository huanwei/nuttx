/****************************************************************************
 * sched/tls/tls_initinfo.c
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

#include <assert.h>
#include <errno.h>

#include "tls.h"
#include "sched/sched.h"   /* [§109] ort_pidns_next_tid 原型 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tls_init_info
 *
 * Description:
 *   Allocate and initialize tls_info_s structure.
 *
 * Input Parameters:
 *   - tcb: The TCB of new task
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int tls_init_info(FAR struct tcb_s *tcb)
{
  FAR struct tls_info_s *info;

  /* Allocate thread local storage */

  info = up_stack_frame(tcb, tls_info_size());
  if (info == NULL)
    {
      return -ENOMEM;
    }

  DEBUGASSERT(info == tcb->stack_alloc_ptr);

  /* Initialize thread local storage */

  up_tls_initialize(info);

  /* Derive tl_size w/o arch knowledge */

  info->tl_size =
        (FAR char *)tcb->stack_base_ptr - (FAR char *)tcb->stack_alloc_ptr;

  /* Attach per-task info in group to TLS */

  info->tl_task = tcb->group->tg_info;

  /* Thread ID */

#ifdef CONFIG_ORT_CONTAINER
  /* [ORT §109] 容器**所有线程**的 gettid 视图 = 本线程本地 tid：
   *   组长 = 组 lpid（join 时已落 ort_ltid，本处沿用）；pthread =
   *   从同实例**同一计数器**取号（Linux：tid 与 pid 同空间）。
   *   计数器耗尽（返回 0）→ 降级为全局号（fail-open，如实）。
   * 咽喉：knsh 用户态 gettid() 读 TLS tl_tid（本函数对 pthread 也
   * 调用 —— pthread_create.c；§102 时曾排除 pthread，§109 收口）。 */

  if (tcb->group != NULL && tcb->group->tg_ort_nsid != 0)
    {
      if (tcb->ort_ltid == 0)
        {
          tcb->ort_ltid = ort_pidns_next_tid(tcb->group->tg_ort_nsid);
        }

      info->tl_tid = tcb->ort_ltid != 0 ? (pid_t)tcb->ort_ltid
                                        : tcb->pid;
    }
  else
#endif
    {
      info->tl_tid = tcb->pid;
    }

  /* Initialize the starting address of argv to NULL to prevent
   * it from being misused.
   */

  info->tl_argv = NULL;

  return OK;
}
