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
  /* [ORT §102] 容器**主线程**的 gettid 视图 = 组本地号（Linux 口径
   * tid==pid；容器 init getpid=gettid=1）。pthread 线程不在此虚拟化
   * （线程号面仍全局 —— 边界如实，与 tgkill 的 tid 面一起留后续刀）。
   * 落点在此的原因：knsh 用户态 gettid() = 读 TLS tl_tid（一处咽喉）；
   * 本函数对 pthread 也调用（pthread_create.c）——故必须排除 pthread。 */

  if (tcb->group != NULL && tcb->group->tg_ort_lpid != 0 &&
      (tcb->flags & TCB_FLAG_TTYPE_MASK) != TCB_FLAG_TTYPE_PTHREAD)
    {
      info->tl_tid = (pid_t)tcb->group->tg_ort_lpid;
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
