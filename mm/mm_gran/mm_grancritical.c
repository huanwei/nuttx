/****************************************************************************
 * mm/mm_gran/mm_grancritical.c
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

#include <stdlib.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/semaphore.h>
#include <nuttx/mm/gran.h>

#include "mm_gran/mm_gran.h"

#ifdef CONFIG_GRAN

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gran_enter_critical and gran_leave_critical
 *
 * Description:
 *   Critical section management for the granule allocator.
 *
 * Input Parameters:
 *   priv - Pointer to the gran state
 *
 * Returned Value:
 *   gran_enter_critical() may return any error reported by
 *   nxsem_wait_uninterruptible()
 *
 ****************************************************************************/

int gran_enter_critical(FAR struct gran_s *priv)
{
#ifdef CONFIG_GRAN_INTR
  priv->irqstate = spin_lock_irqsave(&priv->lock);
  return OK;
#else
  /* [ORT] 上游缺陷本地修复（第 8 处，手册 §三·补八十三）：改**不可
   * 中断**等待 —— 本函数的文档明写 "may return any error reported by
   * nxsem_wait_uninterruptible()"，且调用方（mm_granfree 等）的断言
   * 只放行 OK/-ECANCELED、并以 `while (ret < 0)` 重试；但实现调的是
   * 可中断的 nxmutex_lock（nxsem_wait），**信号雨**下等待返回 -EINTR
   * ⇒ 断言打停机。本系统实测触发：容器故障的唤醒信号打在 orting 的
   * gran 等待窗口（1/N 复现）。文档与实现取其一 —— 按文档修。 */

  return nxsem_wait_uninterruptible(&priv->lock.sem);
#endif
}

void gran_leave_critical(FAR struct gran_s *priv)
{
#ifdef CONFIG_GRAN_INTR
  spin_unlock_irqrestore(&priv->lock, priv->irqstate);
#else
  nxmutex_unlock(&priv->lock);
#endif
}

#endif /* CONFIG_GRAN */
