/****************************************************************************
 * fs/mqueue/mqueue.h
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

#ifndef __FS_MQUEUE_MQUEUE_H
#define __FS_MQUEUE_MQUEUE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stddef.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Sizes of things */

#define MAX_MQUEUE_PATH 64

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* [ORT §111] mqueue 名字 → VFS 路径（容器实例命名空间翻译）。
 * 实现见 mq_open.c；mq_unlink 共用同一咽喉。 */

int nxmq_ort_fullpath(FAR const char *mq_name, FAR char *fullpath,
                      size_t fullsize);

#endif /* __FS_MQUEUE_MQUEUE_H */
