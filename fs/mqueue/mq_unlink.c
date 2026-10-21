/****************************************************************************
 * fs/mqueue/mq_unlink.c
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

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <mqueue.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/debug.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mqueue.h>
#include <nuttx/sched.h>
#include <nuttx/wqueue.h>

#include "sched/sched.h"     /* [§106] this_task() 在内部头（同 fs_open.c） */
#include "inode/inode.h"
#include "mqueue/mqueue.h"
#include "vfs/vfs.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mq_inode_release
 *
 * Description:
 *   Release a reference count on a message queue inode.
 *
 * Input Parameters:
 *   inode - The message queue inode
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mq_inode_release(FAR struct inode *inode)
{
  if (atomic_read(&inode->i_crefs) <= 1)
    {
      FAR struct mqueue_inode_s *msgq = inode->i_private;

      if (msgq)
        {
          nxmq_free_msgq(msgq);
          inode->i_private = NULL;
        }
    }

  inode_release(inode);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_mq_unlink
 *
 * Description:
 *   This is an internal OS interface.  It is functionally equivalent to
 *   mq_unlink() except that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 *  See comments with mq_unlink() for a more complete description of the
 *  behavior of this function
 *
 * Input Parameters:
 *   mq_name - Name of the message queue
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success. A negated errno value is returned on failure.
 *
 ****************************************************************************/

int file_mq_unlink(FAR const char *mq_name)
{
  FAR struct inode *inode;
  struct inode_search_s desc;
  char fullpath[MAX_MQUEUE_PATH];
  int ret;

  /* Get the full path to the message queue.  [ORT §111] 与 mq_open 同一
   * 翻译咽喉（容器 → 实例前缀 + 单组件校验；宿主 → 原语义）。 */

  ret = nxmq_ort_fullpath(mq_name, fullpath, sizeof(fullpath));
  if (ret < 0)
    {
      return ret;
    }

  /* Get the inode for this message queue.
   * [§111] 前缀路径在全局树解析（容器 root 不参与）。 */

  SETUP_SEARCH(&desc, fullpath, false);
  desc.global = true;

  ret = inode_find(&desc);
  if (ret < 0)
    {
      /* There is no inode that includes in this path */

      goto errout_with_search;
    }

  /* Get the search results */

  inode = desc.node;

  /* Verify that what we found is, indeed, a message queue */

  if (!INODE_IS_MQUEUE(inode))
    {
      ret = -ENXIO;
      goto errout_with_inode;
    }

  /* [ORT §111] 隔离由命名空间承担：容器的名字只会解析到自己的前缀
   * 下，删不到全局/其它实例的队列；§106 的"容器全拒"闸退役。 */

  /* Refuse to unlink the inode if it has children.  I.e., if it is
   * functioning as a directory and the directory is not empty.
   */

  inode_lock();
  if (inode->i_child != NULL)
    {
      ret = -ENOTEMPTY;
      goto errout_with_lock;
    }

  /* Remove the old inode from the tree.  Because we hold a reference count
   * on the inode, it will not be deleted now. This will put reference of
   * inode.
   */

  ret = inode_remove_global(fullpath);   /* [§111] 与查找同一解析面 */

  /* The inode may has been unlinked from the tree in other core,
   * but it is not yet freed.
   */

  if (ret < 0 && ret != -EBUSY)
    {
      goto errout_with_lock;
    }

  /* Now we do not release the reference count in the normal way (by calling
   * inode release.  Rather, we call mq_inode_release().  mq_inode_release
   * will decrement the reference count on the inode.  But it will also free
   * the message queue if that reference count decrements to zero.  Since we
   * hold one reference, that can only occur if the message queue is not
   * in-use.
   */

  inode_unlock();
  mq_inode_release(inode);
  RELEASE_SEARCH(&desc);
#ifdef CONFIG_FS_NOTIFY
  notify_unlink(fullpath);
#endif
  return OK;

errout_with_lock:
  inode_unlock();

errout_with_inode:
  inode_release(inode);

errout_with_search:
  RELEASE_SEARCH(&desc);
  return ret;
}

/****************************************************************************
 * Name: nxmq_unlink
 *
 * Description:
 *   This is an internal OS interface.  It is functionally equivalent to
 *   mq_unlink() except that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 *  See comments with mq_unlink() for a more complete description of the
 *  behavior of this function
 *
 * Input Parameters:
 *   mq_name - Name of the message queue
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success. A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxmq_unlink(FAR const char *mq_name)
{
  return file_mq_unlink(mq_name);
}

/****************************************************************************
 * Name: mq_unlink
 *
 * Description:
 *   This function removes the message queue named by "mq_name." If one
 *   or more tasks have the message queue open when mq_unlink() is called,
 *   removal of the message queue is postponed until all references to the
 *   message queue have been closed.
 *
 * Input Parameters:
 *   mq_name - Name of the message queue
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

int mq_unlink(FAR const char *mq_name)
{
  int ret;

  ret = nxmq_unlink(mq_name);
  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: nxmq_purge_prefix
 *
 * Description:
 *   [ORT §114] 删除 /var/mqueue 下所有以 prefix 开头的队列（叶子名）。
 *   用于实例生命周期：M 的域键在**绑域前**清掉旧实例的遗留（域键复用
 *   ⇒ 不清理会让新实例读到旧实例的消息）。前缀由内核构造、可信；
 *   逐名走 file_mq_unlink（与 mq_unlink 同一语义，含引用计数释放）。
 *
 * Returned Value:
 *   删除条数（≥0）；失败返回负 errno（调用者 fail-closed）。
 *
 ****************************************************************************/

int nxmq_purge_prefix(FAR const char *prefix)
{
  struct inode_search_s desc;
  char name[CONFIG_NAME_MAX + 1];
  size_t plen;
  int n = 0;

  if (prefix == NULL || *prefix == '\0')
    {
      return -EINVAL;
    }

  plen = strlen(prefix);

  for (;;)
    {
      FAR struct inode *dir;
      FAR struct inode *child;
      bool found = false;
      int ret;

      SETUP_SEARCH(&desc, CONFIG_FS_MQUEUE_VFS_PATH, false);
      desc.global = true;

      ret = inode_find(&desc);
      if (ret == -ENOENT)
        {
          /* 命名空间尚不存在（本启动还没建过任何队列）= 无可清理 */

          RELEASE_SEARCH(&desc);
          return 0;
        }
      else if (ret < 0)
        {
          RELEASE_SEARCH(&desc);
          return ret;
        }

      dir = desc.node;

      inode_lock();
      for (child = dir->i_child; child != NULL; child = child->i_peer)
        {
          if (child->i_name != NULL &&
              strlen(child->i_name) <= CONFIG_NAME_MAX &&
              strncmp(child->i_name, prefix, plen) == 0)
            {
              strlcpy(name, child->i_name, sizeof(name));
              found = true;
              break;
            }
        }
      inode_unlock();

      inode_release(dir);
      RELEASE_SEARCH(&desc);

      if (!found)
        {
          break;
        }

      ret = file_mq_unlink(name);
      if (ret < 0 && ret != -ENOENT)
        {
          _alert("ORT: mqueue ns purge failed: %s (%d)\n", name, ret);
          return ret;
        }

      n++;
    }

  if (n > 0)
    {
      _alert("ORT: mqueue ns purge: %s (%d)\n", prefix, n);
    }

  return n;
}

/****************************************************************************
 * Name: nxmq_purge_ns_async
 *
 * Description:
 *   [ORT §115] 实例死亡时异步回收其 mqueue 命名空间（见头文件说明）。
 *   用 LPWORK：组销毁上下文（group_drop）不做 fs 工作；工作项自带
 *   nsid，nsid 单调不复用 ⇒ 清理不会碰到新实例。
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_LPWORK
struct nxmq_purge_ns_s
{
  struct work_s work;
  uint32_t nsid;
};

static void nxmq_purge_ns_worker(FAR void *arg)
{
  FAR struct nxmq_purge_ns_s *p = (FAR struct nxmq_purge_ns_s *)arg;
  char prefix[32];

  snprintf(prefix, sizeof(prefix), "ort.n%u.", (unsigned)p->nsid);
  nxmq_purge_prefix(prefix);
  kmm_free(p);
}
#endif

int nxmq_purge_ns_async(uint32_t nsid)
{
#ifdef CONFIG_SCHED_LPWORK
  FAR struct nxmq_purge_ns_s *p;
  int ret;

  if (nsid == 0)
    {
      return OK;
    }

  p = kmm_malloc(sizeof(*p));
  if (p == NULL)
    {
      _alert("ORT: mqueue ns purge alloc failed: ort.n%u.\n",
             (unsigned)nsid);
      return -ENOMEM;
    }

  memset(p, 0, sizeof(*p));
  p->nsid = nsid;

  ret = work_queue(LPWORK, &p->work, nxmq_purge_ns_worker, p, 0);
  if (ret < 0)
    {
      _alert("ORT: mqueue ns purge queue failed: ort.n%u. (%d)\n",
             (unsigned)nsid, ret);
      kmm_free(p);
    }

  return ret;
#else
  UNUSED(nsid);
  return -ENOSYS;
#endif
}
