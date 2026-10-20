/****************************************************************************
 * fs/mqueue/mq_open.c
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

#include <sys/stat.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sched.h>
#include <mqueue.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/debug.h>
#include <nuttx/mqueue.h>
#include <nuttx/sched.h>
#include <nuttx/fs/fs.h>

#include "sched/sched.h"     /* [§106] this_task() 在内部头（同 fs_open.c） */
#include "inode/inode.h"
#include "mqueue/mqueue.h"
#include "vfs/vfs.h"

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

static int nxmq_file_close(FAR struct file *filep);
static int nxmq_file_poll(FAR struct file *filep,
                          FAR struct pollfd *fds, bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_nxmq_fileops =
{
  NULL,             /* open */
  nxmq_file_close,  /* close */
  NULL,             /* read */
  NULL,             /* write */
  NULL,             /* seek */
  NULL,             /* ioctl */
  NULL,             /* mmap */
  NULL,             /* truncate */
  nxmq_file_poll    /* poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int nxmq_file_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;

  if (atomic_read(&inode->i_crefs) <= 1)
    {
      FAR struct mqueue_inode_s *msgq = inode->i_private;

      if (msgq)
        {
          nxmq_free_msgq(msgq);
          inode->i_private = NULL;
        }
    }

  return 0;
}

static int nxmq_file_poll(FAR struct file *filep,
                          FAR struct pollfd *fds, bool setup)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mqueue_inode_s *msgq = inode->i_private;
  pollevent_t eventset = 0;
  irqstate_t flags;
  int ret = 0;
  int i;

  flags = enter_critical_section();

  if (setup)
    {
      for (i = 0; i < CONFIG_FS_MQUEUE_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (msgq->fds[i] == NULL)
            {
              /* Bind the poll structure and this slot */

              msgq->fds[i] = fds;
              fds->priv    = &msgq->fds[i];
              break;
            }
        }

      if (i >= CONFIG_FS_MQUEUE_NPOLLWAITERS)
        {
          fds->priv = NULL;
          ret       = -EBUSY;
          goto errout;
        }

      /* Immediately notify on any of the requested events */

      if (msgq->nmsgs < msgq->maxmsgs)
        {
          eventset |= POLLOUT;
        }

      if (msgq->nmsgs > 0)
        {
          eventset |= POLLIN;
        }

      poll_notify(&fds, 1, eventset);
    }
  else if (fds->priv != NULL)
    {
      for (i = 0; i < CONFIG_FS_MQUEUE_NPOLLWAITERS; i++)
        {
          if (fds == msgq->fds[i])
            {
              msgq->fds[i] = NULL;
              fds->priv = NULL;
              break;
            }
        }
    }

errout:
  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: nxmq_ort_fullpath
 *
 * Description:
 *   [ORT §111] IPC 面第二刀：把用户给的 mqueue 名字转成 VFS 路径。
 *   · 宿主/非容器：沿既有语义（去掉前导 '/'，其余原样 —— 不上新锁）。
 *   · 容器（re_root || domain != 0）：只允许**单组件**名字（无 '/'、
 *     非 "."/".."）—— 内核构造的路径不可能逃出 /var/mqueue；然后按
 *     **实例**加前缀：
 *       nsid 非零   → "ort.n<nsid>.<name>"（A/M 设根容器）
 *       domain 非零 → "ort.d<domain>.<name>"（M 绑域容器）
 *     两种前缀不同命名域，杜绝 nsid 与 domain 数值撞车。无键 →
 *     -EPERM（fail-closed，绝不放行到全局命名空间）。
 *   · 组件长度受 CONFIG_NAME_MAX 与 MAX_MQUEUE_PATH 双重限制：
 *     前缀吃掉预算，超限 -ENAMETOOLONG（如实）。
 *   · 调用者须以 desc.global=true 在**全局**树解析该路径（容器 root
 *     不参与；见 inode_search/_ort_reroot）。
 *
 ****************************************************************************/

int nxmq_ort_fullpath(FAR const char *mq_name, FAR char *fullpath,
                      size_t fullsize)
{
#ifdef CONFIG_ORT_CONTAINER
  FAR struct tcb_s *rtcb;
  char comp[CONFIG_NAME_MAX + 1];
  uint32_t key;
  char kind;
#endif
  FAR const char *name;
  int n;

  if (mq_name == NULL || fullpath == NULL)
    {
      return -EINVAL;
    }

  name = mq_name;
  while (*name == '/')
    {
      name++;
    }

  if (*name == '\0')
    {
      return -EINVAL;
    }

#ifdef CONFIG_ORT_CONTAINER
  rtcb = this_task();
  if (rtcb != NULL && rtcb->group != NULL &&
      (rtcb->group->tg_ort_re_root || rtcb->group->tg_ort_domain != 0))
    {
      /* 容器：先拒绝一切可逃逸形态（单组件；`.`/`..` 会被 VFS 解析掉） */

      if (strchr(name, '/') != NULL || strcmp(name, ".") == 0 ||
          strcmp(name, "..") == 0)
        {
          return -EINVAL;
        }

      if (rtcb->group->tg_ort_nsid != 0)
        {
          key  = rtcb->group->tg_ort_nsid;
          kind = 'n';
        }
      else if (rtcb->group->tg_ort_domain != 0)
        {
          key  = rtcb->group->tg_ort_domain;
          kind = 'd';
        }
      else
        {
          return -EPERM;   /* 容器谓词为真却无实例键 —— fail-closed */
        }

      n = snprintf(comp, sizeof(comp), "ort.%c%u.%s",
                   kind, (unsigned)key, name);
      if (n < 0 || (size_t)n >= sizeof(comp) ||
          (size_t)n > (size_t)CONFIG_NAME_MAX)
        {
          return -ENAMETOOLONG;
        }

      n = snprintf(fullpath, fullsize,
                   CONFIG_FS_MQUEUE_VFS_PATH "/%s", comp);
      if (n < 0 || (size_t)n >= fullsize)
        {
          return -ENAMETOOLONG;
        }

      /* 全部校验通过才打见证（失败的翻译不产生"已生效"假象） */

      _alert("ORT: mqueue ns: %s -> %s\n", name, comp);
      return OK;
    }
#endif

  /* 宿主/非容器：既有语义（仅去前导 '/'） */

  n = snprintf(fullpath, fullsize,
               CONFIG_FS_MQUEUE_VFS_PATH "/%s", name);
  if (n < 0 || (size_t)n >= fullsize)
    {
      return -ENAMETOOLONG;
    }

  return OK;
}

static int file_mq_vopen(FAR struct file *mq, FAR const char *mq_name,
                         int oflags, mode_t umask, va_list ap,
                         FAR int *created)
{
  FAR struct inode *inode;
  FAR struct mqueue_inode_s *msgq;
  FAR struct mq_attr *attr = NULL;
  struct inode_search_s desc;
  char fullpath[MAX_MQUEUE_PATH];
  irqstate_t flags;
  mode_t mode = 0;
  int ret;

  /* Make sure that a non-NULL name is supplied */

  if (!mq || !mq_name || *mq_name == '\0')
    {
      ret = -EINVAL;
      goto errout;
    }

  /* Were we asked to create it? */

  if ((oflags & O_CREAT) != 0)
    {
      /* We have to extract the additional
       * parameters from the variable argument list.
       */

      mode = va_arg(ap, mode_t);
      attr = va_arg(ap, FAR struct mq_attr *);
      if (attr != NULL)
        {
          if (attr->mq_maxmsg <= 0 || attr->mq_msgsize <= 0)
            {
              ret = -EINVAL;
              goto errout;
            }
        }
    }

  mode &= ~umask;

  /* Get the full path to the message queue.  [ORT §111] 容器名字在此
   * 翻译到**实例命名空间**（单组件校验 + 前缀）；宿主保持原语义。 */

  ret = nxmq_ort_fullpath(mq_name, fullpath, sizeof(fullpath));
  if (ret < 0)
    {
      goto errout;
    }

  /* Make sure that the check for the existence of the message queue
   * and the creation of the message queue are atomic with respect to
   * other processes executing mq_open().  A simple sched_lock() would
   * be sufficient for non-SMP case but critical section is needed for
   * SMP case.
   */

  flags = enter_critical_section();

  /* Get the inode for this mqueue.  This should succeed if the message
   * queue has already been created.  In this case, inode_find() will
   * have incremented the reference count on the inode.
   */

  SETUP_SEARCH(&desc, fullpath, false);

  /* [ORT §111] 名字已由 nxmq_ort_fullpath 校验/加实例前缀，路径在
   * **全局**树解析（容器 root 不参与）。§106 的"容器全拒"闸退役 —
   * 隔离改由命名空间承担：容器的名字只会落到自己的前缀下。 */

  desc.global = true;

  ret = inode_find(&desc);
  if (ret >= 0)
    {
      /* Something exists at this path.  Get the search results */

      inode = desc.node;

      /* Verify that the inode is a message queue */

      if (!INODE_IS_MQUEUE(inode))
        {
          ret = -ENXIO;
          goto errout_with_inode;
        }

      /* It exists and is a message queue.  Check if the caller wanted to
       * create a new mqueue with this name.
       */

      if ((oflags & (O_CREAT | O_EXCL)) == (O_CREAT | O_EXCL))
        {
          ret = -EEXIST;
          goto errout_with_inode;
        }

      /* Associate the inode with a file structure */

      mq->f_oflags = oflags;
      mq->f_inode  = inode;

      if (created)
        {
          *created = 0;
        }
    }
  else
    {
      /* The mqueue does not exists.  Were we asked to create it? */

      if ((oflags & O_CREAT) == 0)
        {
          /* The mqueue does not exist and O_CREAT is not set */

          ret = -ENOENT;
          goto errout_with_lock;
        }

      /* Create an inode in the pseudo-filesystem at this path.
       * [ORT §111] 全局树预留（容器前缀路径不经 root 重挂）。 */

      inode_lock();
      ret = inode_reserve_global(fullpath, mode, &inode);
      inode_unlock();

      if (ret < 0)
        {
          goto errout_with_lock;
        }

      /* Allocate memory for the new message queue.  The new inode will
       * be created with a reference count of zero.
       */

      ret = nxmq_alloc_msgq(attr, &msgq);
      if (ret < 0)
        {
          goto errout_with_inode;
        }

      /* Associate the inode with a file structure */

      mq->f_oflags = oflags;
      mq->f_inode  = inode;

      INODE_SET_MQUEUE(inode);
      inode->u.i_ops    = &g_nxmq_fileops;
      inode->i_private  = msgq;
      msgq->inode       = inode;

      /* Set the initial reference count on this inode to one */

      atomic_fetch_add(&inode->i_crefs, 1);

      if (created)
        {
          *created = 1;
        }
    }

  RELEASE_SEARCH(&desc);
  leave_critical_section(flags);
#ifdef CONFIG_FS_NOTIFY
  notify_open(fullpath, oflags);
#endif
  return OK;

errout_with_inode:
  inode_release(inode);

errout_with_lock:
  RELEASE_SEARCH(&desc);
  leave_critical_section(flags);

errout:
  return ret;
}

static mqd_t nxmq_vopen(FAR const char *mq_name, int oflags, va_list ap)
{
  FAR struct file *mq;
  int created;
  int ret;
  int fd;

  mq = file_allocate();
  if (mq == NULL)
    {
      return -ENOMEM;
    }

  ret = file_mq_vopen(mq, mq_name, oflags, getumask(), ap, &created);
  if (ret < 0)
    {
      file_deallocate(mq);
      return ret;
    }

  fd = file_dup(mq, 0, oflags);
  if (fd < 0)
    {
      file_close(mq);
      file_deallocate(mq);
    }

  return fd;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_mq_open
 *
 * Description:
 *   This function establish a connection between a named message queue and
 *   the calling task. This is an internal OS interface.  It is functionally
 *   equivalent to mq_open() except that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 *  See comments with mq_open() for a more complete description of the
 *  behavior of this function
 *
 * Input Parameters:
 *   mq - address of to-be-initialized struct file instance.
 *   mq_name - Name of the queue to open
 *   oflags - open flags
 *   Optional parameters.  When the O_CREAT flag is specified, two optional
 *   parameters are expected:
 *
 *     1. mode_t mode, and
 *     2. struct mq_attr *attr.  The mq_maxmsg attribute
 *        is used at the time that the message queue is
 *        created to determine the maximum number of
 *        messages that may be placed in the message queue.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success, instance pointed by mq is also initialized.
 *   A negated errno value is returned on failure.
 *
 ****************************************************************************/

int file_mq_open(FAR struct file *mq,
                 FAR const char *mq_name, int oflags, ...)
{
  va_list ap;
  int ret;

  memset(mq, 0, sizeof(*mq));

  va_start(ap, oflags);
  ret = file_mq_vopen(mq, mq_name, oflags, 0, ap, NULL);
  va_end(ap);

  return ret;
}

/****************************************************************************
 * Name: nxmq_open
 *
 * Description:
 *   This function establish a connection between a named message queue and
 *   the calling task. This is an internal OS interface.  It is functionally
 *   equivalent to mq_open() except that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 *  See comments with mq_open() for a more complete description of the
 *  behavior of this function
 *
 * Input Parameters:
 *   mq_name - Name of the queue to open
 *   oflags - open flags
 *   Optional parameters.  When the O_CREAT flag is specified, two optional
 *   parameters are expected:
 *
 *     1. mode_t mode, and
 *     2. struct mq_attr *attr.  The mq_maxmsg attribute
 *        is used at the time that the message queue is
 *        created to determine the maximum number of
 *        messages that may be placed in the message queue.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success, mqdes point to the new message queue descriptor.
 *   A negated errno value is returned on failure.
 *
 ****************************************************************************/

mqd_t nxmq_open(FAR const char *mq_name, int oflags, ...)
{
  va_list ap;
  mqd_t ret;

  va_start(ap, oflags);
  ret = nxmq_vopen(mq_name, oflags, ap);
  va_end(ap);

  return ret;
}

/****************************************************************************
 * Name: mq_open
 *
 * Description:
 *   This function establish a connection between a named message queue and
 *   the calling task.  After a successful call of mq_open(), the task can
 *   reference the message queue using the address returned by the call. The
 *   message queue remains usable until it is closed by a successful call to
 *   mq_close().
 *
 * Input Parameters:
 *   mq_name - Name of the queue to open
 *   oflags - open flags
 *   Optional parameters.  When the O_CREAT flag is specified, two optional
 *   parameters are expected:
 *
 *     1. mode_t mode, and
 *     2. struct mq_attr *attr.  The mq_maxmsg attribute
 *        is used at the time that the message queue is
 *        created to determine the maximum number of
 *        messages that may be placed in the message queue.
 *
 * Returned Value:
 *   A message queue descriptor or (mqd_t)-1 (ERROR)
 *
 * Assumptions:
 *
 ****************************************************************************/

mqd_t mq_open(FAR const char *mq_name, int oflags, ...)
{
  va_list ap;
  mqd_t ret;

  va_start(ap, oflags);
  ret = nxmq_vopen(mq_name, oflags, ap);
  va_end(ap);
  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return ret;
}
