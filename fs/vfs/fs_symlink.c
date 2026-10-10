/****************************************************************************
 * fs/vfs/fs_symlink.c
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

#include <sys/types.h>
#include <sys/stat.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/lib/lib.h>
#include <nuttx/fs/fs.h>

#include "inode/inode.h"
#include "fs_heap.h"
#include "vfs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_PSEUDOFS_SOFTLINKS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: symlink
 *
 * Description:
 *   The symlink() function will create a new link (directory entry) for the
 *   existing file, path2.  This implementation is simplified for use with
 *   NuttX in these ways:
 *
 *   - Links may be created only within the NuttX top-level, pseudo file
 *     system.  No file system currently supported by NuttX provides
 *     symbolic links.
 *   - For the same reason, only soft links are implemented.
 *   - File privileges are ignored.
 *   - c_time is not updated.
 *
 * Input Parameters:
 *   path1 - Points to a pathname naming an existing file.
 *   path2 - Points to a pathname naming the new directory entry to be
 *           created.
 *
 * Returned Value:
 *   On success, zero (OK) is returned.  Otherwise, -1 (ERROR) is returned
 *   the errno variable is set appropriately.
 *
 ****************************************************************************/

int symlink(FAR const char *path1, FAR const char *path2)
{
  struct inode_search_s desc;
  FAR struct inode *inode = NULL;
  int errcode;
  int ret;

  if (path1 == NULL)
    {
      errcode = EINVAL;
      goto errout;
    }

  /* Check that no inode exists at the 'path2' and that the path up to
   * 'path2' does not lie on a mounted volume.
   */

  SETUP_SEARCH(&desc, path2, false);

  ret = inode_find(&desc);
  if (ret >= 0)
    {
      /* Something exists at the path2 where we are trying to create the
       * link.
       */

      DEBUGASSERT(desc.node != NULL);

      /* [ORT §95] path2 落在**挂载点内容**里（desc.node = 挂载点 inode，
       * desc.relpath = 卷内相对路径）：路由给该 FS 的 symlink op。
       * （原实现在这里一律 ENOSYS —— "文件系统不支持符号链接"的
       * 上游常态；我们的临时文件系统与 OCI 层解包需要它。） */

#ifndef CONFIG_DISABLE_MOUNTPOINT
      if (INODE_IS_MOUNTPT(desc.node) && desc.relpath[0] != '\0')
        {
          if (desc.node->u.i_mops == NULL ||
              desc.node->u.i_mops->symlink == NULL)
            {
              errcode = ENOSYS;   /* 该 FS 未实现：fail-closed */
              goto errout_with_inode;
            }

          ret = desc.node->u.i_mops->symlink(desc.node, desc.relpath, path1);
          if (ret < 0)
            {
              errcode = -ret;
              goto errout_with_inode;
            }

          /* Symbolic link successfully created */

          inode_release(desc.node);
          RELEASE_SEARCH(&desc);
#ifdef CONFIG_FS_NOTIFY
          notify_create(path2);
#endif
          return OK;
        }
#endif

      /* 已存在节点（伪 FS 节点，或 path2 就是挂载点本身）：EEXIST */

      errcode = EEXIST;
      goto errout_with_inode;
    }

  /* No inode exists that contains this path.  Create a new inode in the
   * pseudo-filesystem at this location.
   */

  else
    {
      /* Copy path1 */

      FAR char *newpath2 = fs_heap_strdup(path1);
      if (newpath2 == NULL)
        {
          errcode = ENOMEM;
          goto errout_with_search;
        }

      /* Create an inode in the pseudo-filesystem at this path.
       * NOTE that the new inode will be created with a reference
       * count of zero.
       */

      inode_lock();
      ret = inode_reserve(path2, 0777, &inode);

      if (ret >= 0)
        {
          /* Initialize the inode */

          INODE_SET_SOFTLINK(inode);
          inode->u.i_link = newpath2;
        }

      inode_unlock();
      if (ret < 0)
        {
          fs_heap_free(newpath2);
          errcode = -ret;
          goto errout_with_search;
        }
    }

  /* Symbolic link successfully created */

  RELEASE_SEARCH(&desc);
#ifdef CONFIG_FS_NOTIFY
  notify_create(path2);
#endif
  return OK;

errout_with_inode:
  inode_release(inode);

errout_with_search:
  RELEASE_SEARCH(&desc);

errout:
  set_errno(errcode);
  return ERROR;
}

#endif /* CONFIG_PSEUDOFS_SOFTLINKS */
