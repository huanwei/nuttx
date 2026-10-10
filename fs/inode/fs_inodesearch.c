/****************************************************************************
 * fs/inode/fs_inodesearch.c
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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <limits.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#include "inode/inode.h"
#include "fs_heap.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int _inode_compare(FAR const char *fname, FAR struct inode *inode);
#ifdef CONFIG_PSEUDOFS_SOFTLINKS
static int _inode_linktarget(FAR struct inode *inode,
                             FAR struct inode_search_s *desc);
#endif
static int _inode_search(FAR struct inode_search_s *desc);
static FAR const char *_inode_getcwd(void);
#ifdef CONFIG_ORT_CONTAINER
static int _ort_reroot(FAR struct inode_search_s *desc, bool fromcwd);
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

FAR struct inode *g_root_inode = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _inode_compare
 *
 * Description:
 *   Compare two inode names
 *
 ****************************************************************************/

static int _inode_compare(FAR const char *fname, FAR struct inode *inode)
{
  FAR char *nname = inode->i_name;

  if (!fname)
    {
      return -1;
    }

  for (; ; )
    {
      /* At the end of the node name? */

      if (!*nname)
        {
          /* Yes.. also at the end of find name? */

          if (!*fname || *fname == '/')
            {
              /* Yes.. return match */

              return 0;
            }
          else
            {
              /* No... return find name > node name */

              return 1;
            }
        }

      /* At end of the find name? */

      else if (!*fname || *fname == '/')
        {
          /* Yes... return find name < node name */

          return -1;
        }

      /* Check for non-matching characters */

      else if (*fname > *nname)
        {
          return 1;
        }
      else if (*fname < *nname)
        {
          return -1;
        }

      /* Not at the end of either string and all of the
       * characters still match.  keep looking.
       */

      else
        {
          fname++;
          nname++;
        }
    }
}

/****************************************************************************
 * Name: _ort_reroot
 *
 * Description:
 *   [ORT §86/§95] 容器重挂（chroot 族）的**唯一实现** —— inode_search
 *   与软链接跟随（_inode_linktarget）共用，保证"跟随走过的路径"和
 *   "直接给的路径"走同一咽喉：
 *
 *     · 本组是重挂容器（tg_ort_re_root）且路径为绝对路径 ⇒ 就地替换
 *       desc 路径为 `<root><path>`；
 *     · fromcwd 为真（PWD 展开来的路径）且已在根内 ⇒ 原样 —— 那是同一
 *       inode 的全局写法；根外则夹回根内（真实 chroot 同款的逃逸角）。
 *     · 其余情况 no-op（设根者/内核线程/非绝对路径）。
 *
 *   返回 0 或负错误码（ENOMEM）。
 *
 ****************************************************************************/

#ifdef CONFIG_ORT_CONTAINER
static int _ort_reroot(FAR struct inode_search_s *desc, bool fromcwd)
{
  FAR struct tcb_s *rtcb = nxsched_self();


  if (rtcb == NULL || rtcb->group == NULL ||
      !rtcb->group->tg_ort_re_root || rtcb->group->tg_ort_root == NULL ||
      desc->path[0] != '/')
    {
      return 0;
    }

  {
    FAR const char *root = rtcb->group->tg_ort_root;
    size_t rlen = strlen(root);
    bool within = (strncmp(desc->path, root, rlen) == 0 &&
                   (desc->path[rlen] == '/' || desc->path[rlen] == '\0'));

    if (!fromcwd || !within)
      {
        FAR char *rp;

        if (fs_heap_asprintf(&rp, "%s%s", root, desc->path) < 0)
          {
            return -ENOMEM;
          }

        if (desc->buffer != NULL)
          {
            fs_heap_free(desc->buffer);
          }

        desc->buffer = rp;
        desc->path   = desc->buffer;
      }
  }

  return 0;
}
#endif

/****************************************************************************
 * Name: _inode_linktarget
 *
 * Description:
 *   If the inode is a soft link, then (1) recursively look-up the inode
 *   referenced by the soft link, and (2) return the inode referenced by
 *   the soft link.
 *
 * Assumptions:
 *   The caller holds the g_inode_sem semaphore
 *
 ****************************************************************************/

#ifdef CONFIG_PSEUDOFS_SOFTLINKS
static int _inode_linktarget(FAR struct inode *inode,
                             FAR struct inode_search_s *desc)
{
  unsigned int count = 0;
  bool save;
  int ret = -ENOENT;

  DEBUGASSERT(desc != NULL && inode != NULL);

  /* An infinite loop is avoided only by the loop count. */

  save = desc->nofollow;
  while (INODE_IS_SOFTLINK(inode))
    {
      FAR const char *link = (FAR const char *)inode->u.i_link;

      /* Reset and reinitialize the search descriptor.  */

      RELEASE_SEARCH(desc);
      SETUP_SEARCH(desc, link, true);

#ifdef CONFIG_ORT_CONTAINER
      /* [ORT §95] 跟随即咽喉：绝对目标的软链接在重挂容器里按**容器根**
       * 解析（chroot 语义）。原实现直进 _inode_search、绕过 §86 的
       * 重挂 —— 潜伏逃逸面（伪 FS 软链接一旦在容器可达处出现即被利用）。
       * 相对目标维持原语义（仅伪 FS 链接，目标按约定为完整路径）。 */

      ret = _ort_reroot(desc, false);
      if (ret < 0)
        {
          break;
        }
#endif

      /* Look up inode associated with the target of the symbolic link */

      ret = _inode_search(desc);
      if (ret < 0)
        {
          break;
        }

      /* Limit the number of symbolic links that we pass through */

      if (++count > SYMLOOP_MAX)
        {
          ret = -ELOOP;
          break;
        }

      /* Set up for the next time through the loop */

      inode = desc->node;
      DEBUGASSERT(inode != NULL);
    }

  desc->nofollow = save;
  return ret;
}
#endif

/****************************************************************************
 * Name: _inode_search
 *
 * Description:
 *   Find the inode associated with 'path' returning the inode references
 *   and references to its companion nodes.  This is the internal, common
 *   implementation of inode_search().
 *
 *   If a mountpoint is encountered in the search prior to encountering the
 *   terminal node, the search will terminate at the mountpoint inode.  That
 *   inode and the relative path from the mountpoint, 'relpath' will be
 *   returned.
 *
 *   If a soft link is encountered that is not the terminal node in the path,
 *   that link WILL be deferenced unconditionally.
 *
 * Assumptions:
 *   The caller holds the g_inode_sem semaphore
 *
 ****************************************************************************/

static int _inode_search(FAR struct inode_search_s *desc)
{
  FAR const char   *name;
  FAR struct inode *inode   = g_root_inode;
  FAR struct inode *left    = NULL;
  FAR struct inode *above   = NULL;
  FAR const char   *relpath = NULL;
  int ret = -ENOENT;

  /* Get the search path, skipping over the leading '/'.  The leading '/' is
   * mandatory because only absolute paths are expected in this context.
   */

  DEBUGASSERT(desc != NULL && desc->path != NULL);
  name  = desc->path;

  if (*name != '/')
    {
      return -EINVAL;
    }

  /* Traverse the pseudo file system node tree until either (1) all nodes
   * have been examined without finding the matching node, or (2) the
   * matching node is found.
   */

  while (inode != NULL)
    {
      int result = _inode_compare(name, inode);

      /* Case 1:  The name is less than the name of the node.
       * Since the names are ordered, these means that there
       * is no peer node with this name and that there can be
       * no match in the filesystem.
       */

      if (result < 0)
        {
          inode = NULL;
          break;
        }

      /* Case 2: the name is greater than the name of the node.
       * In this case, the name may still be in the list to the
       * "right"
       */

      else if (result > 0)
        {
          /* Continue looking to the "right" of this inode. */

          left  = inode;
          inode = inode->i_peer;
        }

      /* The names match */

      else
        {
          /* Now there are three remaining possibilities:
           *   (1) This is the node that we are looking for.
           *   (2) The node we are looking for is "below" this one.
           *   (3) This node is a mountpoint and will absorb all requests
           *       below this one
           */

          name = inode_nextname(name);
          if (*name == '\0' || INODE_IS_MOUNTPT(inode))
            {
              /* Either (1) we are at the end of the path, so this must be
               * the node we are looking for or else (2) this node is a
               * mountpoint and will handle the remaining part of the
               * pathname
               */

              relpath = name;
              ret = OK;
              break;
            }
          else
            {
              /* More nodes to be examined in the path "below" this one. */

#ifdef CONFIG_PSEUDOFS_SOFTLINKS
              /* Was the node a soft link?  If so, then we need need to
               * continue below the target of the link, not the link itself.
               */

              if (INODE_IS_SOFTLINK(inode))
                {
                  int status;

                  /* If this intermediate inode in the is a soft link, then
                   * (1) recursively look-up the inode referenced by the
                   * soft link, and (2) continue searching with that inode
                   * instead.
                   */

                  status = _inode_linktarget(inode, desc);
                  if (status < 0)
                    {
                      /* Probably means that the target of the symbolic link
                       * does not exist.
                       */

                      ret = status;
                      break;
                    }
                  else
                    {
                      FAR struct inode *newnode = desc->node;

                      if (newnode != inode)
                        {
                          /* The node was a valid symbolic link and we have
                           * jumped to a different, spot in the pseudo file
                           * system tree.
                           */

                          /* Check if this took us to a mountpoint. */

                          if (INODE_IS_MOUNTPT(newnode))
                            {
                              /* Return the mountpoint information.
                               * NOTE that the last path to the link target
                               * was already set by _inode_linktarget().
                               */

                              inode   = newnode;
                              above   = desc->parent;
                              left    = desc->peer;
                              ret     = OK;

                              if (*desc->relpath != '\0')
                                {
                                  FAR char *buffer = NULL;

                                  ret = fs_heap_asprintf(&buffer, "%s/%s",
                                                         desc->relpath,
                                                         name);
                                  if (ret > 0)
                                    {
                                      fs_heap_free(desc->buffer);
                                      desc->buffer = buffer;
                                      relpath = buffer;
                                      ret = OK;
                                    }
                                  else
                                    {
                                      ret = -ENOMEM;
                                    }
                                }
                              else
                                {
                                  relpath = name;
                                }

                              break;
                            }

                          /* Continue from this new inode. */

                          inode = newnode;
                        }
                    }
                }
#endif

              /* Keep looking at the next level "down" */

              above = inode;
              left  = NULL;
              inode = inode->i_child;
            }
        }
    }

  /* The node may or may not be null as per one of the following four cases:
   *
   * With node = NULL
   *
   *   (1) We went left past the final peer:  The new node name is larger
   *       than any existing node name at that level.
   *   (2) We broke out in the middle of the list of peers because the name
   *       was not found in the ordered list.
   *   (3) We went down past the final parent:  The new node name is
   *       "deeper" than anything that we currently have in the tree.
   *
   * With node != NULL
   *
   *   (4) When the node matching the full path is found
   */

  desc->path    = name;
  desc->node    = inode;
  desc->peer    = left;
  desc->parent  = above;
  desc->relpath = relpath;
  return ret;
}

/****************************************************************************
 * Name: _inode_getcwd
 *
 * Description:
 *   Return the current working directory
 *
 ****************************************************************************/

static FAR const char *_inode_getcwd(void)
{
  FAR const char *pwd = "";

#ifndef CONFIG_DISABLE_ENVIRON
  pwd = getenv("PWD");
  if (pwd == NULL)
    {
      pwd = CONFIG_LIBC_HOMEDIR;
    }
#endif

  return pwd;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inode_search
 *
 * Description:
 *   Find the inode associated with 'path' returning the inode references
 *   and references to its companion nodes.
 *
 *   If a mountpoint is encountered in the search prior to encountering the
 *   terminal node, the search will terminate at the mountpoint inode.  That
 *   inode and the relative path from the mountpoint, 'relpath' will be
 *   returned.
 *
 *   inode_search will follow soft links in path leading up to the terminal
 *   node.  Whether or no inode_search() will deference that terminal node
 *   depends on the 'nofollow' input.
 *
 *   If a soft link is encountered that is not the terminal node in the path,
 *   that link WILL be deferenced unconditionally.
 *
 * Assumptions:
 *   The caller holds the g_inode_sem semaphore
 *
 ****************************************************************************/

int inode_search(FAR struct inode_search_s *desc)
{
  int ret;
  bool fromcwd = false;

  /* Perform the common _inode_search() logic.  This does everything except
   * operations special operations that must be performed on the terminal
   * node if node is a symbolic link.
   */

  DEBUGASSERT(desc != NULL && desc->path != NULL);

  /* Convert the relative path to the absolute path */

  if (*desc->path != '/')
    {
      ret = fs_heap_asprintf(&desc->buffer, "%s/%s",
                             _inode_getcwd(), desc->path);
      if (ret < 0)
        {
          return -ENOMEM;
        }

      desc->path = desc->buffer;
      fromcwd    = true;
    }

  /* [ORT §96①] 主循环：咽喉重挂 + 链接跟随**重驱动**。
   *
   * 流程：重挂（容器）→ _inode_search → 伪 FS 软链接终端跟随 → 若停在
   * 挂载点且 relpath 非空，问该 FS 的 resolve op：路径里有没有符号
   * 链接要展开？有就换路径从**咽喉**重新走（跨挂载/容器语义都在咽喉
   * 统一处理），直到 FS 报"没有链接"（0）或出错；链长以 SYMLOOP 为界
   * （-ELOOP）。
   *
   * 重驱动的两种形态：
   *   · 替换路径 `/` 开头（目标是绝对）⇒ 按容器根重挂（chroot 语义，
   *     与伪 FS 软链接跟随的 _ort_reroot(desc, false) 一致）；
   *   · 挂载内相对（相对目标拼接后仍在本挂载）⇒ 与挂载点全局路径拼
   *     接，**不再重挂**（拼出来已是全局视图路径）。
   *
   * 静态缓冲：本函数持 g_inode_sem 全局串行（既有假设），resolve 实现
   * 也不回调 VFS —— 静态安全。
   */

  {
    static char rbuf[300];
    static char mpath[300];
    int  loops;
    bool reroot = true;

    for (loops = 0; ; loops++)
      {
        if (loops > SYMLOOP_MAX)
          {
            ret = -ELOOP;
            break;
          }

#ifdef CONFIG_ORT_CONTAINER
        if (reroot)
          {
            ret = _ort_reroot(desc, loops == 0 ? fromcwd : false);
            if (ret < 0)
              {
                return ret;
              }
          }
#endif

        ret = _inode_search(desc);
        if (ret < 0)
          {
            break;
          }

#ifdef CONFIG_PSEUDOFS_SOFTLINKS
        /* 终端的伪 FS 软链接：跟随（原逻辑；自带 SYMLOOP 界） */

        if (!desc->nofollow && INODE_IS_SOFTLINK(desc->node))
          {
            ret = _inode_linktarget(desc->node, desc);
            if (ret < 0)
              {
                break;
              }
          }
#endif

        /* 停在挂载点、卷内有剩余路径：问 FS 要不要展开链接 */

        if (!INODE_IS_MOUNTPT(desc->node) ||
            desc->node->u.i_mops == NULL ||
            desc->node->u.i_mops->resolve == NULL ||
            desc->relpath == NULL || desc->relpath[0] == '\0')
          {
            break;
          }

        ret = desc->node->u.i_mops->resolve(desc->node, desc->relpath,
                                            desc->nofollow, rbuf,
                                            sizeof(rbuf));
        if (ret <= 0)
          {
            break;                     /* 0 = 无链接；负 = 错误 */
          }

        /* ret == 1：替换路径重驱动 */

        {
          FAR char *np;

          if (rbuf[0] == '/')
            {
              np = fs_heap_strdup(rbuf);
              reroot = true;
            }
          else
            {
              if (inode_getpath(desc->node, mpath, sizeof(mpath)) < 0)
                {
                  ret = -ENAMETOOLONG;
                  break;
                }

              if (fs_heap_asprintf(&np, "%s/%s", mpath, rbuf) < 0)
                {
                  np = NULL;
                }

              reroot = false;
            }

          if (np == NULL)
            {
              ret = -ENOMEM;
              break;
            }

          if (desc->buffer != NULL)
            {
              fs_heap_free(desc->buffer);
            }

          desc->buffer = np;
          desc->path   = np;
        }
      }
  }

  return ret;
}

/****************************************************************************
 * Name: inode_nextname
 *
 * Description:
 *   Given a path with node names separated by '/', return the next path
 *   segment name.
 *
 ****************************************************************************/

FAR const char *inode_nextname(FAR const char *name)
{
  /* Search for the '/' delimiter or the NUL terminator at the end of the
   * path segment.
   */

  while (*name != '\0' && *name != '/')
    {
      name++;
    }

  /* If we found the '/' delimiter, then the path segment we want begins at
   * the next character (which might also be the NUL terminator).
   */

  while (*name == '/')
    {
      name++;
    }

  /* Skip single '.' path segment, but not '..' */

  if (*name == '.' && *(name + 1) == '/')
    {
      /* If there is a '/' after '.',
       * continue searching from the next character
       */

      name = inode_nextname(name);
    }

  return name;
}
