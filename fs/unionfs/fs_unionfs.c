/****************************************************************************
 * fs/unionfs/fs_unionfs.c
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

#include <sys/param.h>
#include <sys/types.h>
#include <sys/statfs.h>
#include <sys/stat.h>
#include <sys/mount.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <fixedmath.h>
#include <nuttx/debug.h>

#include <nuttx/lib/lib.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mutex.h>

#include "inode/inode.h"
#include "fs_heap.h"

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && defined(CONFIG_FS_UNIONFS)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct unionfs_dir_s
{
  struct fs_dirent_s fu_base;          /* Vfs directory structure */
  uint8_t fu_ndx;                      /* Index of file system being enumerated */
  bool fu_eod;                         /* True: At end of directory */
  bool fu_prefix[2];                   /* True: Fake directory in prefix */
  FAR char *fu_relpath;                /* Path being enumerated */
  FAR struct fs_dirent_s *fu_lower[2]; /* dirent struct used by contained file system */
};

/* This structure describes one contained file system mountpoint */

struct unionfs_mountpt_s
{
  FAR struct inode *um_node;         /* Filesystem inode */
  FAR char *um_prefix;               /* Path prefix to filesystem */
};

/* This structure describes the union file system */

struct unionfs_inode_s
{
  struct unionfs_mountpt_s ui_fs[2]; /* Contained file systems */
  mutex_t ui_lock;                   /* Enforces mutually exclusive access */
  int16_t ui_nopen;                  /* Number of open references */
  bool ui_unmounted;                 /* File system has been unmounted */
};

/* This structure describes one opened file */

struct unionfs_file_s
{
  uint8_t uf_ndx;                   /* Filesystem index */
  FAR struct file uf_file;          /* Filesystem open file description */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helper functions */

static FAR const char *unionfs_offsetpath(FAR const char *relpath,
                                          FAR const char *prefix);
static bool    unionfs_ispartprefix(FAR const char *partprefix,
                                    FAR const char *prefix);
static int     unionfs_tryopen(FAR struct file *filep,
                               FAR const char *relpath,
                               FAR const char *prefix, int oflags,
                               mode_t mode);
static int     unionfs_tryopendir(FAR struct inode *inode,
                                  FAR const char *relpath,
                                  FAR const char *prefix,
                                  FAR struct fs_dirent_s **dir);
static int     unionfs_trymkdir(FAR struct inode *inode,
                                FAR const char *relpath,
                                FAR const char *prefix,
                                mode_t mode);
static int     unionfs_tryrmdir(FAR struct inode *inode,
                                FAR const char *relpath,
                                FAR const char *prefix);
static int     unionfs_tryunlink(FAR struct inode *inode,
                                 FAR const char *relpath,
                                 FAR const char *prefix);
static int     unionfs_tryrename(FAR struct inode *mountpt,
                                 FAR const char *oldrelpath,
                                 FAR const char *newrelpath,
                                 FAR const char *prefix);
static int     unionfs_trystat(FAR struct inode *inode,
                               FAR const char *relpath,
                               FAR const char *prefix,
                               FAR struct stat *buf);
static int     unionfs_trychstat(FAR struct inode *inode,
                                 FAR const char *relpath,
                                 FAR const char *prefix,
                                 FAR const struct stat *buf, int flags);
static int     unionfs_trystatdir(FAR struct inode *inode,
                                  FAR const char *relpath,
                                  FAR const char *prefix);
static int     unionfs_trystatfile(FAR struct inode *inode,
                                   FAR const char *relpath,
                                   FAR const char *prefix);
static FAR char *unionfs_relpath(FAR const char *path,
                                 FAR const char *name);

static int     unionfs_unbind_child(FAR struct unionfs_mountpt_s *um);
static void    unionfs_destroy(FAR struct unionfs_inode_s *ui);

/* [ORT §85] whiteout 视图语义（mkparents 先用后定义） */

static bool    unionfs_lowerhidden(FAR struct unionfs_inode_s *ui,
                                   FAR const char *relpath);

/* Operations on opened files (with struct file) */

static int     unionfs_open(FAR struct file *filep, FAR const char *relpath,
                            int oflags, mode_t mode);
static int     unionfs_close(FAR struct file *filep);
static ssize_t unionfs_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t unionfs_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static off_t   unionfs_seek(FAR struct file *filep, off_t offset,
                            int whence);
static int     unionfs_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg);
static int     unionfs_sync(FAR struct file *filep);
static int     unionfs_dup(FAR const struct file *oldp,
                           FAR struct file *newp);
static int     unionfs_fstat(FAR const struct file *filep,
                             FAR struct stat *buf);
static int     unionfs_fchstat(FAR const struct file *filep,
                               FAR const struct stat *buf, int flags);
static int     unionfs_truncate(FAR struct file *filep, off_t length);

/* Operations on directories */

static int     unionfs_opendir(FAR struct inode *mountpt,
                               FAR const char *relpath,
                               FAR struct fs_dirent_s **dir);
static int     unionfs_closedir(FAR struct inode *mountpt,
                                FAR struct fs_dirent_s *dir);
static int     unionfs_readdir(FAR struct inode *mountpt,
                               FAR struct fs_dirent_s *dir,
                               FAR struct dirent *entry);
static int     unionfs_rewinddir(FAR struct inode *mountpt,
                                 FAR struct fs_dirent_s *dir);

static int     unionfs_bind(FAR struct inode *blkdriver,
                            FAR const void *data, FAR void **handle);
static int     unionfs_unbind(FAR void *handle, FAR struct inode **blkdriver,
                              unsigned int flags);
static int     unionfs_statfs(FAR struct inode *mountpt,
                              FAR struct statfs *buf);

  /* Operations on paths */

static int     unionfs_unlink(FAR struct inode *mountpt,
                              FAR const char *relpath);
static int     unionfs_mkdir(FAR struct inode *mountpt,
                             FAR const char *relpath, mode_t mode);
static int     unionfs_rmdir(FAR struct inode *mountpt,
                             FAR const char *relpath);
static int     unionfs_rename(FAR struct inode *mountpt,
                              FAR const char *oldrelpath,
                              FAR const char *newrelpath);
static int     unionfs_stat(FAR struct inode *mountpt,
                            FAR const char *relpath, FAR struct stat *buf);
static int     unionfs_chstat(FAR struct inode *mountpt,
                              FAR const char *relpath,
                              FAR const struct stat *buf, int flags);

/* Initialization */

static int     unionfs_getmount(FAR const char *path,
                                FAR struct inode **inode);
static int     unionfs_dobind(FAR const char *fspath1,
                              FAR const char *prefix1,
                              FAR const char *fspath2,
                              FAR const char *prefix2,
                              FAR void **handle);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* See fs_mount.c -- this structure is explicitly extern'ed there.
 * We use the old-fashioned kind of initializers so that this will compile
 * with any compiler.
 */

const struct mountpt_operations g_unionfs_operations =
{
  unionfs_open,        /* open */
  unionfs_close,       /* close */
  unionfs_read,        /* read */
  unionfs_write,       /* write */
  unionfs_seek,        /* seek */
  unionfs_ioctl,       /* ioctl */
  NULL,                /* mmap */
  unionfs_truncate,    /* truncate */
  NULL,                /* poll */
  NULL,                /* readv */
  NULL,                /* writev */

  unionfs_sync,        /* sync */
  unionfs_dup,         /* dup */
  unionfs_fstat,       /* fstat */
  unionfs_fchstat,     /* fchstat */

  unionfs_opendir,     /* opendir */
  unionfs_closedir,    /* closedir */
  unionfs_readdir,     /* readdir */
  unionfs_rewinddir,   /* rewinddir */

  unionfs_bind,        /* bind */
  unionfs_unbind,      /* unbind */
  unionfs_statfs,      /* statfs */

  unionfs_unlink,      /* unlink */
  unionfs_mkdir,       /* mkdir */
  unionfs_rmdir,       /* rmdir */
  unionfs_rename,      /* rename */
  unionfs_stat,        /* stat */
  unionfs_chstat       /* chstat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: unionfs_offsetpath
 ****************************************************************************/

static FAR const char *unionfs_offsetpath(FAR const char *relpath,
                                          FAR const char *prefix)
{
  FAR const char *trypath;
  int pfxlen;

  /* Is there a prefix on the path to this file system? */

  if (prefix && (pfxlen = strlen(prefix)) > 0)
    {
      /* Does the prefix match? */

      if (strncmp(prefix, relpath, pfxlen) != 0)
        {
          /* No, then this relative cannot be within this file system */

          return NULL;
        }

      /* Skip over the prefix */

      trypath = relpath + pfxlen;

      /* Make sure that what is left is a valid, relative path */

      for (; *trypath == '/'; trypath++);
    }
  else
    {
      /* No.. use the full, relative path */

      trypath = relpath;
    }

  return trypath;
}

/****************************************************************************
 * Name: unionfs_ispartprefix
 ****************************************************************************/

static bool unionfs_ispartprefix(FAR const char *partprefix,
                                 FAR const char *prefix)
{
  int partlen = 0;
  int pfxlen  = 0;

  /* Trim any '/' characters in the partial prefix */

  if (partprefix != NULL)
    {
      /* Skip over any leading '/' */

      for (; *partprefix == '/'; partprefix++);

      /* Skip over any tailing '/' */

      partlen = strlen(partprefix);
      for (; partlen > 1 && partprefix[partlen - 1]  == '/'; partlen--);
    }

  /* Check for NUL or empty partial prefix */

  if (partprefix == NULL || *partprefix == '\0')
    {
      /* A NUL partial prefix is always contained in the full prefix, even
       * if there is no prefix.
       */

      return true;
    }

  /* Trim any '/' characters in the partial prefix */

  if (prefix != NULL)
    {
      /* Skip over any leading '/' */

      for (; *prefix == '/'; prefix++);

      /* Skip over any tailing '/' */

      pfxlen = strlen(prefix);
      for (; pfxlen > 1 && prefix[pfxlen - 1]  == '/'; pfxlen--);
    }

  /* Check for NUL or empty full prefix */

  if (prefix == NULL || *prefix == '\0')
    {
      /* A non-NUL partial path cannot be a contained in a NUL prefix */

      return false;
    }

#if 0 /* Only whole offset is currently supported */
  /* Both the partial path and the prefix are non-NULL.  Check if the partial
   * path is contained in the prefix.
   */

  if (partlen > pfxlen)
    {
      return false;
    }
#else
  /* Check if the trimmed offsets are identical */

  if (partlen != pfxlen)
    {
      return false;
    }
#endif

  if (strncmp(partprefix, prefix, partlen) == 0)
    {
      return true;
    }
  else
    {
      return false;
    }
}

/****************************************************************************
 * Name: unionfs_tryopen
 ****************************************************************************/

static int unionfs_tryopen(FAR struct file *filep, FAR const char *relpath,
                           FAR const char *prefix, int oflags, mode_t mode)
{
  FAR const struct mountpt_operations *ops;
  FAR const char *trypath;

  /* Is this path valid on this file system? */

  trypath = unionfs_offsetpath(relpath, prefix);
  if (trypath == NULL)
    {
      /* No.. return -ENOENT */

      return -ENOENT;
    }

  /* Yes.. try to open this directory */

  DEBUGASSERT(filep->f_inode->u.i_mops != NULL);
  ops = filep->f_inode->u.i_mops;

  if (!ops->open)
    {
      return -ENOSYS;
    }

  return ops->open(filep, trypath, oflags, mode);
}

/****************************************************************************
 * Name: unionfs_tryopendir
 ****************************************************************************/

static int unionfs_tryopendir(FAR struct inode *inode,
                              FAR const char *relpath,
                              FAR const char *prefix,
                              FAR struct fs_dirent_s **dir)
{
  FAR const struct mountpt_operations *ops;
  FAR const char *trypath;

  /* Is this path valid on this file system? */

  trypath = unionfs_offsetpath(relpath, prefix);
  if (trypath == NULL)
    {
      /* No.. return -ENOENT */

      return -ENOENT;
    }

  /* Yes.. Try to open this directory */

  ops = inode->u.i_mops;
  DEBUGASSERT(ops && ops->opendir);

  if (!ops->opendir)
    {
      return -ENOSYS;
    }

  return ops->opendir(inode, trypath, dir);
}

/****************************************************************************
 * Name: unionfs_trymkdir
 ****************************************************************************/

static int unionfs_trymkdir(FAR struct inode *inode, FAR const char *relpath,
                            FAR const char *prefix, mode_t mode)
{
  FAR const struct mountpt_operations *ops;
  FAR const char *trypath;

  /* Is this path valid on this file system? */

  trypath = unionfs_offsetpath(relpath, prefix);
  if (trypath == NULL)
    {
      /* No.. return -ENOENT */

      return -ENOENT;
    }

  /* Yes.. Try to create the directory */

  ops = inode->u.i_mops;
  if (!ops->mkdir)
    {
      return -ENOSYS;
    }

  return ops->mkdir(inode, trypath, mode);
}

/****************************************************************************
 * Name: unionfs_tryrename
 ****************************************************************************/

static int unionfs_tryrename(FAR struct inode *mountpt,
                             FAR const char *oldrelpath,
                             FAR const char *newrelpath,
                             FAR const char *prefix)
{
  FAR const struct mountpt_operations *ops;
  FAR const char *tryoldpath;
  FAR const char *trynewpath;

  /* Is source path valid on this file system? */

  tryoldpath = unionfs_offsetpath(oldrelpath, prefix);
  if (tryoldpath == NULL)
    {
      /* No.. return -ENOENT.  This should not fail because the existence
       * of the file has already been verified.
       */

      return -ENOENT;
    }

  /* Is source path valid on this file system?
   * REVISIT:  There is no logic currently to rename the file by copying i
   * to a different file system.  So we just fail if the destination name
   * is not within the same file system.  I might, however, be on the other
   * file system and that rename should be supported as a file copy and
   * delete.
   */

  trynewpath = unionfs_offsetpath(newrelpath, prefix);
  if (trynewpath == NULL)
    {
      /* No.. return -ENOSYS.  We should be able to do this, but we can't
       * yet.
       */

      return -ENOSYS;
    }

  /* Yes.. Try to rename the file */

  ops = mountpt->u.i_mops;
  if (!ops->rename)
    {
      return -ENOSYS;
    }

  return ops->rename(mountpt, tryoldpath, trynewpath);
}

/****************************************************************************
 * Name: unionfs_trystat
 ****************************************************************************/

static int unionfs_trystat(FAR struct inode *inode, FAR const char *relpath,
                           FAR const char *prefix, FAR struct stat *buf)
{
  FAR const struct mountpt_operations *ops;
  FAR const char *trypath;

  /* Is this path valid on this file system? */

  trypath = unionfs_offsetpath(relpath, prefix);
  if (trypath == NULL)
    {
      /* No.. return -ENOENT */

      return -ENOENT;
    }

  /* Yes.. Try to create the directory */

  ops = inode->u.i_mops;
  if (!ops->stat)
    {
      return -ENOSYS;
    }

  return ops->stat(inode, trypath, buf);
}

/****************************************************************************
 * Name: unionfs_trychstat
 ****************************************************************************/

static int unionfs_trychstat(FAR struct inode *inode,
                             FAR const char *relpath, FAR const char *prefix,
                             FAR const struct stat *buf, int flags)
{
  FAR const struct mountpt_operations *ops;
  FAR const char *trypath;

  /* Is this path valid on this file system? */

  trypath = unionfs_offsetpath(relpath, prefix);
  if (trypath == NULL)
    {
      /* No.. return -ENOENT */

      return -ENOENT;
    }

  /* Yes.. Try to change the status */

  ops = inode->u.i_mops;
  if (!ops->chstat)
    {
      return -ENOSYS;
    }

  return ops->chstat(inode, trypath, buf, flags);
}

/****************************************************************************
 * Name: unionfs_trystatdir
 ****************************************************************************/

static int unionfs_trystatdir(FAR struct inode *inode,
                              FAR const char *relpath,
                              FAR const char *prefix)
{
  struct stat buf;
  int ret;

  /* Check if relative path refers to a directory. */

  ret = unionfs_trystat(inode, relpath, prefix, &buf);
  if (ret >= 0 && !S_ISDIR(buf.st_mode))
    {
      return -ENOTDIR;
    }

  return ret;
}

/****************************************************************************
 * Name: unionfs_trystatfile
 ****************************************************************************/

static int unionfs_trystatfile(FAR struct inode *inode,
                               FAR const char *relpath,
                               FAR const char *prefix)
{
  struct stat buf;
  int ret;

  /* Check if relative path refers to a regular file.  We specifically
   * exclude directories but neither do we expect any kind of special file
   * to reside on the mounted filesystem.
   */

  ret = unionfs_trystat(inode, relpath, prefix, &buf);
  if (ret >= 0 && !S_ISREG(buf.st_mode))
    {
      return -EISDIR;
    }

  return ret;
}

/****************************************************************************
 * Name: unionfs_tryrmdir
 ****************************************************************************/

static int unionfs_tryrmdir(FAR struct inode *inode, FAR const char *relpath,
                            FAR const char *prefix)
{
  FAR const struct mountpt_operations *ops;
  FAR const char *trypath;

  /* Is this path valid on this file system? */

  trypath = unionfs_offsetpath(relpath, prefix);
  if (trypath == NULL)
    {
      /* No.. return -ENOENT */

      return -ENOENT;
    }

  /* Yes.. Try to remove the directory */

  ops = inode->u.i_mops;
  if (!ops->rmdir)
    {
      return -ENOSYS;
    }

  return ops->rmdir(inode, trypath);
}

/****************************************************************************
 * Name: unionfs_tryunlink
 ****************************************************************************/

static int unionfs_tryunlink(FAR struct inode *inode,
                             FAR const char *relpath,
                             FAR const char *prefix)
{
  FAR const struct mountpt_operations *ops;
  FAR const char *trypath;

  /* Is this path valid on this file system? */

  trypath = unionfs_offsetpath(relpath, prefix);
  if (trypath == NULL)
    {
      /* No.. return -ENOENT */

      return -ENOENT;
    }

  /* Yes.. Try to unlink the file */

  ops = inode->u.i_mops;
  if (!ops->unlink)
    {
      return -ENOSYS;
    }

  return ops->unlink(inode, trypath);
}

/****************************************************************************
 * Name: unionfs_relpath
 ****************************************************************************/

static FAR char *unionfs_relpath(FAR const char *path, FAR const char *name)
{
  FAR char *relpath;
  int pathlen;
  int ret;

  /* Check if there is a valid, non-zero-legnth path */

  if (path && (pathlen = strlen(path)) > 0)
    {
      /* Yes.. extend the file name by prepending the path */

      if (path[pathlen - 1] == '/')
        {
          ret = fs_heap_asprintf(&relpath, "%s%s", path, name);
        }
      else
        {
          ret = fs_heap_asprintf(&relpath, "%s/%s", path, name);
        }

      /* Handle errors */

      if (ret < 0)
        {
          return NULL;
        }
      else
        {
          return relpath;
        }
    }
  else
    {
      /* There is no path... just duplicate the name (so that fs_heap_free()
       * will work later).
       */

      return fs_heap_strdup(name);
    }
}

/****************************************************************************
 * Name: unionfs_unbind_child
 ****************************************************************************/

static int unionfs_unbind_child(FAR struct unionfs_mountpt_s *um)
{
  FAR struct inode *mpinode = um->um_node;
  FAR struct inode *bdinode = NULL;
  int ret;

  /* Unbind the block driver from the file system (destroying any fs
   * private data).  This logic is essentially the same as the logic in
   * nuttx/fs/mount/fs_umount2.c.
   */

  if (!mpinode->u.i_mops->unbind)
    {
      /* The filesystem does not support the unbind operation ??? */

      return -EINVAL;
    }

  /* The unbind method returns the number of references to the file system
   * (i.e., open files), zero if the unbind was performed, or a negated
   * error code on a failure.
   */

  ret = mpinode->u.i_mops->unbind(mpinode->i_private, &bdinode, MNT_FORCE);
  if (ret < 0)
    {
      /* Some failure occurred */

      return ret;
    }
  else if (ret > 0)
    {
      /* REVISIT: This is bad if the file system cannot support a deferred
       * unmount.  Ideally it would perform the unmount when the last file
       * is closed.  But I don't think any file system do that.
       */

      return -EBUSY;
    }

  /* Successfully unbound */

  mpinode->i_private = NULL;

  /* Release the mountpoint inode and any block driver inode
   * returned by the file system unbind above.  This should cause
   * the inode to be deleted (unless there are other references)
   */

  inode_release(mpinode);

  /* Did the unbind method return a contained block driver */

  if (bdinode)
    {
      inode_release(bdinode);
    }

  return OK;
}

/****************************************************************************
 * Name: unionfs_destroy
 ****************************************************************************/

static void unionfs_destroy(FAR struct unionfs_inode_s *ui)
{
  DEBUGASSERT(ui != NULL && ui->ui_fs[0].um_node != NULL &&
              ui->ui_fs[1].um_node != NULL && ui->ui_nopen == 0);

  /* Unbind the contained file systems */

  unionfs_unbind_child(&ui->ui_fs[0]);
  unionfs_unbind_child(&ui->ui_fs[1]);

  /* Free any allocated prefix strings */

  if (ui->ui_fs[0].um_prefix)
    {
      fs_heap_free(ui->ui_fs[0].um_prefix);
    }

  if (ui->ui_fs[1].um_prefix)
    {
      fs_heap_free(ui->ui_fs[1].um_prefix);
    }

  /* And finally free the allocated unionfs state structure as well */

  nxmutex_destroy(&ui->ui_lock);
  fs_heap_free(ui);
}

/* [ORT §85] §84 的 unionfs_deny_lower（下层目标一律拒绝）已随 whiteout
 * 落地移除：unlink/rmdir/rename/chstat 的"仅下层"路径现已实现真实语义
 * （白障 / copy-up），不再有拒绝分支。 */


/****************************************************************************
 * Name: unionfs_mkparents
 *
 * Description:
 *   [ORT §84] copy-up 的父目录上移：为 relpath 的每一级祖先目录在上层
 *   补齐 —— 祖先在下层存在（视图可见）而上层缺失时 mkdir 上层；两处都
 *   没有（或下层同名为非目录）则返回相应错误（上层单方面造目录不是
 *   open/mkdir 的语义）。
 *
 ****************************************************************************/

static int unionfs_mkparents(FAR struct unionfs_inode_s *ui,
                             FAR const char *relpath)
{
  FAR struct unionfs_mountpt_s *umu = &ui->ui_fs[0];
  FAR struct unionfs_mountpt_s *uml = &ui->ui_fs[1];
  FAR char *path;
  FAR char *slash;
  int ret = OK;

  path = fs_heap_strdup(relpath);
  if (path == NULL)
    {
      return -ENOMEM;
    }

  for (slash = strchr(path + 1, '/'); slash != NULL;
       slash = strchr(slash + 1, '/'))
    {
      *slash = '\0';

      /* 上层已有该目录？ */

      ret = unionfs_trystatdir(umu->um_node, path, umu->um_prefix);
      if (ret == -ENOENT)
        {
          /* 没有 —— 视图里必须（经下层）存在才允许上移 */

          ret = unionfs_trystatdir(uml->um_node, path, uml->um_prefix);
          if (ret >= 0)
            {
              if (unionfs_lowerhidden(ui, path))
                {
                  /* [ORT §85] 祖先本身被视图屏蔽：不补齐、报不存在 */

                  ret = -ENOENT;
                }
              else
                {
                  ret = unionfs_trymkdir(umu->um_node, path, umu->um_prefix,
                                         0777);
                }
            }
        }

      *slash = '/';

      if (ret < 0)
        {
          break;
        }
    }

  fs_heap_free(path);
  return ret;
}

/****************************************************************************
 * Name: unionfs_copyup
 *
 * Description:
 *   [ORT §84] 写前上移：把下层既有常规文件整份复制到上层（下层 O_RDONLY
 *   → 上层 O_WRONLY|O_CREAT|O_TRUNC），模式位尽力跟随；成功后打内核
 *   见证行。调用者须先 unionfs_mkparents() 补齐上层父目录。
 *
 ****************************************************************************/

#define UNIONFS_COPYBUFSZ 1024

static int unionfs_copyup(FAR struct unionfs_inode_s *ui,
                          FAR const char *relpath)
{
  FAR struct unionfs_mountpt_s *umu = &ui->ui_fs[0];
  FAR struct unionfs_mountpt_s *uml = &ui->ui_fs[1];
  FAR const struct mountpt_operations *lops = uml->um_node->u.i_mops;
  FAR const struct mountpt_operations *uops = umu->um_node->u.i_mops;
  FAR const char *upath;
  FAR const char *lpath;
  FAR char *buf;
  struct file lfile;
  struct file ufile;
  struct stat st;
  off_t total = 0;
  off_t off;
  int ret;

  lpath = unionfs_offsetpath(relpath, uml->um_prefix);
  upath = unionfs_offsetpath(relpath, umu->um_prefix);
  if (lpath == NULL || upath == NULL)
    {
      return -ENOENT;
    }

  if (lops->open == NULL || lops->read == NULL || lops->close == NULL ||
      uops->open == NULL || uops->write == NULL || uops->close == NULL)
    {
      return -ENOSYS;
    }

  buf = fs_heap_malloc(UNIONFS_COPYBUFSZ);
  if (buf == NULL)
    {
      return -ENOMEM;
    }

  /* 下层只读打开（fstat 顺取模式位） */

  memset(&lfile, 0, sizeof(lfile));
  lfile.f_oflags = O_RDONLY;
  lfile.f_inode  = uml->um_node;

  ret = lops->open(&lfile, lpath, O_RDONLY, 0);
  if (ret < 0)
    {
      goto errout_with_buf;
    }

  memset(&st, 0, sizeof(st));
  if (lops->fstat != NULL)
    {
      lops->fstat(&lfile, &st);
    }

  /* 上层创建 + 整份复制 */

  memset(&ufile, 0, sizeof(ufile));
  ufile.f_oflags = O_WRONLY | O_CREAT | O_TRUNC;
  ufile.f_inode  = umu->um_node;

  ret = uops->open(&ufile, upath, O_WRONLY | O_CREAT | O_TRUNC,
                   (st.st_mode & 0777) != 0 ? (st.st_mode & 0777) : 0666);
  if (ret < 0)
    {
      goto errout_with_lfile;
    }

  for (;;)
    {
      ssize_t nrd = lops->read(&lfile, buf, UNIONFS_COPYBUFSZ);
      if (nrd < 0)
        {
          ret = (int)nrd;
          goto errout_with_files;
        }

      if (nrd == 0)
        {
          break;
        }

      for (off = 0; off < nrd; )
        {
          ssize_t nwr = uops->write(&ufile, buf + off, nrd - off);
          if (nwr <= 0)
            {
              ret = (nwr < 0) ? (int)nwr : -EIO;
              goto errout_with_files;
            }

          off += nwr;
        }

      total += nrd;
    }

  ret = OK;

errout_with_files:
  uops->close(&ufile);
errout_with_lfile:
  lops->close(&lfile);
errout_with_buf:
  fs_heap_free(buf);

  if (ret >= 0)
    {
      /* 模式位尽力跟随（失败不影响语义） */

      if (uops->chstat != NULL && (st.st_mode & 0777) != 0)
        {
          struct stat sb;

          memset(&sb, 0, sizeof(sb));
          sb.st_mode = st.st_mode & 0777;
          uops->chstat(umu->um_node, upath, &sb, CH_STAT_MODE);
        }

      _alert("ORT: unionfs copy-up: %s (%ld bytes)\n", relpath, (long)total);
    }

  return ret;
}

/****************************************************************************
 * Name: unionfs_hidemarker
 *
 * Description:
 *   [ORT §85] 名字是否属于白障命名域（`.wh.` 前缀；含 opaque 标记
 *   `.wh..wh..opq`）。标记是 unionfs 的私有簿记：视图内不可见、不可
 *   寻址（解层侧 orting 的 OCI 白障同用此命名域）。
 *
 ****************************************************************************/

static bool unionfs_hidemarker(FAR const char *relpath)
{
  FAR const char *base;

  if (relpath == NULL || *relpath == '\0')
    {
      return false;
    }

  base = strrchr(relpath, '/');
  base = (base == NULL) ? relpath : base + 1;

  return strncmp(base, ".wh.", 4) == 0;
}

/****************************************************************************
 * Name: unionfs_iswhiteout
 *
 * Description:
 *   [ORT §85] relpath 是否被上层白障遮蔽（上层同目录存在 `.wh.<名>`）。
 *   标记自身不算被白障。用裸 trystat（不经视图语义，防递归）。
 *
 ****************************************************************************/

static bool unionfs_iswhiteout(FAR struct unionfs_inode_s *ui,
                               FAR const char *relpath)
{
  FAR struct unionfs_mountpt_s *umu = &ui->ui_fs[0];
  FAR const char *base;
  FAR char *dir = NULL;
  FAR char *marker = NULL;
  struct stat buf;
  bool found;

  if (relpath == NULL || *relpath == '\0' ||
      unionfs_hidemarker(relpath))
    {
      return false;
    }

  base = strrchr(relpath, '/');
  if (base != NULL)
    {
      dir = fs_heap_strndup(relpath, base - relpath);
      if (dir == NULL)
        {
          return false;                /* 内存不足：保守按"无白障" */
        }

      base++;
    }
  else
    {
      base = relpath;
    }

  if (dir != NULL)
    {
      if (fs_heap_asprintf(&marker, "%s/.wh.%s", dir, base) < 0)
        {
          marker = NULL;
        }
    }
  else if (fs_heap_asprintf(&marker, ".wh.%s", base) < 0)
    {
      marker = NULL;
    }

  found = (marker != NULL &&
           unionfs_trystat(umu->um_node, marker, umu->um_prefix, &buf) >= 0);

  fs_heap_free(marker);
  fs_heap_free(dir);
  return found;
}

/****************************************************************************
 * Name: unionfs_isopaque
 *
 * Description:
 *   [ORT §85] 目录是否带 opaque 标记（上层 `<目录>/.wh..wh..opq`）——
 *   视图里**屏蔽下层同目录的全部子项**（"删目录后重建"的语义：新目录
 *   是全新的，下层旧内容不得渗出）。根目录 relpath = ""。
 *
 ****************************************************************************/

static bool unionfs_isopaque(FAR struct unionfs_inode_s *ui,
                             FAR const char *relpath)
{
  FAR struct unionfs_mountpt_s *umu = &ui->ui_fs[0];
  FAR char *marker = NULL;
  struct stat buf;
  bool found;

  if (relpath == NULL)
    {
      return false;
    }

  if (*relpath == '\0')
    {
      if (fs_heap_asprintf(&marker, ".wh..wh..opq") < 0)
        {
          marker = NULL;
        }
    }
  else if (fs_heap_asprintf(&marker, "%s/.wh..wh..opq", relpath) < 0)
    {
      marker = NULL;
    }

  found = (marker != NULL &&
           unionfs_trystat(umu->um_node, marker, umu->um_prefix, &buf) >= 0);

  fs_heap_free(marker);
  return found;
}

/****************************************************************************
 * Name: unionfs_lowerhidden
 *
 * Description:
 *   [ORT §85] 下层路径是否被视图屏蔽 —— **逐级**祖先检查（白障使该级
 *   隐；opq 使其下全部下层子项隐）加叶节点白障。必须逐级：VFS 把完整
 *   relpath 直接交给 mountpt ops，不经逐组件解析 —— 只查叶节点会让
 *   `dirX/f` 从被删除的 dirX 下漏出来。
 *
 ****************************************************************************/

static bool unionfs_lowerhidden(FAR struct unionfs_inode_s *ui,
                                FAR const char *relpath)
{
  FAR char *dup;
  FAR char *slash;
  bool hidden = false;

  if (relpath == NULL || *relpath == '\0')
    {
      return false;
    }

  dup = fs_heap_strdup(relpath);
  if (dup == NULL)
    {
      return false;
    }

  for (slash = strchr(dup + 1, '/'); slash != NULL;
       slash = strchr(slash + 1, '/'))
    {
      *slash = '\0';

      hidden = unionfs_iswhiteout(ui, dup) || unionfs_isopaque(ui, dup);

      *slash = '/';

      if (hidden)
        {
          break;
        }
    }

  fs_heap_free(dup);

  if (!hidden)
    {
      hidden = unionfs_iswhiteout(ui, relpath);
    }

  return hidden;
}

/****************************************************************************
 * Name: unionfs_mkmarker
 *
 * Description:
 *   [ORT §85] 建标记：whiteout（`<父>/.wh.<名>`，隐藏下层同名项）或
 *   opaque（`<目录>/.wh..wh..opq`，屏蔽下层子项）。父目录先上移；成功
 *   打内核见证行。
 *
 ****************************************************************************/

static int unionfs_mkmarker(FAR struct unionfs_inode_s *ui,
                            FAR const char *relpath, bool opaque)
{
  FAR struct unionfs_mountpt_s *umu = &ui->ui_fs[0];
  FAR const struct mountpt_operations *ops = umu->um_node->u.i_mops;
  FAR const char *base;
  FAR const char *mrel;
  FAR char *dir = NULL;
  FAR char *marker = NULL;
  struct file mfile;
  int ret;

  if (relpath == NULL || *relpath == '\0' ||
      unionfs_hidemarker(relpath))
    {
      return -EINVAL;
    }

  ret = unionfs_mkparents(ui, relpath);
  if (ret < 0)
    {
      return ret;
    }

  if (opaque)
    {
      if (fs_heap_asprintf(&marker, "%s/.wh..wh..opq", relpath) < 0)
        {
          marker = NULL;
        }
    }
  else
    {
      base = strrchr(relpath, '/');
      if (base != NULL)
        {
          dir = fs_heap_strndup(relpath, base - relpath);
          if (dir == NULL)
            {
              return -ENOMEM;
            }

          base++;
        }
      else
        {
          base = relpath;
        }

      if (dir != NULL)
        {
          if (fs_heap_asprintf(&marker, "%s/.wh.%s", dir, base) < 0)
            {
              marker = NULL;
            }
        }
      else if (fs_heap_asprintf(&marker, ".wh.%s", base) < 0)
        {
          marker = NULL;
        }
    }

  if (marker == NULL || ops->open == NULL || ops->close == NULL)
    {
      ret = -ENOMEM;
      goto errout;
    }

  mrel = unionfs_offsetpath(marker, umu->um_prefix);
  if (mrel == NULL)
    {
      ret = -ENOENT;
      goto errout;
    }

  memset(&mfile, 0, sizeof(mfile));
  mfile.f_oflags = O_WRONLY | O_CREAT | O_TRUNC;
  mfile.f_inode  = umu->um_node;

  ret = ops->open(&mfile, mrel, O_WRONLY | O_CREAT | O_TRUNC, 0);
  if (ret < 0)
    {
      goto errout;
    }

  ops->close(&mfile);
  ret = OK;

  if (opaque)
    {
      _alert("ORT: unionfs opaque: %s\n", relpath);
    }
  else
    {
      _alert("ORT: unionfs whiteout: %s\n", relpath);
    }

errout:
  fs_heap_free(marker);
  fs_heap_free(dir);
  return ret;
}

/****************************************************************************
 * Name: unionfs_dirviewempty
 *
 * Description:
 *   [ORT §85] 目录在**视图**里是否为空（rmdir 的 ENOTEMPTY 判定）：
 *   上层项跳过标记；下层项跳过标记、被上层同名遮蔽的、被上层白障
 *   遮蔽的；opaque 目录直接屏蔽下层全部子项。返回 OK=空，
 *   -ENOTEMPTY=非空，其它负值为底层错误（fail-closed）。
 *
 ****************************************************************************/

static int unionfs_dirviewempty(FAR struct unionfs_inode_s *ui,
                                FAR const char *relpath)
{
  FAR struct unionfs_mountpt_s *um;
  FAR const struct mountpt_operations *ops;
  FAR struct fs_dirent_s *dir;
  struct dirent entry;
  struct stat buf;
  int ndx;
  int ret;

  for (ndx = 0; ndx <= 1; ndx++)
    {
      if (ndx == 1 && unionfs_isopaque(ui, relpath))
        {
          /* [ORT §85] opaque：下层子项整体屏蔽 */

          break;
        }

      um = &ui->ui_fs[ndx];
      ops = um->um_node->u.i_mops;
      dir = NULL;

      ret = unionfs_tryopendir(um->um_node, relpath, um->um_prefix, &dir);
      if (ret < 0)
        {
          continue;                    /* 该侧没有此目录 */
        }

      for (;;)
        {
          ret = ops->readdir(um->um_node, dir, &entry);
          if (ret == -ENOENT)
            {
              ret = OK;                /* 该侧列完 */
              break;
            }

          if (ret < 0)
            {
              break;                   /* 底层错误：fail-closed 上报 */
            }

          if (unionfs_hidemarker(entry.d_name))
            {
              continue;
            }

          if (ndx == 1)
            {
              FAR char *cpath = unionfs_relpath(relpath, entry.d_name);

              if (cpath != NULL)
                {
                  if (unionfs_trystat(ui->ui_fs[0].um_node, cpath,
                                      ui->ui_fs[0].um_prefix, &buf) >= 0 ||
                      unionfs_iswhiteout(ui, cpath))
                    {
                      fs_heap_free(cpath);
                      continue;
                    }

                  fs_heap_free(cpath);
                }
            }

          ret = -ENOTEMPTY;
          break;
        }

      if (ops->closedir != NULL)
        {
          ops->closedir(um->um_node, dir);
        }

      if (ret == -ENOTEMPTY || ret < 0)
        {
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: unionfs_clearmarkers
 *
 * Description:
 *   [ORT §85] 清掉上层目录内的全部标记（`.wh.*`）。rmdir 前用：视图
 *   已判空的上层目录，物理上只剩标记 —— tmpfs 视它们为真文件，不清掉
 *   会报 EBUSY（冒烟实测 errno=16）。逐个单查单删（不在枚举中途
 *   unlink，防迭代器失效）。
 *
 ****************************************************************************/

static int unionfs_clearmarkers(FAR struct unionfs_inode_s *ui,
                                FAR const char *dirrelpath)
{
  FAR struct unionfs_mountpt_s *umu = &ui->ui_fs[0];
  FAR const struct mountpt_operations *ops = umu->um_node->u.i_mops;
  FAR struct fs_dirent_s *dir;
  struct dirent entry;
  FAR char *mpath;
  int ret;

  for (;;)
    {
      mpath = NULL;
      dir   = NULL;

      ret = unionfs_tryopendir(umu->um_node, dirrelpath, umu->um_prefix,
                               &dir);
      if (ret < 0)
        {
          return ret;
        }

      for (;;)
        {
          ret = ops->readdir(umu->um_node, dir, &entry);
          if (ret < 0)
            {
              break;
            }

          if (unionfs_hidemarker(entry.d_name))
            {
              mpath = unionfs_relpath(dirrelpath, entry.d_name);
              break;
            }
        }

      if (ops->closedir != NULL)
        {
          ops->closedir(umu->um_node, dir);
        }

      if (mpath == NULL)
        {
          return OK;                   /* 标记已清光 */
        }

      ret = unionfs_tryunlink(umu->um_node, mpath, umu->um_prefix);
      fs_heap_free(mpath);

      if (ret < 0)
        {
          return ret;
        }
    }
}

/****************************************************************************
 * Name: unionfs_open
 ****************************************************************************/

static int unionfs_open(FAR struct file *filep, FAR const char *relpath,
                        int oflags, mode_t mode)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_file_s *uf;
  FAR struct unionfs_mountpt_s *um;
  int ret;

  /* Recover the open file data from the struct file instance */

  ui = filep->f_inode->i_private;

  finfo("Opening: ui_nopen=%d\n", ui->ui_nopen);

  /* Get exclusive access to the file system data structures */

  ret = nxmutex_lock(&ui->ui_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Allocate a container to hold the open file system information */

  uf = (FAR struct unionfs_file_s *)
    fs_heap_zalloc(sizeof(struct unionfs_file_s));
  if (uf == NULL)
    {
      ret = -ENOMEM;
      goto errout_with_lock;
    }

  /* Try to open the file on file system 1 */

  um = &ui->ui_fs[0];
  DEBUGASSERT(um != NULL && um->um_node != NULL &&
              um->um_node->u.i_mops != NULL);

  if ((oflags & O_ACCMODE) != O_RDONLY ||
      (oflags & (O_CREAT | O_TRUNC | O_APPEND)) != 0)
    {
      /* [ORT §84] 写意图（含 O_CREAT/O_TRUNC/O_APPEND —— 创建与截断
       * 都是修改）：**先按视图状态定 copy-up，再开上层**。不能靠
       * "上层 O_CREAT 直接成功建空 shadow" —— 那会把下层既有内容悄悄
       * 丢掉（append/无截断写），冒烟实测踩过。规则：
       *   上层已有同名（shadow）→ 直接开上层；
       *   仅下层有常规文件     → 父目录上移 + 整份 copy-up，再开上层；
       *   仅下层有目录         → -EISDIR；
       *   两处都没有           → 父目录上移，创建归上层（无 O_CREAT
       *                          则照旧 ENOENT）。
       * **绝不回落下层写**（原实现的回落 = 直写污染镜像层）。
       */

      FAR struct unionfs_mountpt_s *uml = &ui->ui_fs[1];
      struct stat st;

      /* [ORT §85] 白障命名域保留（标记是私有簿记，视图不可建） */

      if (unionfs_hidemarker(relpath))
        {
          ret = -EPERM;
          goto errout_with_uf;
        }

      ret = unionfs_trystat(um->um_node, relpath, um->um_prefix, &st);
      if (ret < 0)
        {
          ret = unionfs_trystat(uml->um_node, relpath, uml->um_prefix, &st);
          if (ret >= 0)
            {
              if (S_ISREG(st.st_mode))
                {
                  /* [ORT §85] 下层同名被白障遮蔽 → 视图里"不存在"：
                   * 不得 copy-up（否则把已删文件复活），按新建走。 */

                  ret = unionfs_mkparents(ui, relpath);
                  if (ret >= 0 && !unionfs_lowerhidden(ui, relpath))
                    {
                      ret = unionfs_copyup(ui, relpath);
                    }
                }
              else if (S_ISDIR(st.st_mode))
                {
                  ret = -EISDIR;
                }
              else
                {
                  ret = -EINVAL;
                }
            }
          else
            {
              /* 下层也没有：父目录上移（祖先须视图可见），建归上层。 */

              ret = unionfs_mkparents(ui, relpath);
            }
        }

      if (ret >= 0)
        {
          uf->uf_file.f_oflags = filep->f_oflags;
          uf->uf_file.f_inode  = um->um_node;

          ret = unionfs_tryopen(&uf->uf_file, relpath, um->um_prefix,
                                oflags, mode);
        }

      if (ret < 0)
        {
          goto errout_with_uf;
        }

      /* Successfully opened on file system 1 (shadow / copy-up / create) */

      uf->uf_ndx = 0;
    }
  else
    {
      /* Read-only open：上层优先（shadow），回落只读下层 ——
       * [ORT §85] 但白障遮蔽/标记名一律不可见。 */

      if (unionfs_hidemarker(relpath))
        {
          ret = -ENOENT;
          goto errout_with_uf;
        }

      uf->uf_file.f_oflags = filep->f_oflags;
      uf->uf_file.f_inode  = um->um_node;

      ret = unionfs_tryopen(&uf->uf_file, relpath, um->um_prefix, oflags,
                            mode);
      if (ret >= 0)
        {
          /* Successfully opened on file system 1 */

          uf->uf_ndx = 0;
        }
      else if (unionfs_lowerhidden(ui, relpath))
        {
          /* 视图里不存在（下层同名被白障遮蔽） */

          ret = -ENOENT;
          goto errout_with_uf;
        }
      else
        {
          um  = &ui->ui_fs[1];

          uf->uf_file.f_oflags = filep->f_oflags;
          uf->uf_file.f_inode  = um->um_node;

          ret = unionfs_tryopen(&uf->uf_file, relpath, um->um_prefix, oflags,
                                mode);
          if (ret < 0)
            {
              goto errout_with_uf;
            }

          /* Successfully opened on file system 1 */

          uf->uf_ndx = 1;
        }
    }

  /* Increment the open reference count */

  ui->ui_nopen++;
  DEBUGASSERT(ui->ui_nopen > 0);

  /* Save our private data in the file structure */

  filep->f_priv = (FAR void *)uf;
  ret = OK;

errout_with_lock:
  nxmutex_unlock(&ui->ui_lock);
  return ret;

errout_with_uf:
  /* [ORT §84] 顺带修复：原实现 open 失败路径泄漏 uf（上游同款）。 */

  fs_heap_free(uf);
  goto errout_with_lock;
}

/****************************************************************************
 * Name: unionfs_close
 ****************************************************************************/

static int unionfs_close(FAR struct file *filep)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_file_s *uf;
  FAR struct unionfs_mountpt_s *um;
  FAR const struct mountpt_operations *ops;
  int ret = OK;

  /* Recover the open file data from the struct file instance */

  ui = filep->f_inode->i_private;

  /* Get exclusive access to the file system data structures */

  ret = nxmutex_lock(&ui->ui_lock);
  if (ret < 0)
    {
      return ret;
    }

  DEBUGASSERT(ui != NULL && filep->f_priv != NULL);
  uf = (FAR struct unionfs_file_s *)filep->f_priv;

  DEBUGASSERT(uf->uf_ndx == 0 || uf->uf_ndx == 1);
  um = &ui->ui_fs[uf->uf_ndx];

  DEBUGASSERT(um != NULL && um->um_node != NULL &&
              um->um_node->u.i_mops != NULL);
  ops = um->um_node->u.i_mops;

  finfo("Closing: ui_nopen=%d\n", ui->ui_nopen);

  /* Perform the lower level close operation */

  if (ops->close != NULL)
    {
      ret = ops->close(&uf->uf_file);
    }

  /* Decrement the count of open reference.  If that count would go to zero
   * and if the file system has been unmounted, then destroy the file system
   * now.
   */

  if (--ui->ui_nopen <= 0 && ui->ui_unmounted)
    {
      unionfs_destroy(ui);
    }
  else
    {
      nxmutex_unlock(&ui->ui_lock);
    }

  /* Free the open file container */

  fs_heap_free(uf);
  filep->f_priv = NULL;
  return ret;
}

/****************************************************************************
 * Name: unionfs_read
 ****************************************************************************/

static ssize_t unionfs_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_file_s *uf;
  FAR struct unionfs_mountpt_s *um;
  FAR const struct mountpt_operations *ops;

  finfo("buflen: %lu\n", (unsigned long)buflen);

  /* Recover the open file data from the struct file instance */

  ui = filep->f_inode->i_private;

  DEBUGASSERT(ui != NULL && filep->f_priv != NULL);
  uf = (FAR struct unionfs_file_s *)filep->f_priv;

  DEBUGASSERT(uf->uf_ndx == 0 || uf->uf_ndx == 1);
  um = &ui->ui_fs[uf->uf_ndx];

  DEBUGASSERT(um != NULL && um->um_node != NULL &&
              um->um_node->u.i_mops != NULL);
  ops = um->um_node->u.i_mops;

  /* Perform the lower level read operation */

  return ops->read ? ops->read(&uf->uf_file, buffer, buflen) : -EPERM;
}

/****************************************************************************
 * Name: unionfs_write
 ****************************************************************************/

static ssize_t unionfs_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_file_s *uf;
  FAR struct unionfs_mountpt_s *um;
  FAR const struct mountpt_operations *ops;

  finfo("buflen: %lu\n", (unsigned long)buflen);

  /* Recover the open file data from the struct file instance */

  ui = filep->f_inode->i_private;

  DEBUGASSERT(ui != NULL && filep->f_priv != NULL);
  uf = (FAR struct unionfs_file_s *)filep->f_priv;

  DEBUGASSERT(uf->uf_ndx == 0 || uf->uf_ndx == 1);
  um = &ui->ui_fs[uf->uf_ndx];

  DEBUGASSERT(um != NULL && um->um_node != NULL &&
              um->um_node->u.i_mops != NULL);
  ops = um->um_node->u.i_mops;

  /* [ORT §84] 绊线：写意图的 open 一律 copy-up 到上层（uf_ndx==0），
   * 落到下层 fd 的写 = copy-up 漏网 —— fail-closed，宁可 EROFS 也
   * 不改镜像层。
   */

  if (uf->uf_ndx != 0)
    {
      _alert("ORT: unionfs BUG: write on lower fd — copy-up missed\n");
      return -EROFS;
    }

  /* Perform the lower level write operation */

  return ops->write ? ops->write(&uf->uf_file, buffer, buflen) : -EPERM;
}

/****************************************************************************
 * Name: unionfs_seek
 ****************************************************************************/

static off_t unionfs_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_file_s *uf;
  FAR struct unionfs_mountpt_s *um;
  FAR const struct mountpt_operations *ops;

  finfo("offset: %lu whence: %d\n", (unsigned long)offset, whence);

  /* Recover the open file data from the struct file instance */

  ui = filep->f_inode->i_private;

  DEBUGASSERT(ui != NULL && filep->f_priv != NULL);
  uf = (FAR struct unionfs_file_s *)filep->f_priv;

  DEBUGASSERT(uf->uf_ndx == 0 || uf->uf_ndx == 1);
  um = &ui->ui_fs[uf->uf_ndx];

  DEBUGASSERT(um != NULL && um->um_node != NULL &&
              um->um_node->u.i_mops != NULL);
  ops = um->um_node->u.i_mops;

  /* Invoke the file seek method if available */

  if (ops->seek != NULL)
    {
      offset = ops->seek(&uf->uf_file, offset, whence);
    }
  else
    {
      int ret;

      /* Get exclusive access to the file system data structures */

      ret = nxmutex_lock(&ui->ui_lock);
      if (ret < 0)
        {
          return ret;
        }

      /* No... Just set the common file position value */

      switch (whence)
        {
          case SEEK_CUR:
            offset += filep->f_pos;

          case SEEK_SET:
            if (offset >= 0)
              {
                filep->f_pos = offset; /* Might be beyond the end-of-file */
              }
            else
              {
                offset = (off_t)-EINVAL;
              }
            break;

          case SEEK_END:
            offset = (off_t)-ENOSYS;
            break;

          default:
            offset = (off_t)-EINVAL;
            break;
        }

      nxmutex_unlock(&ui->ui_lock);
    }

  return offset;
}

/****************************************************************************
 * Name: unionfs_ioctl
 ****************************************************************************/

static int unionfs_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_file_s *uf;
  FAR struct unionfs_mountpt_s *um;
  FAR const struct mountpt_operations *ops;

  finfo("cmd: %d arg: %lu\n", cmd, arg);

  /* Recover the open file data from the struct file instance */

  ui = filep->f_inode->i_private;

  DEBUGASSERT(ui != NULL && filep->f_priv != NULL);
  uf = (FAR struct unionfs_file_s *)filep->f_priv;

  DEBUGASSERT(uf->uf_ndx == 0 || uf->uf_ndx == 1);
  um = &ui->ui_fs[uf->uf_ndx];

  DEBUGASSERT(um != NULL && um->um_node != NULL &&
              um->um_node->u.i_mops != NULL);
  ops = um->um_node->u.i_mops;

  /* Perform the lower level ioctl operation */

  return ops->ioctl ? ops->ioctl(&uf->uf_file, cmd, arg) : -ENOTTY;
}

/****************************************************************************
 * Name: unionfs_sync
 ****************************************************************************/

static int unionfs_sync(FAR struct file *filep)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_file_s *uf;
  FAR struct unionfs_mountpt_s *um;
  FAR const struct mountpt_operations *ops;

  finfo("filep=%p\n", filep);

  /* Recover the open file data from the struct file instance */

  ui = filep->f_inode->i_private;

  DEBUGASSERT(ui != NULL && filep->f_priv != NULL);
  uf = (FAR struct unionfs_file_s *)filep->f_priv;

  DEBUGASSERT(uf->uf_ndx == 0 || uf->uf_ndx == 1);
  um = &ui->ui_fs[uf->uf_ndx];

  DEBUGASSERT(um != NULL && um->um_node != NULL &&
              um->um_node->u.i_mops != NULL);
  ops = um->um_node->u.i_mops;

  /* Perform the lower level sync operation */

  return ops->sync ? ops->sync(&uf->uf_file) : -EINVAL;
}

/****************************************************************************
 * Name: unionfs_dup
 ****************************************************************************/

static int unionfs_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct unionfs_file_s *oldpriv;
  FAR struct unionfs_file_s *newpriv;
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_mountpt_s *um;
  FAR const struct mountpt_operations *ops;
  int ret = -ENOMEM;

  finfo("oldp=%p newp=%p\n", oldp, newp);

  /* Recover the open file data from the struct file instance */

  DEBUGASSERT(oldp != NULL && oldp->f_inode != NULL);
  ui = oldp->f_inode->i_private;

  DEBUGASSERT(ui != NULL && oldp->f_priv != NULL);
  oldpriv = (FAR struct unionfs_file_s *)oldp->f_priv;

  DEBUGASSERT(oldpriv->uf_ndx == 0 || oldpriv->uf_ndx == 1);
  um = &ui->ui_fs[oldpriv->uf_ndx];

  DEBUGASSERT(um != NULL && um->um_node != NULL &&
              um->um_node->u.i_mops != NULL);
  ops = um->um_node->u.i_mops;

  DEBUGASSERT(newp != NULL && newp->f_priv == NULL);

  /* Allocate a new container for the union FS open file */

  newpriv = (FAR struct unionfs_file_s *)
    fs_heap_malloc(sizeof(struct unionfs_file_s));
  if (newpriv != NULL)
    {
      /* Clone the old file structure into the newly allocated one */

      memcpy(newpriv, oldpriv, sizeof(struct unionfs_file_s));
      newpriv->uf_file.f_priv = NULL;

      /* Then perform the lower lowel dup operation */

      ret = OK;
      if (ops->dup != NULL)
        {
          ret = ops->dup(&oldpriv->uf_file, &newpriv->uf_file);
          if (ret < 0)
            {
              fs_heap_free(newpriv);
              newpriv = NULL;
            }
        }

      /* Save the new container in the new file structure */

      newp->f_priv = newpriv;
    }

  return ret;
}

/****************************************************************************
 * Name: unionfs_fstat
 *
 * Description:
 *   Obtain information about an open file associated with the file
 *   descriptor 'fd', and will write it to the area pointed to by 'buf'.
 *
 ****************************************************************************/

static int unionfs_fstat(FAR const struct file *filep, FAR struct stat *buf)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_file_s *uf;
  FAR struct unionfs_mountpt_s *um;
  FAR const struct mountpt_operations *ops;

  finfo("filep=%p buf=%p\n", filep, buf);

  /* Recover the open file data from the struct file instance */

  ui = filep->f_inode->i_private;

  DEBUGASSERT(ui != NULL && filep->f_priv != NULL);
  uf = (FAR struct unionfs_file_s *)filep->f_priv;

  DEBUGASSERT(uf->uf_ndx == 0 || uf->uf_ndx == 1);
  um = &ui->ui_fs[uf->uf_ndx];

  DEBUGASSERT(um != NULL && um->um_node != NULL &&
              um->um_node->u.i_mops != NULL);
  ops = um->um_node->u.i_mops;

  /* Perform the lower level write operation */

  return ops->fstat ? ops->fstat(&uf->uf_file, buf) : -EPERM;
}

/****************************************************************************
 * Name: unionfs_fchstat
 *
 * Description:
 *   Change information about an open file associated with the file
 *   descriptor 'filep'.
 *
 ****************************************************************************/

static int unionfs_fchstat(FAR const struct file *filep,
                           FAR const struct stat *buf, int flags)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_file_s *uf;
  FAR struct unionfs_mountpt_s *um;
  FAR const struct mountpt_operations *ops;

  finfo("filep=%p buf=%p\n", filep, buf);

  /* Recover the open file data from the struct file instance */

  ui = filep->f_inode->i_private;

  DEBUGASSERT(ui != NULL && filep->f_priv != NULL);
  uf = (FAR struct unionfs_file_s *)filep->f_priv;

  DEBUGASSERT(uf->uf_ndx == 0 || uf->uf_ndx == 1);
  um = &ui->ui_fs[uf->uf_ndx];

  DEBUGASSERT(um != NULL && um->um_node != NULL &&
              um->um_node->u.i_mops != NULL);
  ops = um->um_node->u.i_mops;

  /* [ORT §84] 绊线：fd 的元数据修改也不许落下层。 */

  if (uf->uf_ndx != 0)
    {
      _alert("ORT: unionfs BUG: fchstat on lower fd — copy-up missed\n");
      return -EROFS;
    }

  /* Perform the lower level change operation */

  return ops->fchstat ? ops->fchstat(&uf->uf_file, buf, flags) : -EPERM;
}

/****************************************************************************
 * Name: unionfs_truncate
 *
 * Description:
 *   Set the size of the file references by 'filep' to 'length'.
 *
 ****************************************************************************/

static int unionfs_truncate(FAR struct file *filep, off_t length)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_file_s *uf;
  FAR struct unionfs_mountpt_s *um;
  FAR const struct mountpt_operations *ops;

  finfo("filep=%p length=%ld\n", filep, (long)length);

  /* Recover the open file data from the struct file instance */

  ui = filep->f_inode->i_private;

  DEBUGASSERT(ui != NULL && filep->f_priv != NULL);
  uf = (FAR struct unionfs_file_s *)filep->f_priv;

  DEBUGASSERT(uf->uf_ndx == 0 || uf->uf_ndx == 1);
  um = &ui->ui_fs[uf->uf_ndx];

  DEBUGASSERT(um != NULL && um->um_node != NULL &&
              um->um_node->u.i_mops != NULL);
  ops = um->um_node->u.i_mops;

  /* [ORT §84] 绊线：同 unionfs_write —— 截断也绝不打下层 fd
   * （fd 开在上层 → 正常；开在下层 = copy-up 漏网，fail-closed）。
   */

  if (uf->uf_ndx != 0)
    {
      _alert("ORT: unionfs BUG: truncate on lower fd — copy-up missed\n");
      return -EROFS;
    }

  /* Perform the lower level write operation */

  return ops->truncate ? ops->truncate(&uf->uf_file, length) : -EPERM;
}

/****************************************************************************
 * Name: unionfs_opendir
 ****************************************************************************/

static int unionfs_opendir(FAR struct inode *mountpt,
                           FAR const char *relpath,
                           FAR struct fs_dirent_s **dir)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_mountpt_s *um;
  FAR struct unionfs_dir_s *udir;
  int ret;

  finfo("relpath: \"%s\"\n", relpath ? relpath : "NULL");

  if (!relpath)
    {
      return -EINVAL;
    }

  /* Recover the filesystem data from the struct inode instance */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);
  ui = mountpt->i_private;

  udir = fs_heap_zalloc(sizeof(*udir));
  if (udir == NULL)
    {
      return -ENOMEM;
    }

  /* Get exclusive access to the file system data structures */

  ret = nxmutex_lock(&ui->ui_lock);
  if (ret < 0)
    {
      goto errout_with_udir;
    }

  DEBUGASSERT(dir);

  /* [ORT §85] 标记路径不可寻址；被白障遮蔽（且上层无同名）的目录在
   * 视图里不存在。
   */

  if (unionfs_hidemarker(relpath) ||
      (unionfs_lowerhidden(ui, relpath) &&
       unionfs_trystatdir(ui->ui_fs[0].um_node, relpath,
                          ui->ui_fs[0].um_prefix) < 0))
    {
      ret = -ENOENT;
      goto errout_with_lock;
    }

  /* Clone the path.  We will need this when we traverse file system 2 to
   * omit duplicates on file system 1.
   */

  if (strlen(relpath) > 0)
    {
      udir->fu_relpath = fs_heap_strdup(relpath);
      if (!udir->fu_relpath)
        {
          goto errout_with_lock;
        }
    }

  /* Check file system 2 first.  [ORT §85] opaque 目录屏蔽下层 ——
   * 下层干脆不开（等价于下层目录为空）。
   */

  um = &ui->ui_fs[1];
  ret = -ENOENT;
  if (!unionfs_isopaque(ui, relpath))
    {
      ret = unionfs_tryopendir(um->um_node, relpath, um->um_prefix,
                               &udir->fu_lower[1]);
    }

  if (ret >= 0)
    {
      /* Save the file system 2 access info */

      udir->fu_ndx = 1;
      udir->fu_lower[1]->fd_root = um->um_node;
    }

  /* Check if the user is stat'ing some "fake" node between the unionfs root
   * and the file system 1/2 root directory.
   */

  else if (unionfs_ispartprefix(relpath, ui->ui_fs[1].um_prefix))
    {
      /* File system 2 prefix includes this relpath */

      udir->fu_ndx = 1;
      udir->fu_prefix[1] = true;
    }

  /* Check file system 1 last, possibly overwriting fu_ndx */

  um = &ui->ui_fs[0];
  ret = unionfs_tryopendir(um->um_node, relpath, um->um_prefix,
                           &udir->fu_lower[0]);
  if (ret >= 0)
    {
      /* Save the file system 1 access info */

      udir->fu_ndx = 0;
      udir->fu_lower[0]->fd_root = um->um_node;
    }
  else
    {
      /* Check if the user is stat'ing some "fake" node between the unionfs
       * root and the file system 1 root directory.
       */

      if (unionfs_ispartprefix(relpath, ui->ui_fs[0].um_prefix))
        {
          /* File system 1 offset includes this relpath.  Make sure that only
           * one
           */

          udir->fu_ndx = 0;
          udir->fu_prefix[0] = true;
          udir->fu_prefix[1] = false;
        }

      /* If the directory was not found on either file system, then we have
       * failed to open this path on either file system.
       */

      else if (udir->fu_lower[1] == NULL && !udir->fu_prefix[1])
        {
          /* Neither of the two path file systems include this relpath */

          ret = -ENOENT;
          goto errout_with_relpath;
        }
    }

  /* Increment the number of open references and return success */

  ui->ui_nopen++;
  DEBUGASSERT(ui->ui_nopen > 0);

  nxmutex_unlock(&ui->ui_lock);
  *dir = &udir->fu_base;
  return OK;

errout_with_relpath:
  if (udir->fu_relpath != NULL)
    {
      fs_heap_free(udir->fu_relpath);
    }

errout_with_lock:
  nxmutex_unlock(&ui->ui_lock);

errout_with_udir:
  fs_heap_free(udir);
  return ret;
}

/****************************************************************************
 * Name: unionfs_closedir
 ****************************************************************************/

static int unionfs_closedir(FAR struct inode *mountpt,
                            FAR struct fs_dirent_s *dir)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_mountpt_s *um;
  FAR const struct mountpt_operations *ops;
  FAR struct unionfs_dir_s *udir;
  int ret = OK;
  int i;

  finfo("mountpt=%p dir=%p\n", mountpt, dir);

  /* Recover the union file system data from the struct inode instance */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);
  ui = mountpt->i_private;

  /* Get exclusive access to the file system data structures */

  ret = nxmutex_lock(&ui->ui_lock);
  if (ret < 0)
    {
      return ret;
    }

  DEBUGASSERT(dir);
  udir = (FAR struct unionfs_dir_s *)dir;

  /* Close both contained file systems */

  for (i = 0; i < 2; i++)
    {
      /* Was this file system opened? */

      if (udir->fu_lower[i] != NULL)
        {
          um = &ui->ui_fs[i];

          DEBUGASSERT(um != NULL && um->um_node != NULL &&
                      um->um_node->u.i_mops != NULL);
          ops = um->um_node->u.i_mops;

          /* Perform the lower level closedir operation */

          if (ops->closedir != NULL)
            {
              ret = ops->closedir(um->um_node, udir->fu_lower[i]);
            }
        }
    }

  /* Free any allocated path */

  if (udir->fu_relpath != NULL)
    {
      fs_heap_free(udir->fu_relpath);
    }

  fs_heap_free(udir);

  /* Decrement the count of open reference.  If that count would go to zero
   * and if the file system has been unmounted, then destroy the file system
   * now.
   */

  if (--ui->ui_nopen <= 0 && ui->ui_unmounted)
    {
      unionfs_destroy(ui);
    }
  else
    {
      nxmutex_unlock(&ui->ui_lock);
    }

  return ret;
}

/****************************************************************************
 * Name: unionfs_readdir
 ****************************************************************************/

static int unionfs_readdir(FAR struct inode *mountpt,
                           FAR struct fs_dirent_s *dir,
                           FAR struct dirent *entry)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_mountpt_s *um;
  FAR struct unionfs_mountpt_s *um0;
  FAR const struct mountpt_operations *ops;
  FAR struct unionfs_dir_s *udir;
  FAR char *relpath;
  struct stat buf;
  bool duplicate;
  bool hidden;
  int ret = -ENOSYS;

  /* Recover the union file system data from the struct inode instance */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);
  ui = mountpt->i_private;

  DEBUGASSERT(dir);
  udir = (FAR struct unionfs_dir_s *)dir;

  /* Check if we are at the end of the directory listing. */

  if (udir->fu_eod)
    {
      /* End of file and error conditions are not distinguishable
       * with readdir.  Here we return -ENOENT to signal the end
       * of the directory.
       */

      return -ENOENT;
    }

  DEBUGASSERT(udir->fu_ndx == 0 || udir->fu_ndx == 1);
  um = &ui->ui_fs[udir->fu_ndx];

  /* Special case: If the open directory is a 'fake' node in the prefix on
   * one of the mounted file system, then we must also fake the return value.
   */

  if (udir->fu_prefix[udir->fu_ndx])
    {
      DEBUGASSERT(udir->fu_lower[udir->fu_ndx] == NULL &&
                  um->um_prefix != NULL);

      /* Copy the file system offset into the dirent structure.
       * REVISIT:  This will not handle the case where the prefix contains
       * the '/' character the so the offset appears to be multiple
       * directories.
       */

      strlcpy(entry->d_name, um->um_prefix, sizeof(entry->d_name));

      /* Describe this as a read only directory */

      entry->d_type = DTYPE_DIRECTORY;

      /* Increment the index to file system 2 (maybe) */

      if (udir->fu_ndx == 0 && (udir->fu_prefix[1] ||
                                udir->fu_lower[1] != NULL))
        {
          /* Yes.. set up to do file system 2 next time */

          udir->fu_ndx++;
        }
      else
        {
          /* No.. we are finished */

          udir->fu_eod = true;
        }

      return OK;
    }

  /* This is a normal, mediated file system readdir() */

  DEBUGASSERT(udir->fu_lower[udir->fu_ndx] != NULL);
  DEBUGASSERT(um->um_node != NULL && um->um_node->u.i_mops != NULL);
  ops = um->um_node->u.i_mops;

  finfo("fu_ndx: %d\n", udir->fu_ndx);

  /* Perform the lower level readdir operation */

  if (ops->readdir != NULL)
    {
      /* Loop if we discard duplicate directory entries in filey system 2 */

      do
        {
          /* Read the directory entry */

          ret = ops->readdir(um->um_node, udir->fu_lower[udir->fu_ndx],
                             entry);

          /* Did the read operation fail because we reached the end of the
           * directory?  In that case, the error would be -ENOENT.  If we
           * hit the end-of-directory on file system, we need to seamlessly
           * move to the second file system (if there is one).
           */

          if (ret == -ENOENT && udir->fu_ndx == 0)
            {
              /* Special case: If the open directory is a 'fake' node in the
               * prefix on file system2, then we must also fake the return
               * value.
               */

              if (udir->fu_prefix[1])
                {
                  DEBUGASSERT(udir->fu_lower[1] == NULL);

                  /* Switch to the second file system */

                  udir->fu_ndx = 1;
                  um = &ui->ui_fs[1];

                  DEBUGASSERT(um != NULL && um->um_prefix != NULL);

                  /* Copy the file system offset into the dirent structure.
                   * REVISIT:  This will not handle the case where the prefix
                   * contains the '/' character the so the offset appears to
                   * be multiple directories.
                   */

                  strlcpy(entry->d_name, um->um_prefix,
                          sizeof(entry->d_name));

                  /* Describe this as a read only directory */

                  entry->d_type = DTYPE_DIRECTORY;

                  /* Mark the end of the directory listing */

                  udir->fu_eod = true;

                  /* Check if have already reported something of this name
                   * in file system 1.
                   */

                  relpath = unionfs_relpath(udir->fu_relpath, um->um_prefix);
                  if (relpath)
                    {
                      int tmp;

                      /* Check if anything exists at this path on file
                       * system 1
                       */

                      um0 = &ui->ui_fs[0];
                      tmp = unionfs_trystat(um0->um_node, relpath,
                                            um0->um_prefix, &buf);

                      /* Free the allocated relpath */

                      fs_heap_free(relpath);

                      /* Check for a duplicate */

                      if (tmp >= 0)
                        {
                          /* There is something there!
                           * REVISIT: We could allow files and directories to
                           * have duplicate names.
                           */

                          return -ENOENT;
                        }
                    }

                  return OK;
                }

              /* No.. check for a normal directory access */

              else if (udir->fu_lower[1] != NULL)
                {
                  /* Switch to the second file system */

                  udir->fu_ndx = 1;
                  um = &ui->ui_fs[1];

                  DEBUGASSERT(um != NULL && um->um_node != NULL &&
                              um->um_node->u.i_mops != NULL);
                  ops = um->um_node->u.i_mops;

                  /* Make sure that the second file system directory
                   * enumeration is rewound to the beginning of the
                   * directory.
                   */

                  if (ops->rewinddir != NULL)
                    {
                      ret = ops->rewinddir(um->um_node, udir->fu_lower[1]);
                      if (ret < 0)
                        {
                          return ret;
                        }
                    }

                  /* Then try the read operation again */

                  ret = ops->readdir(um->um_node, udir->fu_lower[1], entry);
                }
            }

          /* Did we successfully read a directory from file system 2?  If
           * so, we need to omit an duplicates that should be occluded by
           * the matching file on file system 1 (if we are enumerating
           * file system 1).
           */

          duplicate = false;
          hidden = false;
          if (ret >= 0 && udir->fu_ndx == 0)
            {
              /* [ORT §85] 标记（.wh.* / .wh..wh..opq）是私有簿记，
               * 视图不可见。
               */

              hidden = unionfs_hidemarker(entry->d_name);
            }
          else if (ret >= 0 && udir->fu_ndx == 1 &&
                   udir->fu_lower[0] != NULL)
            {
              /* Get the relative path to the same file on file system 1.
               * NOTE: the on any failures we just assume that the filep
               * is not a duplicate.
               */

              relpath = unionfs_relpath(udir->fu_relpath, entry->d_name);
              if (relpath)
                {
                  int tmp;

                  /* Check if anything exists at this path on file system 1 */

                  um0 = &ui->ui_fs[0];
                  tmp = unionfs_trystat(um0->um_node, relpath,
                                        um0->um_prefix, &buf);
                  if (tmp >= 0)
                    {
                      /* There is something there!
                       * REVISIT: We could allow files and directories to
                       * have duplicate names.
                       */

                      duplicate = true;
                    }

                  /* [ORT §85] 下层同名被上层白障遮蔽 → 视同重复跳过 */

                  if (!duplicate && unionfs_iswhiteout(ui, relpath))
                    {
                      duplicate = true;
                    }

                  /* Free the allocated relpath */

                  fs_heap_free(relpath);
                }
            }
        }
      while (duplicate || hidden);
    }

  return ret;
}

/****************************************************************************
 * Name: unionfs_rewindir
 ****************************************************************************/

static int unionfs_rewinddir(FAR struct inode *mountpt,
                             FAR struct fs_dirent_s *dir)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_mountpt_s *um;
  FAR const struct mountpt_operations *ops;
  FAR struct unionfs_dir_s *udir;
  int ret = -EINVAL;

  finfo("mountpt=%p dir=%p\n", mountpt, dir);

  /* Recover the union file system data from the struct inode instance */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);
  ui = mountpt->i_private;

  DEBUGASSERT(dir);
  udir = (FAR struct unionfs_dir_s *)dir;

  /* Were we currently enumerating on file system 1?  If not, is an
   * enumeration possible on file system 1?
   */

  DEBUGASSERT(udir->fu_ndx == 0 || udir->fu_ndx == 1);
  if (/* udir->fu_ndx != 0 && */ udir->fu_prefix[0] || udir->fu_lower[0] != NULL)
    {
      /* Yes.. switch to file system 1 */

      udir->fu_ndx = 0;
    }

  if (!udir->fu_prefix[udir->fu_ndx])
    {
      DEBUGASSERT(udir->fu_lower[udir->fu_ndx] != NULL);
      um = &ui->ui_fs[udir->fu_ndx];

      DEBUGASSERT(um != NULL && um->um_node != NULL &&
                  um->um_node->u.i_mops != NULL);
      ops = um->um_node->u.i_mops;

      /* Perform the file system rewind operation */

      if (ops->rewinddir != NULL)
        {
          ret = ops->rewinddir(um->um_node, udir->fu_lower[udir->fu_ndx]);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: unionfs_bind
 ****************************************************************************/

static int unionfs_bind(FAR struct inode *blkdriver, FAR const void *data,
                        FAR void **handle)
{
  FAR const char *fspath1 = "";
  FAR const char *prefix1 = "";
  FAR const char *fspath2 = "";
  FAR const char *prefix2 = "";
  FAR char *dup;
  FAR char *tmp;
  FAR char *tok;
  int ret;

  /* Parse options from mount syscall */

  dup = tmp = fs_heap_strdup(data);
  if (!dup)
    {
      return -ENOMEM;
    }

  while ((tok = strsep(&tmp, ",")))
    {
      if (tok == strstr(tok, "fspath1="))
        {
          fspath1 = tok + 8;
        }
      else if (tok == strstr(tok, "prefix1="))
        {
          prefix1 = tok + 8;
        }
      else if (tok == strstr(tok, "fspath2="))
        {
          fspath2 = tok + 8;
        }
      else if (tok == strstr(tok, "prefix2="))
        {
          prefix2 = tok + 8;
        }
    }

  /* Call unionfs_dobind to do the real work. */

  ret = unionfs_dobind(fspath1, prefix1, fspath2, prefix2, handle);
  fs_heap_free(dup);

  return ret;
}

/****************************************************************************
 * Name: unionfs_unbind
 ****************************************************************************/

static int unionfs_unbind(FAR void *handle, FAR struct inode **blkdriver,
                          unsigned int flags)
{
  FAR struct unionfs_inode_s *ui;
  int ret;

  finfo("handle=%p blkdriver=%p flags=%x\n", handle, blkdriver, flags);

  /* Recover the union file system data from the struct inode instance */

  DEBUGASSERT(handle != NULL);
  ui = (FAR struct unionfs_inode_s *)handle;

  /* Get exclusive access to the file system data structures */

  ret = nxmutex_lock(&ui->ui_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Mark the file system as unmounted. */

  ui->ui_unmounted = true;

  /* If there are no open references, then we can destroy the file system
   * now.
   */

  if (ui->ui_nopen <= 0)
    {
      nxmutex_unlock(&ui->ui_lock);
      unionfs_destroy(ui);
    }
  else
    {
      nxmutex_unlock(&ui->ui_lock);
    }

  return OK;
}

/****************************************************************************
 * Name: unionfs_statfs
 ****************************************************************************/

static int unionfs_statfs(FAR struct inode *mountpt, FAR struct statfs *buf)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_mountpt_s *um1;
  FAR struct unionfs_mountpt_s *um2;
  FAR const struct mountpt_operations *ops1;
  FAR const struct mountpt_operations *ops2;
  FAR struct statfs *adj;
  struct statfs buf1;
  struct statfs buf2;
  uint64_t tmp;
  uint32_t ratiob16;
  int ret;

  finfo("mountpt=%p buf=%p\n", mountpt, buf);

  /* Recover the union file system data from the struct inode instance */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL && buf != NULL);
  ui = mountpt->i_private;

  /* Get statfs info from file system 1.
   *
   * REVISIT: What would it mean if one file system did not support statfs?
   * Perhaps we could simplify the following by simply insisting that both
   * file systems support the statfs method.
   */

  um1 = &ui->ui_fs[0];
  DEBUGASSERT(um1 != NULL && um1->um_node != NULL &&
              um1->um_node->u.i_mops != NULL);
  ops1 = um1->um_node->u.i_mops;

  um2 = &ui->ui_fs[1];
  DEBUGASSERT(um2 != NULL && um2->um_node != NULL &&
              um2->um_node->u.i_mops != NULL);
  ops2 = um2->um_node->u.i_mops;

  memset(&buf1, 0, sizeof(struct statfs));
  memset(&buf2, 0, sizeof(struct statfs));
  if (ops1->statfs != NULL && ops2->statfs != NULL)
    {
      ret = ops1->statfs(um1->um_node, &buf1);
      if (ret < 0)
        {
          return ret;
        }

      /* Get stafs info from file system 2 */

      ret = ops2->statfs(um2->um_node, &buf2);
      if (ret < 0)
        {
          return ret;
        }
    }
  else if (ops1->statfs != NULL)
    {
      /* We have statfs for file system 1 only */

      return ops1->statfs(um1->um_node, buf);
    }
  else if (ops2->statfs != NULL)
    {
      /* We have statfs for file system 2 only */

      return ops2->statfs(um2->um_node, buf);
    }
  else
    {
      /* We could not get stafs info from either file system */

      return -ENOSYS;
    }

  /* We get here is we successfully obtained statfs info from both file
   * systems.  Now combine those results into one statfs report, trying to
   * reconcile any conflicts between the file system geometries.
   */

  buf->f_type    = UNIONFS_MAGIC;
  buf->f_namelen = MIN(buf1.f_namelen, buf2.f_namelen);
  buf->f_files   = buf1.f_files + buf2.f_files;
  buf->f_ffree   = buf1.f_ffree + buf2.f_ffree;

  /* Things expressed in units of blocks are the only tricky ones.  We will
   * depend on a uint64_t * temporary to avoid arithmetic overflow.
   */

  buf->f_bsize           = buf1.f_bsize;
  if (buf1.f_bsize != buf2.f_bsize)
    {
      if (buf1.f_bsize < buf2.f_bsize)
        {
          tmp           = (((uint64_t)buf2.f_blocks *
                            (uint64_t)buf2.f_bsize) << 16);
          ratiob16      = (uint32_t)(tmp / buf1.f_bsize);
          adj           = &buf2;
        }
      else
        {
          buf->f_bsize  = buf2.f_bsize;
          tmp           = (((uint64_t)buf1.f_blocks *
                            (uint64_t)buf1.f_bsize) << 16);
          ratiob16      = (uint32_t)(tmp / buf2.f_bsize);
          adj           = &buf1;
        }

      tmp               = (uint64_t)adj->f_blocks * ratiob16;
      adj->f_blocks     = (off_t)(tmp >> 16);

      tmp               = (uint64_t)adj->f_bfree  * ratiob16;
      adj->f_bfree      = (off_t)(tmp >> 16);

      tmp               = (uint64_t)adj->f_bavail * ratiob16;
      adj->f_bavail     = (off_t)(tmp >> 16);
    }

  /* Then we can just sum the adjusted sizes */

  buf->f_blocks         = buf1.f_blocks + buf2.f_blocks;
  buf->f_bfree          = buf1.f_bfree  + buf2.f_bfree;
  buf->f_bavail         = buf1.f_bavail + buf2.f_bavail;

  return OK;
}

/****************************************************************************
 * Name: unionfs_unlink
 ****************************************************************************/

static int unionfs_unlink(FAR struct inode *mountpt,
                          FAR const char *relpath)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_mountpt_s *um;
  struct stat buf;
  int ust;
  int lst;
  bool whited;
  int ret;

  finfo("relpath: %s\n", relpath);

  /* Recover the union file system data from the struct inode instance */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL &&
              relpath != NULL);
  ui = mountpt->i_private;

  /* [ORT §85] 标记不可寻址 */

  if (unionfs_hidemarker(relpath))
    {
      return -ENOENT;
    }

  /* 视图状态：上层有？（裸 stat）目录一律 -EISDIR（保持原语义） */

  um  = &ui->ui_fs[0];
  ust = unionfs_trystat(um->um_node, relpath, um->um_prefix, &buf);
  if (ust >= 0 && S_ISDIR(buf.st_mode))
    {
      return -EISDIR;
    }

  um  = &ui->ui_fs[1];
  lst = unionfs_trystat(um->um_node, relpath, um->um_prefix, &buf);
  if (lst >= 0 && S_ISDIR(buf.st_mode))
    {
      return -EISDIR;
    }

  whited = unionfs_lowerhidden(ui, relpath);

  if (ust < 0)
    {
      /* 上层没有：视图里"存在"仅当下层有且未被白障。 */

      if (lst < 0 || whited)
        {
          return -ENOENT;
        }

      /* [ORT §85] 仅下层文件：**白障即删**（原实现直接 unlink 下层
       * = 删镜像层；§84 先行拒绝）。 */

      return unionfs_mkmarker(ui, relpath, false);
    }

  /* 上层有副本：删上层；下层同名仍在且未白障 → 补白障，防"删了又
   * 冒出来"（§65 起的已知"暴露"语义至此收口）。 */

  um  = &ui->ui_fs[0];
  ret = unionfs_tryunlink(um->um_node, relpath, um->um_prefix);
  if (ret < 0)
    {
      return ret;
    }

  if (lst >= 0 && !whited)
    {
      ret = unionfs_mkmarker(ui, relpath, false);
    }

  return ret;
}

/****************************************************************************
 * Name: unionfs_mkdir
 ****************************************************************************/

static int unionfs_mkdir(FAR struct inode *mountpt, FAR const char *relpath,
                         mode_t mode)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_mountpt_s *um;
  struct stat buf;
  int lst;
  bool whited;
  int ret;

  finfo("relpath: %s\n", relpath);

  /* Recover the union file system data from the struct inode instance */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL &&
              relpath != NULL);
  ui = mountpt->i_private;

  /* [ORT §85] 标记命名域保留 */

  if (unionfs_hidemarker(relpath))
    {
      return -EPERM;
    }

  /* Is there anything with this name on the upper file system? */

  um  = &ui->ui_fs[0];
  ret = unionfs_trystat(um->um_node, relpath, um->um_prefix, &buf);
  if (ret >= 0)
    {
      return -EEXIST;
    }

  /* [ORT §85] 下层同名：被白障遮蔽 → 视图里不存在（重建场景），不
   * EEXIST；未被白障 → 视图里存在，EEXIST。 */

  um     = &ui->ui_fs[1];
  lst    = unionfs_trystat(um->um_node, relpath, um->um_prefix, &buf);
  whited = unionfs_lowerhidden(ui, relpath);
  if (lst >= 0 && !whited)
    {
      return -EEXIST;
    }

  /* [ORT §84] 只在上层建目录（父目录先上移）—— 下层视为只读，绝不在
   * 下层建（原实现"两边都试、有一边成就算成"会把目录建进下层）。
   */

  ret = unionfs_mkparents(ui, relpath);
  if (ret < 0)
    {
      return ret;
    }

  um  = &ui->ui_fs[0];
  ret = unionfs_trymkdir(um->um_node, relpath, um->um_prefix, mode);
  if (ret < 0)
    {
      return ret;
    }

  /* [ORT §85] 重建场景（下层物理还有同名实体）：opq 标记屏蔽下层全部
   * 子项 —— 新目录是"全新"的，旧内容不得渗出。 */

  if (lst >= 0)
    {
      ret = unionfs_mkmarker(ui, relpath, true);
    }

  return ret;
}

/****************************************************************************
 * Name: unionfs_rmdir
 ****************************************************************************/

static int unionfs_rmdir(FAR struct inode *mountpt, FAR const char *relpath)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_mountpt_s *um;
  int ust;
  int lst;
  bool whited;
  int ret = OK;

  finfo("relpath: %s\n", relpath);

  /* Recover the union file system data from the struct inode instance */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL &&
              relpath != NULL);
  ui = mountpt->i_private;

  /* [ORT §85] 标记不可寻址 */

  if (unionfs_hidemarker(relpath))
    {
      return -ENOENT;
    }

  um     = &ui->ui_fs[0];
  ust    = unionfs_trystatdir(um->um_node, relpath, um->um_prefix);
  um     = &ui->ui_fs[1];
  lst    = unionfs_trystatdir(um->um_node, relpath, um->um_prefix);
  whited = unionfs_lowerhidden(ui, relpath);

  /* 视图里有没有这个目录：上层有（重建场景仍算）或下层有且未白障 */

  if (ust < 0 && (lst < 0 || whited))
    {
      return -ENOENT;
    }

  /* 视图非空 → -ENOTEMPTY（含"上层空、下层有内容"的合并视图；
   * opaque 目录屏蔽下层子项） */

  ret = unionfs_dirviewempty(ui, relpath);
  if (ret < 0)
    {
      return ret;
    }

  /* 上层有 → 先清标记再删上层（视图已验空 ⇒ 物理上只剩 .wh.* 标记；
   * tmpfs 视标记为真文件，不清则 EBUSY） */

  if (ust >= 0)
    {
      ret = unionfs_clearmarkers(ui, relpath);
      if (ret < 0)
        {
          return ret;
        }

      um  = &ui->ui_fs[0];
      ret = unionfs_tryrmdir(um->um_node, relpath, um->um_prefix);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* [ORT §85] 下层同名仍在且未白障 → 白障（防"删了又冒出来"） */

  if (lst >= 0 && !whited)
    {
      ret = unionfs_mkmarker(ui, relpath, false);
    }

  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: unionfs_rename
 ****************************************************************************/

static int unionfs_rename(FAR struct inode *mountpt,
                          FAR const char *oldrelpath,
                          FAR const char *newrelpath)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_mountpt_s *um;
  int ust;
  int lst;
  bool whited;
  int ret = -ENOENT;

  finfo("oldrelpath: %s newrelpath: %s\n", oldrelpath, newrelpath);

  /* Recover the union file system data from the struct inode instance */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);
  ui = mountpt->i_private;

  DEBUGASSERT(oldrelpath != NULL && oldrelpath != NULL);

  /* [ORT §85] 标记不可寻址 / 新名不得落进标记命名域 */

  if (unionfs_hidemarker(oldrelpath))
    {
      return -ENOENT;
    }

  if (unionfs_hidemarker(newrelpath))
    {
      return -EPERM;
    }

  /* 视图状态：上层有源？（裸 statfile）下层有源？白障？ */

  um     = &ui->ui_fs[0];
  ust    = unionfs_trystatfile(um->um_node, oldrelpath, um->um_prefix);
  um     = &ui->ui_fs[1];
  lst    = unionfs_trystatfile(um->um_node, oldrelpath, um->um_prefix);
  whited = unionfs_lowerhidden(ui, oldrelpath);

  if (ust < 0)
    {
      /* 视图里没有源 */

      if (lst < 0 || whited)
        {
          return -ENOENT;
        }

      /* [ORT §85] 源仅下层：copy-up 到旧名 → 上层改名 → 白障旧名
       * （原实现把 rename 直接打在下层 = 改镜像层）。 */

      ret = unionfs_mkparents(ui, oldrelpath);
      if (ret >= 0)
        {
          ret = unionfs_copyup(ui, oldrelpath);
        }

      if (ret < 0)
        {
          return ret;
        }
    }

  /* 新名的父目录先上移（新名可能落进仅下层有的目录） */

  ret = unionfs_mkparents(ui, newrelpath);
  if (ret < 0)
    {
      return ret;
    }

  um  = &ui->ui_fs[0];
  ret = unionfs_tryrename(um->um_node, oldrelpath, newrelpath,
                          um->um_prefix);
  if (ret < 0)
    {
      return ret;
    }

  /* 旧名下层同名仍在且未白障 → 补白障（否则改名后旧名"复活"）。 */

  if (lst >= 0 && !whited)
    {
      ret = unionfs_mkmarker(ui, oldrelpath, false);
    }

  return ret;
}

/****************************************************************************
 * Name: unionfs_stat
 ****************************************************************************/

static int unionfs_stat(FAR struct inode *mountpt, FAR const char *relpath,
                        FAR struct stat *buf)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_mountpt_s *um;
  int ret;

  finfo("relpath: %s\n", relpath);

  /* Recover the union file system data from the struct inode instance */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL &&
              relpath != NULL);
  ui = mountpt->i_private;

  /* [ORT §85] 标记路径视图不可见 */

  if (unionfs_hidemarker(relpath))
    {
      return -ENOENT;
    }

  /* stat this path on file system 1 */

  um  = &ui->ui_fs[0];
  ret = unionfs_trystat(um->um_node, relpath, um->um_prefix, buf);
  if (ret >= 0)
    {
      /* Return on the first success.  The first instance of the file will
       * shadow the second anyway.
       */

      return OK;
    }

  /* stat failed on the file system 1.  [ORT §85] 被白障遮蔽 → ENOENT。 */

  if (unionfs_lowerhidden(ui, relpath))
    {
      return -ENOENT;
    }

  /* Try again on file system 2. */

  um  = &ui->ui_fs[1];
  ret = unionfs_trystat(um->um_node, relpath, um->um_prefix, buf);
  if (ret >= 0)
    {
      /* Return on the first success.  The first instance of the file will
       * shadow the second anyway.
       */

      return OK;
    }

  /* Special case the unionfs root directory when both file systems are
   * offset.  In that case, both of the above trystat calls will fail.
   */

  if (ui->ui_fs[0].um_prefix != NULL && ui->ui_fs[1].um_prefix != NULL)
    {
      /* Most of the stat entries just do not apply */

      memset(buf, 0, sizeof(struct stat));

      /* Claim that this is a read-only directory */

      buf->st_mode = S_IFDIR | S_IROTH | S_IRGRP | S_IRUSR;

      /* Check if the user is stat'ing some "fake" node between the unionfs
       * root and the file system 1 root directory.
       */

      if (unionfs_ispartprefix(relpath, ui->ui_fs[0].um_prefix) ||
          unionfs_ispartprefix(relpath, ui->ui_fs[1].um_prefix))
        {
          ret = OK;
        }
      else
        {
          ret = -ENOENT;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: unionfs_chstat
 ****************************************************************************/

static int unionfs_chstat(FAR struct inode *mountpt, FAR const char *relpath,
                          FAR const struct stat *buf, int flags)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_mountpt_s *um;
  int ret;

  finfo("relpath: %s\n", relpath);

  /* Recover the union file system data from the struct inode instance */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL &&
              relpath != NULL);
  ui = mountpt->i_private;

  /* [ORT §85] 标记不可寻址 */

  if (unionfs_hidemarker(relpath))
    {
      return -ENOENT;
    }

  /* chstat this path on file system 1 */

  um  = &ui->ui_fs[0];
  ret = unionfs_trychstat(um->um_node, relpath, um->um_prefix, buf, flags);
  if (ret >= 0)
    {
      /* Return on the first success.  The first instance of the file will
       * shadow the second anyway.
       */

      return OK;
    }

  /* [ORT §85] 仅下层存在：copy-up 后改**上层副本**（§84 先行拒绝；
   * 直改下层 = 改镜像层元数据）。视图里不存在（白障/两处皆无）→
   * -ENOENT。
   */

  {
    struct stat stbuf;

    um  = &ui->ui_fs[1];
    ret = unionfs_trystat(um->um_node, relpath, um->um_prefix, &stbuf);
  }

  if (ret < 0)
    {
      return ret;
    }

  if (unionfs_lowerhidden(ui, relpath))
    {
      return -ENOENT;
    }

  ret = unionfs_mkparents(ui, relpath);
  if (ret >= 0)
    {
      ret = unionfs_copyup(ui, relpath);
    }

  if (ret < 0)
    {
      return ret;
    }

  um  = &ui->ui_fs[0];
  ret = unionfs_trychstat(um->um_node, relpath, um->um_prefix, buf, flags);

  return ret;
}

/****************************************************************************
 * Name: unionfs_getmount
 ****************************************************************************/

static int unionfs_getmount(FAR const char *path, FAR struct inode **inode)
{
  FAR struct inode *minode;
  struct inode_search_s desc;
  int ret;

  /* Find the mountpt */

  SETUP_SEARCH(&desc, path, false);

  ret = inode_find(&desc);
  if (ret < 0)
    {
      /* Mountpoint inode not found */

      goto errout_with_search;
    }

  /* Get the search results */

  minode = desc.node;
  DEBUGASSERT(minode != NULL);

  /* Verify that the inode is a mountpoint.
   *
   * REVISIT: If desc.relpath points to a non-empty string, then the path
   * does not really refer to a mountpoint but, rather, to a some entity
   * within the mounted volume.
   */

  if (!INODE_IS_MOUNTPT(minode))
    {
      /* Inode was found, but is it is not a mounpoint */

      ret = -EINVAL;
      goto errout_with_inode;
    }

  /* Success! */

  *inode = minode;
  RELEASE_SEARCH(&desc);
  return OK;

errout_with_inode:
  inode_release(minode);

errout_with_search:
  RELEASE_SEARCH(&desc);
  return ret;
}

/****************************************************************************
 * Name: unionfs_dobind
 ****************************************************************************/

static int unionfs_dobind(FAR const char *fspath1, FAR const char *prefix1,
                          FAR const char *fspath2, FAR const char *prefix2,
                          FAR void **handle)
{
  FAR struct unionfs_inode_s *ui;
  int ret;

  DEBUGASSERT(fspath1 != NULL && fspath2 != NULL && handle != NULL);

  /* Allocate a structure a structure that will describe the union file
   * system.
   */

  ui = (FAR struct unionfs_inode_s *)
    fs_heap_zalloc(sizeof(struct unionfs_inode_s));
  if (!ui)
    {
      ferr("ERROR: Failed to allocated union FS state structure\n");
      return -ENOMEM;
    }

  nxmutex_init(&ui->ui_lock);

  /* Get the inodes associated with fspath1 and fspath2 */

  ret = unionfs_getmount(fspath1, &ui->ui_fs[0].um_node);
  if (ret < 0)
    {
      ferr("ERROR: unionfs_getmount(fspath1) failed: %d\n", ret);
      goto errout_with_uinode;
    }

  ret = unionfs_getmount(fspath2, &ui->ui_fs[1].um_node);
  if (ret < 0)
    {
      ferr("ERROR: unionfs_getmount(fspath2) failed: %d\n", ret);
      goto errout_with_fs1;
    }

  /* Duplicate the prefix strings */

  if (prefix1 && strlen(prefix1) > 0)
    {
      ui->ui_fs[0].um_prefix = fs_heap_strdup(prefix1);
      if (ui->ui_fs[0].um_prefix == NULL)
        {
          ferr("ERROR: fs_heap_strdup(prefix1) failed\n");
          ret = -ENOMEM;
          goto errout_with_fs2;
        }
    }

  if (prefix2 && strlen(prefix2) > 0)
    {
      ui->ui_fs[1].um_prefix = fs_heap_strdup(prefix2);
      if (ui->ui_fs[1].um_prefix == NULL)
        {
          ferr("ERROR: fs_heap_strdup(prefix2) failed\n");
          ret = -ENOMEM;
          goto errout_with_prefix1;
        }
    }

  /* Unlink the contained mountpoint inodes from the pseudo file system.
   * The inodes will be marked as deleted so that they will be removed when
   * the reference count decrements to zero in inode_release().  Because we
   * hold a reference count on the inodes, they will not be deleted until
   * this logic calls inode_release() in the unionfs_destroy() function.
   */

  inode_remove(fspath1);
  inode_remove(fspath2);

  *handle = ui;
  return OK;

errout_with_prefix1:
  if (ui->ui_fs[0].um_prefix != NULL)
    {
      fs_heap_free(ui->ui_fs[0].um_prefix);
    }

errout_with_fs2:
  inode_release(ui->ui_fs[1].um_node);

errout_with_fs1:
  inode_release(ui->ui_fs[0].um_node);

errout_with_uinode:
  nxmutex_destroy(&ui->ui_lock);
  fs_heap_free(ui);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* !CONFIG_DISABLE_MOUNTPOINT && CONFIG_FS_UNIONFS */
