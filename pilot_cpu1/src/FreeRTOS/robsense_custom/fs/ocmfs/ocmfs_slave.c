#include <sys/types.h>
#include <sys/stat.h>
#include <sys/statfs.h>

#include <stdlib.h>
#include <string.h>

#include <fs/fs.h>
#include <fs/fat.h>
#include <fcntl.h>
#include "../fs_dirent.h"
#include "ocm/ocm.h"
#include "ocmfs.h"
#include "custom_sem.h"
#include "hrt/drv_hrt.h"

#ifdef CONFIG_FS_OCMFS 

struct ocmfs_mountpt_s{
    int ocm_chn;
    sem_t fs_sem;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     ocmfs_open(FAR struct file *filep, const char *relpath,
                          int oflags, mode_t mode);
static int     ocmfs_close(FAR struct file *filep);
static ssize_t ocmfs_read(FAR struct file *filep, char *buffer, size_t buflen);
static ssize_t ocmfs_write(FAR struct file *filep, const char *buffer,
                           size_t buflen);
static off_t   ocmfs_seek(FAR struct file *filep, off_t offset, int whence);
static int     ocmfs_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

static int     ocmfs_sync(FAR struct file *filep);
static int     ocmfs_dup(FAR const struct file *oldp, FAR struct file *newp);

static int     ocmfs_opendir(struct inode *mountpt, const char *relpath,
                             struct fs_dirent_s *dir);
static int     ocmfs_readdir(struct inode *mountpt, struct fs_dirent_s *dir);
static int     ocmfs_rewinddir(struct inode *mountpt, struct fs_dirent_s *dir);

static int     ocmfs_bind(FAR struct inode *blkdriver, const void *data,
                          void **handle);
static int     ocmfs_unbind(void *handle, FAR struct inode **blkdriver);
static int     ocmfs_statfs(struct inode *mountpt, const char *relpath, struct statfs *buf);

static int     ocmfs_unlink(struct inode *mountpt, const char *relpath);
static int     ocmfs_mkdir(struct inode *mountpt, const char *relpath,
                           mode_t mode);
static int     ocmfs_rmdir(struct inode *mountpt, const char *relpath);
static int     ocmfs_rename(struct inode *mountpt, const char *oldrelpath,
                            const char *newrelpath);
static int     ocmfs_stat(struct inode *mountpt, const char *relpath, struct stat *buf);

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/* See fs_mount.c -- this structure is explicitly externed there.
 * We use the old-fashioned kind of initializers so that this will compile
 * with any compiler.
 */

const struct mountpt_operations ocmfs_operations =
{
  ocmfs_open,          /* open */
  ocmfs_close,         /* close */
  ocmfs_read,          /* read */
  ocmfs_write,         /* write */
  ocmfs_seek,          /* seek */
  ocmfs_ioctl,         /* ioctl */

  ocmfs_sync,          /* sync */
  ocmfs_dup,           /* dup */

  ocmfs_opendir,       /* opendir */
  NULL,              /* closedir */
  ocmfs_readdir,       /* readdir */
  ocmfs_rewinddir,     /* rewinddir */

  ocmfs_bind,          /* bind */
  ocmfs_unbind,        /* unbind */
  ocmfs_statfs,        /* statfs */

  ocmfs_unlink,        /* unlinke */
  ocmfs_mkdir,         /* mkdir */
  ocmfs_rmdir,         /* rmdir */
  ocmfs_rename,        /* rename */
  ocmfs_stat           /* stat */
};

static int ocmfs_msg_trans(struct ocmfs_msg *send_msg, struct ocmfs_msg *recv_msg)
{
    int tmp_len = 0;
    int ret = 0;

    tmp_len = sizeof(send_msg->cmd) + sizeof(send_msg->data_len) + send_msg->data_len;

    //align
    tmp_len = OCMFS_ALIGN(tmp_len);

    while(ocm_msg_busy(OCM_CHN_FS, tmp_len)){;}
    ocm_msg_send(OCM_CHN_FS, (uint8_t *)send_msg, tmp_len);

    hrt_abstime start = hrt_absolute_time();
    while(recv_msg != NULL)
    {
        //check timout
        if(hrt_absolute_time() - start > 1000000)
        {
            pilot_warn("recv master msg timeout\n");
            return -1;
        }

        ret = ocm_msg_recv(OCM_CHN_FS, (uint8_t *)recv_msg, sizeof(struct ocmfs_msg));
        if(ret != 0)
            break;
    }   

    return 0;
}

void ocmfs_semtake(struct ocmfs_mountpt_s *fs)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&fs->fs_sem) != 0)
    {
      /* The only case that an error should occur here is if
       * the wait was awakened by a signal.
       */

    }
}

void ocmfs_semgive(struct ocmfs_mountpt_s *fs)
{
   sem_post(&fs->fs_sem);
}

/****************************************************************************
 * Name: ocmfs_mount
 *
 * Desciption: This function is called only when the mountpoint is first
 *   established.  It initializes the mountpoint structure and verifies
 *   that a valid filesystem is provided by the block driver.
 *
 *   The caller should hold the mountpoint semaphore
 *
 ****************************************************************************/
static int ocmfs_mount(struct ocmfs_mountpt_s *fs, bool writeable)
{
    int ret = 0;

    ret = ocm_msg_chn_init(OCM_CHN_FS, "ocmfs");
    if(ret != 0)
    {
        pilot_err("ocmfs mount failed!!\n");
        return -1;
    }

    fs->ocm_chn = OCM_CHN_FS;

    return 0;
}

static int ocmfs_bind(struct inode *blkdriver, const void *data, void **handle)
{
    struct ocmfs_mountpt_s *fs = NULL;
    int ret;

    /* Create an instance of the mountpt state structure */
    fs = (struct ocmfs_mountpt_s *)kzalloc(sizeof(struct ocmfs_mountpt_s));
    if (!fs)
    {
        return -ENOMEM;
    }

    sem_init(&fs->fs_sem, 0, 0);   /* Initialize the semaphore that controls access */

    ret = ocmfs_mount(fs, true);
    if (ret != 0)
    {
        sem_destroy(&fs->fs_sem);
        kfree(fs);
        return ret;
    }

    *handle = (void*)fs;
    ocmfs_semgive(fs);

    return OK;
}

static int ocmfs_open(FAR struct file *filep, const char *relpath, int oflags, mode_t mode)
{
    struct inode         *inode;
    struct ocmfs_mountpt_s *fs;
    struct ocmfs_msg msg = {0};
    struct ocmfs_file_s *ff = NULL;
    int offset = 0;
    int path_len = strlen(relpath)+1;
    int data_len = path_len + sizeof(oflags) + sizeof(mode);
    int ret = 0;

    /* Sanity checks */

    DEBUGASSERT(filep->f_priv == NULL && filep->f_inode != NULL);

    /* Get the mountpoint inode reference from the file structure and the
     * mountpoint private data from the inode structure
     */

    inode = filep->f_inode;
    fs    = inode->i_private;

    DEBUGASSERT(fs != NULL);

    if(data_len > OCMFS_MAX_DATA_LEN)
    {
        pilot_err("data too long: %d\n", data_len);
        return -1;
    }

    ocmfs_semtake(fs);
    ff = (struct ocmfs_file_s *)kzalloc(sizeof(struct ocmfs_file_s));
    if (!ff)
    {
        ret = -ENOMEM;
        goto out_with_semaphore;
    }

    msg.cmd = OCMFS_CMD_OPEN;
    memcpy(&msg.data[offset], relpath, path_len);
    offset += path_len;

    memcpy(&msg.data[offset], &oflags, sizeof(oflags));
    offset += sizeof(oflags);

    memcpy(&msg.data[offset], &mode, sizeof(mode));
    offset += sizeof(mode);
    
    msg.data_len = offset;

    ret = ocmfs_msg_trans(&msg, &msg);
    if(msg.data_len != sizeof(struct ocmfs_file_s) || ret != 0)
    {
        pilot_err("recv err: relpath=%s len=%d ret=%d\n", relpath, msg.data_len, ret);
        ret = -1;
        goto out_with_semaphore;
    }
    else
    {
        memcpy(ff, msg.data, sizeof(struct ocmfs_file_s));
//        pilot_info("fd=%d %s oflags=%x\n", ff->fd, relpath, ff->oflags);
    }

    if(ff->fd < 0)
    {
        kfree(ff);
        ret = -ENOENT;
        goto out_with_semaphore;
    }
    else
    {
        filep->f_priv = ff;
    }

out_with_semaphore:
    ocmfs_semgive(fs);
    return ret;
}

static int ocmfs_close(FAR struct file *filep)
{
    struct inode         *inode;
    struct ocmfs_mountpt_s *fs;
    struct ocmfs_file_s    *ff;
    struct ocmfs_msg msg = {0};
    int offset = 0;
    int ret = 0;

    /* Sanity checks */

    DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);
    
    /* Recover our private data from the struct file instance */

    ff    = filep->f_priv;
    inode = filep->f_inode;
    fs    = inode->i_private;

    DEBUGASSERT(fs != NULL);
    ocmfs_semtake(fs);

    msg.cmd = OCMFS_CMD_CLOSE;

    memcpy(&msg.data[offset], ff, sizeof(struct ocmfs_file_s));
    offset += sizeof(struct ocmfs_file_s);

    msg.data_len = offset;

    ret = ocmfs_msg_trans(&msg, &msg);
    if(msg.data_len != sizeof(ret) || ret != 0)
    {
        pilot_err("recv err: len=%d ret=%d\n", msg.data_len, ret);
        ret = -1;
        goto out_with_semaphore;
    }
    else
    {
        memcpy(&ret, msg.data, sizeof(ret));
        //pilot_warn("fd=%d close ret=%d\n", ff->fd, ret);
    }

    /* Then free the file structure itself. */
    kfree(ff);
    filep->f_priv = NULL;

out_with_semaphore:
    ocmfs_semgive(fs);
    return ret;
}

static ssize_t ocmfs_read(FAR struct file *filep, char *buffer, size_t buflen)
{
    struct inode         *inode;
    struct ocmfs_mountpt_s *fs;
    struct ocmfs_file_s    *ff;
    struct ocmfs_msg msg = {0};
    int offset = 0;
    int ret = 0;

    /* Sanity checks */

    DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);
    
    /* Recover our private data from the struct file instance */

    ff    = filep->f_priv;
    inode = filep->f_inode;
    fs    = inode->i_private;

    DEBUGASSERT(fs != NULL);

    ocmfs_semtake(fs);
    /* Check if the file was opened for write access */
    if ((ff->oflags & O_RDOK) == 0 && ff->oflags != O_RDONLY)
    {
        ret = -EACCES;
        goto out_with_semaphore;
    }

    msg.cmd = OCMFS_CMD_READ;

    memcpy(&msg.data[offset], ff, sizeof(struct ocmfs_file_s));
    offset += sizeof(struct ocmfs_file_s);

    memcpy(&msg.data[offset], &buflen, sizeof(buflen));
    offset += sizeof(buflen);

    msg.data_len = offset;

    ret = ocmfs_msg_trans(&msg, &msg);
    if(ret != 0)
    {
        pilot_err("recv err: len=%d ret=%d\n", msg.data_len, ret);
        ret = -1;
        goto out_with_semaphore;
    }
    else
    {
        //linux return how many bytes were read;
        memcpy(&ret, msg.data, sizeof(ret));
        if(ret <= buflen)
        {
            memcpy(buffer, msg.data + sizeof(ret), ret);
        }
        else
        {
            pilot_err("something wrong during msg transfer:%d/%d\n", ret, buflen);
            ret = -1;
        }
      //pilot_info("fd=%d read ret=%d :\n", ff->fd, ret);
//      for(int i=0; i<ret; i++)
//          pilot_info("%d \n", buffer[i]);
    }

out_with_semaphore:
    ocmfs_semgive(fs);
    return ret;
}

static ssize_t ocmfs_write(FAR struct file *filep, const char *buffer, size_t buflen)
{
    struct inode         *inode;
    struct ocmfs_mountpt_s *fs;
    struct ocmfs_file_s    *ff;
    struct ocmfs_msg msg = {0};
    int offset = 0;
    int ret = 0;

    /* Sanity checks */

    DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);
    
    /* Recover our private data from the struct file instance */

    ff    = filep->f_priv;
    inode = filep->f_inode;
    fs    = inode->i_private;

    DEBUGASSERT(fs != NULL);

    ocmfs_semtake(fs);
    /* Check if the file was opened for write access */
    if ((ff->oflags & O_WROK) == 0)
    {
        ret = -EACCES;
        goto out_with_semaphore;
    }

    msg.cmd = OCMFS_CMD_WRITE;

    memcpy(&msg.data[offset], ff, sizeof(struct ocmfs_file_s));
    offset += sizeof(struct ocmfs_file_s);

    /*prevent writing off the end of the array*/
    if(offset + buflen + sizeof(buflen) > OCMFS_MAX_DATA_LEN)
        buflen = OCMFS_MAX_DATA_LEN - offset - sizeof(buflen);

    memcpy(&msg.data[offset], &buflen, sizeof(buflen));
    offset += sizeof(buflen);

    memcpy(&msg.data[offset], buffer, buflen);
    offset += buflen;

    msg.data_len = offset;

    ret = ocmfs_msg_trans(&msg, &msg);
    if(msg.data_len != sizeof(ret) || ret != 0)
    {
        pilot_err("recv err: fd=%d len=%d ret=%d\n", ff->fd, msg.data_len, ret);
        ret = -1;
        goto out_with_semaphore;
    }
    else
    {
        memcpy(&ret, msg.data, sizeof(ret));
        //pilot_warn("fd=%d write ret=%d sizeof=%d\n", ff->fd, ret, sizeof(struct ocmfs_msg));
    }

out_with_semaphore:
    ocmfs_semgive(fs);
    return ret;
}

static off_t ocmfs_seek(FAR struct file *filep, off_t offset, int whence)
{
    struct inode         *inode;
    struct ocmfs_mountpt_s *fs;
    struct ocmfs_file_s    *ff;
    struct ocmfs_msg msg = {0};
    int cpy_offset = 0;
    int ret = 0;

    /* Sanity checks */

    DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);
    
    /* Recover our private data from the struct file instance */

    ff    = filep->f_priv;
    inode = filep->f_inode;
    fs    = inode->i_private;

    DEBUGASSERT(fs != NULL);

    ocmfs_semtake(fs);
    msg.cmd = OCMFS_CMD_LSEEK;

    memcpy(&msg.data[cpy_offset], ff, sizeof(struct ocmfs_file_s));
    cpy_offset += sizeof(struct ocmfs_file_s);

    memcpy(&msg.data[cpy_offset], &offset, sizeof(offset));
    cpy_offset += sizeof(cpy_offset);

    memcpy(&msg.data[cpy_offset], &whence, sizeof(whence));
    cpy_offset += sizeof(whence);

    msg.data_len = cpy_offset;

    ret = ocmfs_msg_trans(&msg, &msg);
    if(msg.data_len != sizeof(ret) || ret != 0)
    {
        pilot_err("recv err: len=%d ret=%d\n", msg.data_len, ret);
        ret = -1;
        goto out_with_semaphore;
    }
    else
    {
        memcpy(&ret, msg.data, sizeof(ret));
//        pilot_warn("fd=%d lseek ret=%d\n", ff->fd, ret);
    }

    filep->f_pos = ret;

out_with_semaphore:
    ocmfs_semgive(fs);
    return ret;
}

static int ocmfs_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
   return 0; 
}


static int ocmfs_sync(FAR struct file *filep)
{
    struct inode         *inode;
    struct ocmfs_mountpt_s *fs;
    struct ocmfs_file_s    *ff;
    struct ocmfs_msg msg = {0};
    int offset = 0;
    int ret = 0;

    /* Sanity checks */

    DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);
    
    /* Recover our private data from the struct file instance */

    ff    = filep->f_priv;
    inode = filep->f_inode;
    fs    = inode->i_private;

    DEBUGASSERT(fs != NULL);

    ocmfs_semtake(fs);
    msg.cmd = OCMFS_CMD_SYNC;

    memcpy(&msg.data[offset], ff, sizeof(struct ocmfs_file_s));
    offset += sizeof(struct ocmfs_file_s);

    msg.data_len = offset;

    ret = ocmfs_msg_trans(&msg, &msg);
    if(msg.data_len != sizeof(ret) || ret != 0)
    {
        pilot_err("recv err: len=%d ret=%d\n", msg.data_len, ret);
        ret = -1;
        goto out_with_semaphore;
    }
    else
    {
        memcpy(&ret, msg.data, sizeof(ret));
        //pilot_warn("fd=%d sync ret=%d\n", ff->fd, ret);
    }

out_with_semaphore:
    ocmfs_semgive(fs);
    return ret;
}

static int ocmfs_dup(FAR const struct file *oldp, FAR struct file *newp)
{
    pilot_err("unsupport api\n");
    return 0; 
}


/*fill dir->u structure*/
static int ocmfs_opendir(struct inode *mountpt, const char *relpath, struct fs_dirent_s *dir)
{
    struct ocmfs_mountpt_s *fs;
    struct ocmfs_msg msg = {0};
    int offset = 0;
    int path_len = strlen(relpath)+1;
    int data_len = path_len;
    int ret = 0;

    /* Sanity checks */

    DEBUGASSERT(mountpt != NULL);

    /* Get the mountpoint inode reference from the file structure and the
     * mountpoint private data from the inode structure
     */

    fs    = mountpt->i_private;

    DEBUGASSERT(fs != NULL);

    if(data_len > OCMFS_MAX_DATA_LEN)
    {
        pilot_err("data too long: %d\n", data_len);
        return -1;
    }

    ocmfs_semtake(fs);
    msg.cmd = OCMFS_CMD_OPENDIR;
    memcpy(&msg.data[offset], relpath, path_len);
    offset += path_len;
    
    msg.data_len = offset;

    ret = ocmfs_msg_trans(&msg, &msg);
    if(msg.data_len != sizeof(dir->u.ocmfs) || ret != 0)
    {
        pilot_err("recv err: len=%d ret=%d\n", msg.data_len, ret);
        ret = -1;
        goto out_with_semaphore;
    }
    else
    {
        memcpy(&dir->u.ocmfs, msg.data, sizeof(dir->u.ocmfs));
        //pilot_info("dir ptr in linux=0x%x\n", (int)dir->u.ocmfs.dir);
    }

    if(dir->u.ocmfs.dir == NULL)
       ret = -ENOTDIR;

out_with_semaphore:
    ocmfs_semgive(fs);
    return ret;
}

/*fill dir->fd_dir structure*/
static int ocmfs_readdir(struct inode *mountpt, struct fs_dirent_s *dir)
{
    struct ocmfs_mountpt_s *fs;
    struct ocmfs_msg msg = {0};
    int offset = 0;
    int ret = 0;

    /* Sanity checks */

    DEBUGASSERT(mountpt != NULL);

    /* Get the mountpoint inode reference from the file structure and the
     * mountpoint private data from the inode structure
     */

    fs    = mountpt->i_private;

    DEBUGASSERT(fs != NULL);

    ocmfs_semtake(fs);
    msg.cmd = OCMFS_CMD_READDIR;
    memcpy(&msg.data[offset], &dir->u.ocmfs, sizeof(dir->u.ocmfs));
    offset += sizeof(dir->u.ocmfs);
    
    msg.data_len = offset;

    ret = ocmfs_msg_trans(&msg, &msg);
    if(ret != 0)
    {
        pilot_err("recv err: len=%d ret=%d\n", msg.data_len, ret);
        ret = -1;
        goto out_with_semaphore;
    }
    else
    {
        char *p_data = msg.data;

        memcpy(&dir->fd_dir.d_type, p_data, sizeof(dir->fd_dir.d_type));
        p_data += sizeof(dir->fd_dir.d_type);

        if(strlen(p_data) > NAME_MAX)
        {
            ret = -ENOENT; 
            goto out_with_semaphore;
        }

        memcpy(dir->fd_dir.d_name, p_data, strlen(p_data)+1);

       // pilot_info("type=%d d_name=%s\n", dir->fd_dir.d_type, dir->fd_dir.d_name);
    }

    if(dir->fd_dir.d_type == 0xff)
    {
        ret = -ENOENT;
        goto out_with_semaphore;
    }

out_with_semaphore:
    ocmfs_semgive(fs);
    return ret;
}

static int ocmfs_rewinddir(struct inode *mountpt, struct fs_dirent_s *dir)
{
    pilot_err("unsupport api\n");
    return 0; 
}

static int ocmfs_unbind(void *handle, FAR struct inode **blkdriver)
{
    struct ocmfs_mountpt_s *fs = (struct ocmfs_mountpt_s*)handle;

    if (!fs)
    {
        return -EINVAL;
    }
   
    sem_destroy(&fs->fs_sem);   /* destory the semaphore that controls access */
    kfree(fs);

    return 0; 
}

static int ocmfs_statfs(struct inode *mountpt, const char *relpath, struct statfs *buf)
{
    struct ocmfs_mountpt_s *fs;
    struct ocmfs_msg msg = {0};
    int offset = 0;
    int path_len = strlen(relpath)+1;
    int data_len = path_len;
    int ret = 0;

    /* Sanity checks */

    DEBUGASSERT(mountpt != NULL);

    /* Get the mountpoint inode reference from the file structure and the
     * mountpoint private data from the inode structure
     */

    fs    = mountpt->i_private;

    DEBUGASSERT(fs != NULL);

    if(data_len > OCMFS_MAX_DATA_LEN)
    {
        pilot_err("data too long: %d\n", data_len);
        return -1;
    }

    ocmfs_semtake(fs);
    msg.cmd = OCMFS_CMD_STATFS;
    memcpy(&msg.data[offset], relpath, path_len);
    offset += path_len;
   
    msg.data_len = offset;

    ret = ocmfs_msg_trans(&msg, &msg);
    if(msg.data_len != sizeof(struct ocmfs_statfs)+sizeof(ret) || ret != 0)
    {
        pilot_err("recv err: len=%d ret=%d\n", msg.data_len, ret);
        ret = -1;
        goto out_with_semaphore;
    }
    else
    {
        offset = 0;

        ret = *(int *)(msg.data + offset);
        offset += sizeof(ret);

        memcpy(buf, msg.data + offset, sizeof(struct ocmfs_statfs));
        //pilot_info("ret=%d st_size=%d st_blksize=%d st_blocks=%d\n", ret, buf->st_size, buf->st_blksize, buf->st_blocks);
    }

out_with_semaphore:
    ocmfs_semgive(fs);
    return ret;
}


static int ocmfs_unlink(struct inode *mountpt, const char *relpath)
{
    pilot_err("unsupport api\n");
    return 0; 
}

static int ocmfs_mkdir(struct inode *mountpt, const char *relpath, mode_t mode)
{
    struct ocmfs_mountpt_s *fs;
    struct ocmfs_msg msg = {0};
    int offset = 0;
    int path_len = strlen(relpath)+1;
    int data_len = path_len + sizeof(mode);
    int ret = 0;

    /* Sanity checks */

    DEBUGASSERT(mountpt != NULL);

    /* Get the mountpoint inode reference from the file structure and the
     * mountpoint private data from the inode structure
     */

    fs    = mountpt->i_private;

    DEBUGASSERT(fs != NULL);

    if(data_len > OCMFS_MAX_DATA_LEN)
    {
        pilot_err("data too long: %d\n", data_len);
        return -1;
    }

    ocmfs_semtake(fs);
    msg.cmd = OCMFS_CMD_MKDIR;
    memcpy(&msg.data[offset], relpath, path_len);
    offset += path_len;
   
    memcpy(&msg.data[offset], &mode, sizeof(mode));
    offset += sizeof(mode);
   
    msg.data_len = offset;

    ret = ocmfs_msg_trans(&msg, &msg);
    if(msg.data_len != sizeof(ret) || ret != 0)
    {
        pilot_err("recv err: len=%d ret=%d\n", msg.data_len, ret);
        ret = -1;
        goto out_with_semaphore;
    }
    else
    {
        ret = *(int *)(msg.data);

//        pilot_warn("mkdir ret=%d\n", ret);
    }

out_with_semaphore:
    ocmfs_semgive(fs);
    return ret;
}

static int ocmfs_rmdir(struct inode *mountpt, const char *relpath)
{
    struct ocmfs_mountpt_s *fs;
    struct ocmfs_msg msg = {0};
    int offset = 0;
    int path_len = strlen(relpath)+1;
    int data_len = path_len;
    int ret = 0;

    /* Sanity checks */

    DEBUGASSERT(mountpt != NULL);

    /* Get the mountpoint inode reference from the file structure and the
     * mountpoint private data from the inode structure
     */

    fs    = mountpt->i_private;

    DEBUGASSERT(fs != NULL);

    if(data_len > OCMFS_MAX_DATA_LEN)
    {
        pilot_err("data too long: %d\n", data_len);
        return -1;
    }

    ocmfs_semtake(fs);
    msg.cmd = OCMFS_CMD_RMDIR;
    memcpy(&msg.data[offset], relpath, path_len);
    offset += path_len;
   
    msg.data_len = offset;

    ret = ocmfs_msg_trans(&msg, &msg);
    if(msg.data_len != sizeof(ret) || ret != 0)
    {
        pilot_err("recv err: len=%d ret=%d\n", msg.data_len, ret);
        ret = -1;
        goto out_with_semaphore;
    }
    else
    {
        ret = *(int *)(msg.data);
        //pilot_warn("rmdir ret=%d\n", ret);
    }

out_with_semaphore:
    ocmfs_semgive(fs);
    return ret;
}

static int ocmfs_rename(struct inode *mountpt, const char *oldrelpath, const char *newrelpath)
{
    struct ocmfs_mountpt_s *fs;
    struct ocmfs_msg msg = {0};
    int offset = 0;
    int old_path_len = strlen(oldrelpath)+1;
    int new_path_len = strlen(newrelpath)+1;
    int data_len = old_path_len + new_path_len;
    int ret = 0;

    /* Sanity checks */

    DEBUGASSERT(mountpt != NULL);

    /* Get the mountpoint inode reference from the file structure and the
     * mountpoint private data from the inode structure
     */

    fs    = mountpt->i_private;

    DEBUGASSERT(fs != NULL);

    if(data_len > OCMFS_MAX_DATA_LEN)
    {
        pilot_err("data too long: %d\n", data_len);
        return -1;
    }

    ocmfs_semtake(fs);
    msg.cmd = OCMFS_CMD_RENAME;
    memcpy(&msg.data[offset], oldrelpath, old_path_len);
    offset += old_path_len;
   
    memcpy(&msg.data[offset], newrelpath, new_path_len);
    offset += new_path_len;
   
    msg.data_len = offset;

    ret = ocmfs_msg_trans(&msg, &msg);
    if(msg.data_len != sizeof(ret) || ret != 0)
    {
        pilot_err("recv err: len=%d ret=%d\n", msg.data_len, ret);
        ret = -1;
        goto out_with_semaphore;
    }
    else
    {
        ret = *(int *)(msg.data);
        //pilot_warn("rename ret=%d\n", ret);
    }

out_with_semaphore:
    ocmfs_semgive(fs);
    return ret;
}

static int ocmfs_stat(struct inode *mountpt, const char *relpath, struct stat *buf)
{
    struct ocmfs_mountpt_s *fs;
    struct ocmfs_msg msg = {0};
    int offset = 0;
    int path_len = strlen(relpath)+1;
    int data_len = path_len;
    int ret = 0;

    /* Sanity checks */

    DEBUGASSERT(mountpt != NULL);

    /* Get the mountpoint inode reference from the file structure and the
     * mountpoint private data from the inode structure
     */

    fs    = mountpt->i_private;

    DEBUGASSERT(fs != NULL);

    if(data_len > OCMFS_MAX_DATA_LEN)
    {
        pilot_err("data too long: %d\n", data_len);
        return -1;
    }

    ocmfs_semtake(fs);
    msg.cmd = OCMFS_CMD_STAT;
    memcpy(&msg.data[offset], relpath, path_len);
    offset += path_len;
   
    msg.data_len = offset;

    ret = ocmfs_msg_trans(&msg, &msg);
    if(msg.data_len != sizeof(struct ocmfs_stat)+sizeof(ret) || ret != 0)
    {
        pilot_err("recv err: len=%d ret=%d\n", msg.data_len, ret);
        ret = -1;
        goto out_with_semaphore;
    }
    else
    {
        offset = 0;

        ret = *(int *)(msg.data + offset);
        offset += sizeof(ret);

        memcpy(buf, msg.data + offset, sizeof(struct ocmfs_stat));
        //pilot_info("ret=%d st_size=%d st_blksize=%d st_blocks=%d\n", ret, buf->st_size, buf->st_blksize, buf->st_blocks);
    }

out_with_semaphore:
    ocmfs_semgive(fs);
    return ret;
}



#endif
