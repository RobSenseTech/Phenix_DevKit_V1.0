#ifndef __OCMFS__
#define __OCMFS__

#include "ocm/ocm.h"

#define OCMFS_ALIGN(l)     (l % OCM_ALIGN_BYTES != 0)?\
                           (l / OCM_ALIGN_BYTES + 1) * OCM_ALIGN_BYTES :\
                            l

struct ocmfs_file_s{
    int fd;                 //file fd in linux
    int oflags;              /* Flags provided when file was opened */
};

#define OCMFS_MAX_DATA_LEN  540 
#define OCMFS_MAX_ARGS      128
#define OCMFS_SYNC          0xff
#define OCMFS_MSG_HEAD      "OCM_STA" //8 bytes, 4 bytes align
#define OCMFS_MSG_HEAD_LEN  8 
#if 1
struct ocmfs_msg{
    int cmd;
    int data_len;
    char data[OCMFS_MAX_DATA_LEN];
};
#else
struct ocmfs_cmd{
    char msg_head[OCMFS_MSG_HEAD_LEN];
    int id;
    int args_len;
    char args[OCMFS_MAX_ARGS];
};

struct ocmfs_msg{
    struct ocmfs_cmd cmd;
    int data_len;
    char *data;
};
#endif

enum ocmfs_cmd_id{
    OCMFS_CMD_OPEN,
    OCMFS_CMD_CLOSE,
    OCMFS_CMD_READ,
    OCMFS_CMD_WRITE,
    OCMFS_CMD_LSEEK,
    OCMFS_CMD_IOCTL,
    OCMFS_CMD_SYNC,
    OCMFS_CMD_DUP,
    OCMFS_CMD_OPENDIR,
    OCMFS_CMD_READDIR,
    OCMFS_CMD_REWINDDIR,
    OCMFS_CMD_STATFS,
    OCMFS_CMD_MKDIR,
    OCMFS_CMD_RMDIR,
    OCMFS_CMD_RENAME,
    OCMFS_CMD_STAT,
    OCMFS_CMD_NR,
};

struct ocmfs_stat{
    //stat structure is different with linux, be careful with data type!!
    unsigned int st_mode;    
    int32_t      st_size;    
    int16_t      st_blksize; 
    uint32_t     st_blocks;  
    uint32_t     atime;   
    uint32_t     mtime;   
    uint32_t     ctime;   
};

struct ocmfs_statfs
{
  uint32_t f_type;     /* Type of filesystem (see definitions above) */
  int32_t  f_namelen;  /* Maximum length of filenames */
  int32_t  f_bsize;    /* Optimal block size for transfers */
  int32_t  f_blocks;   /* Total data blocks in the file system of this size */
  int32_t  f_bfree;    /* Free blocks in the file system */
  int32_t  f_bavail;   /* Free blocks avail to non-superuser */
  int32_t  f_files;    /* Total file nodes in the file system */
  int32_t  f_ffree;    /* Free file nodes in the file system */
};

struct fs_ocmfsdir_s
{
    void *dir;
};

#endif
