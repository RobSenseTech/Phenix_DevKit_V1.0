#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/vfs.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <string.h>
#include "ocm/ocm_internal.h"
#include "ocm/ocm.h"
#include "ocmfs.h"


#define SD_PATH     "/mnt/"

#define PAGE_SIZE ((size_t)getpagesize())

struct ocmfs_proc{
    int (*proc)(struct ocmfs_msg *);
};

uint32_t ocm_get_vir_addr(uint32_t phy_addr)
{
	int mem_fd;
    uint32_t vir_addr;

	mem_fd = open("/dev/mem", O_RDWR|O_SYNC);
	if (mem_fd < 0) {
		pilot_err("open(/dev/mem) failed (%d)\n", errno);
		return -1;
	}

    //mmap ocm physical address, 64k == 16 pages
	vir_addr = (uint32_t)mmap(NULL, PAGE_SIZE*16, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, phy_addr);
	if (vir_addr == MAP_FAILED) {
		pilot_err("mmap64(0x4@0x%x) failed (%d)\n", (uint32_t)phy_addr, errno);
		return -1;
	}

    return vir_addr;
}

static int ocmfs_open_proc(struct ocmfs_msg *msg)
{
    char *relpath;
    int oflags;
    unsigned int mode;
    char *p_data = msg->data;
    struct ocmfs_file_s ff;
    char *path = NULL;
    struct ocmfs_msg send_msg;
    int send_len = 0;

    relpath = p_data;    
    p_data += strlen(relpath) + 1;

    oflags = *(int *)p_data;
    p_data += sizeof(oflags);

    mode = *(int *)p_data;

    asprintf(&path, "%s%s", SD_PATH, relpath);

    ff.fd = open(path, oflags, mode);
    ff.oflags = oflags;

    //pilot_info("linux fd=%d\n", ff.fd);

    send_msg.cmd = OCMFS_CMD_OPEN;
    send_msg.data_len = sizeof(ff);
    memcpy(send_msg.data, &ff, sizeof(ff));

    send_len = sizeof(send_msg.cmd) + sizeof(send_msg.data_len) + send_msg.data_len;
    //align
    send_len = OCMFS_ALIGN(send_len);
    while(ocm_msg_busy(OCM_CHN_FS, send_len)){;}
    ocm_msg_send(OCM_CHN_FS, (uint8_t *)&send_msg, send_len);

    free(path);

    return 0;
}

static int ocmfs_write_proc(struct ocmfs_msg *msg)
{
    char *p_data = msg->data;
    size_t buflen = 0;
    struct ocmfs_file_s ff;
    int ret = 0;
    struct ocmfs_msg send_msg;
    int send_len = 0;

    memcpy(&ff, p_data, sizeof(struct ocmfs_file_s));
    p_data += sizeof(struct ocmfs_file_s);

    memcpy(&buflen, p_data, sizeof(buflen));
    p_data += sizeof(buflen);

    ret = write(ff.fd, p_data, buflen);
    if(ret < 0)
        pilot_err("linux fd=%d write err len=%d ret=%d errno=%d\n", ff.fd, buflen, ret, errno);
    else
        fsync(ff.fd);

    send_msg.cmd = OCMFS_CMD_WRITE;
    send_msg.data_len = sizeof(ret);
    memcpy(send_msg.data, &ret, sizeof(ret));

    send_len = sizeof(send_msg.cmd) + sizeof(send_msg.data_len) + send_msg.data_len;
    //align
    send_len = OCMFS_ALIGN(send_len);
    while(ocm_msg_busy(OCM_CHN_FS, send_len)){;}
    ocm_msg_send(OCM_CHN_FS, (uint8_t *)&send_msg, send_len);

    return 0;
}

static int ocmfs_read_proc(struct ocmfs_msg *msg)
{
    char *p_data = msg->data;
    size_t buflen = 0;
    struct ocmfs_file_s ff;
    int ret = 0;
    struct ocmfs_msg send_msg;
    int send_len = 0;

    memcpy(&ff, p_data, sizeof(struct ocmfs_file_s));
    p_data += sizeof(struct ocmfs_file_s);

    memcpy(&buflen, p_data, sizeof(buflen));
    p_data += sizeof(buflen);

    send_msg.cmd = OCMFS_CMD_READ;

    /*prevent writing off the end of the array*/
    if(sizeof(ret) + buflen > OCMFS_MAX_DATA_LEN)
        buflen = OCMFS_MAX_DATA_LEN - sizeof(ret) - 4;

    if(buflen > 0)
    {
        //reply bytes read, slave should make sure that arry never overflow
        ret = read(ff.fd, send_msg.data + sizeof(ret), buflen);
    }
    //pilot_info("linux fd=%d read len=%d ret=%d\n", ff.fd, buflen, ret);

    //reply how many bytes read
    memcpy(send_msg.data, &ret, sizeof(ret));

    send_msg.data_len = sizeof(ret) + ret;

    send_len = sizeof(send_msg.cmd) + sizeof(send_msg.data_len) + send_msg.data_len;
    //align
    send_len = OCMFS_ALIGN(send_len);
    while(ocm_msg_busy(OCM_CHN_FS, send_len)){;}
    ocm_msg_send(OCM_CHN_FS, (uint8_t *)&send_msg, send_len);

    return 0;
}

static int ocmfs_lseek_proc(struct ocmfs_msg *msg)
{
    char *p_data = msg->data;
    off_t offset = 0;
    int whence = 0;
    struct ocmfs_file_s ff;
    int ret = 0;
    struct ocmfs_msg send_msg;
    int send_len = 0;

    memcpy(&ff, p_data, sizeof(struct ocmfs_file_s));
    p_data += sizeof(struct ocmfs_file_s);

    memcpy(&offset, p_data, sizeof(offset));
    p_data += sizeof(offset);

    memcpy(&whence, p_data, sizeof(whence));
    p_data += sizeof(whence);

    ret = lseek(ff.fd, offset, whence);
//    pilot_info("linux fd=%d lseek ret=%d\n", ff.fd, ret);

    send_msg.cmd = OCMFS_CMD_LSEEK;
    send_msg.data_len = sizeof(ret);
    memcpy(send_msg.data, &ret, sizeof(ret));

    send_len = sizeof(send_msg.cmd) + sizeof(send_msg.data_len) + send_msg.data_len;
    //align
    send_len = OCMFS_ALIGN(send_len);
    while(ocm_msg_busy(OCM_CHN_FS, send_len)){;}
    ocm_msg_send(OCM_CHN_FS, (uint8_t *)&send_msg, send_len);

    return 0;
}

static int ocmfs_close_proc(struct ocmfs_msg *msg)
{
    char *p_data = msg->data;
    struct ocmfs_file_s ff;
    int ret = 0;
    struct ocmfs_msg send_msg;
    int send_len = 0;

    memcpy(&ff, p_data, sizeof(struct ocmfs_file_s));
    p_data += sizeof(struct ocmfs_file_s);

    ret = close(ff.fd);
//    pilot_info("linux fd=%d lseek ret=%d\n", ff.fd, ret);

    send_msg.cmd = OCMFS_CMD_CLOSE;
    send_msg.data_len = sizeof(ret);
    memcpy(send_msg.data, &ret, sizeof(ret));

    send_len = sizeof(send_msg.cmd) + sizeof(send_msg.data_len) + send_msg.data_len;
    //align
    send_len = OCMFS_ALIGN(send_len);
    while(ocm_msg_busy(OCM_CHN_FS, send_len)){;}
    ocm_msg_send(OCM_CHN_FS, (uint8_t *)&send_msg, send_len);

    return 0;
}

static int ocmfs_stat_proc(struct ocmfs_msg *msg)
{
    char *relpath;
    char *p_data = msg->data;
    char *path = NULL;
    struct ocmfs_msg send_msg;
    int send_len = 0;
    struct stat st = {0};
    struct ocmfs_stat ocmfs_st = {0};
    int offset = 0;
    int ret = 0;

    relpath = p_data;    
    p_data += strlen(relpath) + 1;

    asprintf(&path, "%s%s", SD_PATH, relpath);

    ret = stat(path, &st);

    //structure transform
    ocmfs_st.st_mode = st.st_mode;    
    ocmfs_st.st_size = st.st_size;    
    ocmfs_st.st_blksize = st.st_blksize; 
    ocmfs_st.st_blocks = st.st_blocks;  
    ocmfs_st.atime = st.st_atime;
    ocmfs_st.mtime = st.st_mtime;
    ocmfs_st.ctime = st.st_ctime;
    //pilot_warn("st_size=%d st_blksize=%d st_blocks=%d\n", ocmfs_st.st_size, ocmfs_st.st_blksize, ocmfs_st.st_blocks);

    send_msg.cmd = OCMFS_CMD_STAT;
    memcpy(send_msg.data, &ret, sizeof(ret));
    offset += sizeof(ret);
    memcpy(send_msg.data + offset, &ocmfs_st, sizeof(ocmfs_st));
    offset += sizeof(ocmfs_st);

    send_msg.data_len = offset;

    send_len = sizeof(send_msg.cmd) + sizeof(send_msg.data_len) + send_msg.data_len;
    //align
    send_len = OCMFS_ALIGN(send_len);
    while(ocm_msg_busy(OCM_CHN_FS, send_len)){;}
    ocm_msg_send(OCM_CHN_FS, (uint8_t *)&send_msg, send_len);

    free(path);

    return 0;
}

static int ocmfs_sync_proc(struct ocmfs_msg *msg)
{
    char *p_data = msg->data;
    struct ocmfs_file_s ff;
    int ret = 0;
    struct ocmfs_msg send_msg;
    int send_len = 0;

    memcpy(&ff, p_data, sizeof(struct ocmfs_file_s));
    p_data += sizeof(struct ocmfs_file_s);

    ret = fsync(ff.fd);
//    pilot_info("linux fd=%d sync ret=%d\n", ff.fd, ret);

    send_msg.cmd = OCMFS_CMD_SYNC;
    send_msg.data_len = sizeof(ret);
    memcpy(send_msg.data, &ret, sizeof(ret));

    send_len = sizeof(send_msg.cmd) + sizeof(send_msg.data_len) + send_msg.data_len;
    //align
    send_len = OCMFS_ALIGN(send_len);
    while(ocm_msg_busy(OCM_CHN_FS, send_len)){;}
    ocm_msg_send(OCM_CHN_FS, (uint8_t *)&send_msg, send_len);

    return 0;
}

static int ocmfs_statfs_proc(struct ocmfs_msg *msg)
{
    char *relpath;
    char *p_data = msg->data;
    char *path = NULL;
    struct ocmfs_msg send_msg;
    int send_len = 0;
    struct statfs stfs = {0};
    struct ocmfs_statfs ocmfs_stfs = {0};
    int offset = 0;
    int ret = 0;

    relpath = p_data;    
    p_data += strlen(relpath) + 1;

    asprintf(&path, "%s%s", SD_PATH, relpath);

    ret = statfs(path, &stfs);

    //structure transform
    ocmfs_stfs.f_type       = stfs.f_type;     
    ocmfs_stfs.f_namelen    = stfs.f_namelen;  
    ocmfs_stfs.f_bsize      = stfs.f_bsize;   
    ocmfs_stfs.f_blocks     = stfs.f_blocks;   
    ocmfs_stfs.f_bfree      = stfs.f_bfree;  
    ocmfs_stfs.f_bavail     = stfs.f_bavail; 
    ocmfs_stfs.f_files      = stfs.f_files;  
    ocmfs_stfs.f_ffree      = stfs.f_ffree;    
    //pilot_warn("bsize=%d f_bfree=%d f_blocks=%d\n", ocmfs_stfs.f_bsize, ocmfs_stfs.f_bfree, ocmfs_stfs.f_blocks);

    send_msg.cmd = OCMFS_CMD_STATFS;
    memcpy(send_msg.data, &ret, sizeof(ret));
    offset += sizeof(ret);
    memcpy(send_msg.data + offset, &ocmfs_stfs, sizeof(ocmfs_stfs));
    offset += sizeof(ocmfs_stfs);

    send_msg.data_len = offset;

    send_len = sizeof(send_msg.cmd) + sizeof(send_msg.data_len) + send_msg.data_len;
    //align
    send_len = OCMFS_ALIGN(send_len);
    while(ocm_msg_busy(OCM_CHN_FS, send_len)){;}
    ocm_msg_send(OCM_CHN_FS, (uint8_t *)&send_msg, send_len);

    free(path);

    return 0;
}

static int ocmfs_mkdir_proc(struct ocmfs_msg *msg)
{
    char *relpath;
    char *p_data = msg->data;
    char *path = NULL;
    struct ocmfs_msg send_msg;
    int send_len = 0;
    int ret = 0;
    unsigned int mode = 0;

    relpath = p_data;    
    p_data += strlen(relpath) + 1;

    mode = *(int *)p_data;

    asprintf(&path, "%s%s", SD_PATH, relpath);

    ret = mkdir(path, mode);

    send_msg.cmd = OCMFS_CMD_MKDIR;
    memcpy(send_msg.data, &ret, sizeof(ret));
    send_msg.data_len = sizeof(ret);

    send_len = sizeof(send_msg.cmd) + sizeof(send_msg.data_len) + send_msg.data_len;
    //align
    send_len = OCMFS_ALIGN(send_len);
    while(ocm_msg_busy(OCM_CHN_FS, send_len)){;}
    ocm_msg_send(OCM_CHN_FS, (uint8_t *)&send_msg, send_len);

    free(path);

    return 0;
}

static int ocmfs_rmdir_proc(struct ocmfs_msg *msg)
{
    char *relpath;
    char *p_data = msg->data;
    char *path = NULL;
    struct ocmfs_msg send_msg;
    int send_len = 0;
    int ret = 0;

    relpath = p_data;    
    p_data += strlen(relpath) + 1;

    asprintf(&path, "%s%s", SD_PATH, relpath);

    ret = rmdir(path);

    //pilot_info("rmdir %s\n", path);

    send_msg.cmd = OCMFS_CMD_RMDIR;
    memcpy(send_msg.data, &ret, sizeof(ret));
    send_msg.data_len = sizeof(ret);

    send_len = sizeof(send_msg.cmd) + sizeof(send_msg.data_len) + send_msg.data_len;
    //align
    send_len = OCMFS_ALIGN(send_len);
    while(ocm_msg_busy(OCM_CHN_FS, send_len)){;}
    ocm_msg_send(OCM_CHN_FS, (uint8_t *)&send_msg, send_len);

    free(path);

    return 0;

}

static int ocmfs_rename_proc(struct ocmfs_msg *msg)
{
    char *oldrelpath = NULL;
    char *newrelpath = NULL;
    char *p_data = msg->data;
    char *oldpath = NULL;
    char *newpath = NULL;
    struct ocmfs_msg send_msg;
    int send_len = 0;
    int ret = 0;

    oldrelpath = p_data;    
    p_data += strlen(oldrelpath) + 1;

    newrelpath = p_data;
    p_data += strlen(newrelpath) + 1;

    asprintf(&oldpath, "%s%s", SD_PATH, oldrelpath);
    asprintf(&newpath, "%s%s", SD_PATH, newrelpath);

    ret = rename(oldpath, newpath);

    //pilot_info("old:%s new:%s\n", oldpath, newpath);

    send_msg.cmd = OCMFS_CMD_RENAME;
    memcpy(send_msg.data, &ret, sizeof(ret));
    send_msg.data_len = sizeof(ret);

    send_len = sizeof(send_msg.cmd) + sizeof(send_msg.data_len) + send_msg.data_len;
    //align
    send_len = OCMFS_ALIGN(send_len);
    while(ocm_msg_busy(OCM_CHN_FS, send_len)){;}
    ocm_msg_send(OCM_CHN_FS, (uint8_t *)&send_msg, send_len);

    free(oldpath);
    free(newpath);

    return 0;
}

static int ocmfs_opendir_proc(struct ocmfs_msg *msg)
{
    char *relpath;
    char *p_data = msg->data;
    char *path = NULL;
    struct ocmfs_msg send_msg;
    int send_len = 0;
    DIR *dir = NULL;
    struct fs_ocmfsdir_s ocmfsdir;

    relpath = p_data;    
    p_data += strlen(relpath) + 1;

    asprintf(&path, "%s%s", SD_PATH, relpath);

    dir = opendir(path);
    ocmfsdir.dir = (void *)dir;

    send_msg.cmd = OCMFS_CMD_OPENDIR;
    send_msg.data_len = sizeof(ocmfsdir);
    memcpy(send_msg.data, &ocmfsdir, sizeof(ocmfsdir));

    send_len = sizeof(send_msg.cmd) + sizeof(send_msg.data_len) + send_msg.data_len;
    //align
    send_len = OCMFS_ALIGN(send_len);
    while(ocm_msg_busy(OCM_CHN_FS, send_len)){;}
    ocm_msg_send(OCM_CHN_FS, (uint8_t *)&send_msg, send_len);

    //pilot_info("linux dir=0x%x\n", dir);

    free(path);

    return 0;
}

static int ocmfs_readdir_proc(struct ocmfs_msg *msg)
{
    char *p_data = msg->data;
    struct fs_ocmfsdir_s ocmfsdir;
    struct dirent *p_dir = NULL;
    uint8_t d_type;             /* type of file */
    char *p_name = NULL;
    struct ocmfs_msg send_msg;
    int send_len = 0;

    memcpy(&ocmfsdir, p_data, sizeof(struct fs_ocmfsdir_s));
    p_data += sizeof(struct fs_ocmfsdir_s);

    p_dir = readdir(ocmfsdir.dir);
    if(p_dir != NULL)
    {
        d_type = p_dir->d_type;
        p_name = p_dir->d_name; 
    }
    else
    {
        //no more dirctory
        d_type = 0xff;
        p_name = "no more"; 
    }

    send_msg.cmd = OCMFS_CMD_READDIR;
    send_msg.data_len = sizeof(d_type) + strlen(p_name) + 1;
    memcpy(send_msg.data, &d_type, sizeof(d_type));
    memcpy(send_msg.data + sizeof(d_type), p_name, strlen(p_name) + 1);

    send_len = sizeof(send_msg.cmd) + sizeof(send_msg.data_len) + send_msg.data_len;
    //align
    send_len = OCMFS_ALIGN(send_len);
    while(ocm_msg_busy(OCM_CHN_FS, send_len)){;}
    ocm_msg_send(OCM_CHN_FS, (uint8_t *)&send_msg, send_len);
//    pilot_warn("type=%d name=%s\n", d_type, p_name);

    return 0;
}

static struct ocmfs_proc process[OCMFS_CMD_NR] = {
    {ocmfs_open_proc},
    {ocmfs_close_proc},
    {ocmfs_read_proc},
    {ocmfs_write_proc},
    {ocmfs_lseek_proc},
    {NULL},
    {ocmfs_sync_proc},
    {NULL},
    {ocmfs_opendir_proc},
    {ocmfs_readdir_proc},
    {NULL},
    {ocmfs_statfs_proc},
    {ocmfs_mkdir_proc},
    {ocmfs_rmdir_proc},
    {ocmfs_rename_proc},
    {ocmfs_stat_proc},
   
};


int main()
{
    int ret = 0;
    struct ocmfs_msg recv_msg = {0};

    ret = ocm_msg_init();
    if(ret != 0)
    {
        pilot_err("ocm msg init err!!\n");
        return -1;
    }

    ocm_msg_chn_init(OCM_CHN_FS, "ocmfs");

    while(1)
    {
        ret = ocm_msg_recv(OCM_CHN_FS, (uint8_t *)&recv_msg, sizeof(struct ocmfs_msg));
        if(ret != 0)
        {
            process[recv_msg.cmd].proc(&recv_msg);
        }

        usleep(10000);
    }

    return 0;
}



