#ifndef DRIVER_H
#define DRIVER_H

#ifdef __cplusplus
extern "C"{
#endif

#include "FreeRTOS_Print.h"
#include <string.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "common_list.h"
#include "xscugic.h"
#include "ioctl.h"
#include "gic/zynq_gic.h"
#include "hrt/drv_hrt.h"
#include "iic/iic.h"
#include "irq.h"
#include "gpio/gpio_init_api.h"
#include "spi/spi_backend.h"
#include "uart/zynq_uart.h"

#include "error.h"

#define MAX_DRIVER_NAME_LEN 40

#define POLLIN       (0x01)  /* NuttX does not make priority distinctions */
#define POLLRDNORM   (0x01)
#define POLLRDBAND   (0x01)
#define POLLPRI      (0x01)

#define POLLOUT      (0x02)  /* NuttX does not make priority distinctions */
#define POLLWRNORM   (0x02)
#define POLLWRBAND   (0x02)

#define POLLERR      (0x04)
#define POLLHUP      (0x08)
#define POLLNVAL     (0x10)

#if 1
/* open flag settings for open() (and related APIs) */

#define O_RDONLY    (1 << 0)        /* Open for read access (only) */
#define O_RDOK      O_RDONLY        /* Read access is permitted (non-standard) */
#define O_WRONLY    (1 << 1)        /* Open for write access (only) */
#define O_WROK      O_WRONLY        /* Write access is permitted (non-standard) */
#define O_RDWR      (O_RDOK|O_WROK) /* Open for both read & write access */
#define O_CREAT     (1 << 2)        /* Create file/sem/mq object */
#define O_EXCL      (1 << 3)        /* Name must not exist when opened  */
#define O_APPEND    (1 << 4)        /* Keep contents, append to end */
#define O_TRUNC     (1 << 5)        /* Delete contents */
#define O_NONBLOCK  (1 << 6)        /* Don't wait for data */
#define O_NDELAY    O_NONBLOCK      /* Synonym for O_NONBLOCK */
#define O_SYNC      (1 << 7)        /* Synchronize output on write */
#define O_DSYNC     O_SYNC          /* Equivalent to OSYNC in NuttX */
#define O_BINARY    (1 << 8)        /* Open the file in binary (untranslated) mode. */
#endif

/* Values for seeking */

#define SEEK_SET    0  /* From the start of the file */
#define SEEK_CUR    1  /* From the current file offset */
#define SEEK_END    2  /* From the end of the file */

typedef uint8_t pollevent_t;

struct pollfd
{
    int                 fd;       /* The descriptor being polled */
    SemaphoreHandle_t   sem;      /* Pointer to semaphore used to post output event */
    pollevent_t         events;   /* The input event flags */
    pollevent_t         revents;  /* The output event flags */
    void                *priv;     /* For use by drivers */
};

typedef struct inode
{
    uint16_t    iFlags;
    void *      pvPrivate;
}Inode_t;

typedef struct xHANDLE
{
    int iFlag;
    int iDriverId;
    Inode_t xInode;
	void *pvDriverPrivate;	//驱动私有数据
}Handle_t;

//效仿linux进行驱动模块化管理，一个operation对应一个传感器
typedef struct xDRIVER_OPERATIONS
{
	int (*open)(Handle_t *);
	int (*close)(Handle_t *);
	size_t (*read)(Handle_t *, char *, size_t);
	size_t (*write)(Handle_t *, const char *, size_t);
	int (*ioctl)(Handle_t *, int, void*);
    int (*poll) (Handle_t *, struct pollfd *, bool);
    int (*lseek) (Handle_t *, off_t, int);
} DriverOps_t;

int DriverManagerInit();
int DriverDevExists(const char *pcDrvName);
int DriverRegister(const char *pcDrvName, const DriverOps_t *pOps, void *pvPrivate);
int DriverUnregister(const char *pcDrvName);
int open(const char *pcName, int iFlag);
int close(int iFd);
size_t read(int iFd, char *pcBuffer, size_t xBufLen);
size_t write(int iFd, const char *pcBuffer, size_t xBufLen);
int ioctl(int iFd, int iCmd, void *pvArg);
int poll(struct pollfd *fds, unsigned int nfds, int timeout);
int lseek(int iFd, off_t xOffset, int iWhence);

#ifdef __cplusplus
}
#endif
#endif
