/* Standard includes. */
#include <stdio.h>
#include <limits.h>
#include <string.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/*Custom includes*/
#include "pilot_print.h"
#if 1
#include "drv_accel.h"
#include "drv_gyro.h"
#include "driver.h"
#include "pilot_init.h"
#include "ocm_config.h"
#include  <fs/fs.h>
#include <sys/stat.h>

void sd_test()
{
    int fd;
    int ret = 0;
    char a[20];
    char b[20] = {0};
    memset(a, 'h', sizeof(a));
    fd = open("/fs/microsd/pilot_param", O_CREAT|O_RDWR|O_SYNC, S_IRWXU|S_IRWXG|S_IRWXO); 
    if(fd < 0)
        pilot_err("open file failed:%d\n", (int)fd);
    else
    {
        ret = write(fd, a, sizeof(a));
        pilot_info("write bytes %d\n", ret);
    }

    lseek(fd, 0, SEEK_SET);
    read(fd, b, sizeof(b));
    xil_printf("read: ");
    for(int i = 0; i < 20; i++)
        xil_printf("%c ", b[i]);
    xil_printf("over\r\n");

//    close(fd);
}

#else
#include "xparameters.h"	/* SDK generated parameters */
#include "xsdps.h"		/* SD device driver */
#include "xil_printf.h"
#include "sd_test/ff.h"
#include "xil_cache.h"
#include "xplatform_info.h"

TCHAR *Path = "0:/";
static FIL fil;		/* File object */
static FATFS fatfs;
static char *SD_File;
static char FileName[32] = "0:/test.txt";
void sd_test()
{
	FRESULT Res;
    char a[512] = {'H'};
    char b[20] = {0};
	UINT NumBytesWritten;
    UINT NumBytesRead;
    memset(a, 'h', sizeof(a));

//SDIO时钟使能
#define SDIO_CLK_CTRL       (*(volatile unsigned long *)(0xf8000150))
#define SDIO0_CLK_ENABLE    (1 << 0)
#define SDIO1_CLK_ENABLE    (1 << 1)

//外设时钟使能
#define APER_CLK_CTRL       (*(volatile unsigned long *)(0xf800012c)) //AMBA Peripheral Clock Control
#define SDI0_CPU_1XCLKACT  (1 << 10)
#define SDI1_CPU_1XCLKACT  (1 << 11)

        //SDIO clock
        SDIO_CLK_CTRL |= SDIO0_CLK_ENABLE;

        //peripheral clock
        APER_CLK_CTRL |= SDI0_CPU_1XCLKACT;



	/*
	 * Register volume work area, initialize device
	 */
	Res = f_mount(&fatfs, Path, 0);

	if (Res != FR_OK) {
        xil_printf("mount err\n");
		return XST_FAILURE;
	}
    
    SD_File = (char *)FileName;

	Res = f_open(&fil, SD_File, FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
	if (Res) {
        xil_printf("open err\n");
		return XST_FAILURE;
	}

    /*
	 * Pointer to beginning of file .
	 */
	Res = f_lseek(&fil, 0);
	if (Res) {
        xil_printf("lseek err\n");
		return XST_FAILURE;
	}

	Res = f_write(&fil, (const void*)a, sizeof(a),
			&NumBytesWritten);
	if (Res) {
        xil_printf("write err\n");
		return XST_FAILURE;
	}

	/*
	 * Pointer to beginning of file .
	 */
	Res = f_lseek(&fil, 0);
	if (Res) {
		return XST_FAILURE;
	}

	Res = f_read(&fil, (void*)b, sizeof(b),
			&NumBytesRead);
	if (Res) {
		return XST_FAILURE;
	}

    xil_printf("read: ");
    for(int i = 0; i < 20; i++)
        xil_printf("%c ", b[i]);
    xil_printf("over\r\n");

}
#endif
