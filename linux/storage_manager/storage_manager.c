#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <byteswap.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <sys/mman.h>
#include <string.h>
#include "../../pilot_cpu1/include/config/ocm_config.h"

#define PAGE_SIZE ((size_t)getpagesize())

int main()
{
	int mem_fd;
    int param_fd;
	volatile uint8_t *tx_flag;
	volatile uint8_t *storage_virAddr;
    struct stat st;
    uint32_t flag;
    int32_t ret = 0;

	mem_fd = open("/dev/mem", O_RDWR|O_SYNC);
	if (mem_fd < 0) {
		printf("open(/dev/mem) failed (%d)\n", errno);
		return -1;
	}

	tx_flag = mmap(NULL, PAGE_SIZE*5, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, COMM_TX_FLAG_ADDR);
	if (tx_flag == MAP_FAILED) {
		printf("mmap64(0x4@0x%x) failed (%d)\n", (uint32_t)(COMM_TX_FLAG_ADDR), errno);
		return -1;
	}

    storage_virAddr = tx_flag+sizeof(int32_t);

    *(volatile uint32_t *)tx_flag = 0;
    memset((void *)storage_virAddr, 0, HAL_STORAGE_SIZE);

    //如果不存在，则创建
    if(stat(MTD_PARAMS_FILE, &st) < 0)
    {
        uint8_t v[128] = {0};
        uint32_t pos;
        printf(">>>>>>>>>>>>>>>>>>>>..there is no %s, create it!!\n", MTD_PARAMS_FILE);
        param_fd = open(MTD_PARAMS_FILE, O_CREAT|O_RDWR|O_SYNC, S_IRWXU|S_IRWXG|S_IRWXO); 
        for(pos = 0; pos < HAL_STORAGE_SIZE; pos += 128)
        {
            write(param_fd, v, sizeof(v));
        } 
    }
    else
    {
        //开机时，把文件内容写入ocm
        param_fd = open(MTD_PARAMS_FILE, O_RDWR|O_SYNC); 
        if(param_fd < 0)
        {
            printf(">>>>>>>>>>>>>>>>>>open %s failed, exit!!\n", MTD_PARAMS_FILE);
            exit(1);
        }
        lseek(param_fd, 0, SEEK_SET);
        ret = read(param_fd, (void *)storage_virAddr, HAL_STORAGE_SIZE);
        printf(">>>>>>>>>>>>>>>>ret=%d read param:%x %x %x\n", ret, storage_virAddr[0], storage_virAddr[1], storage_virAddr[2]);
    }

    while(1)
    {
        flag = *(volatile uint32_t *)tx_flag;
//       printf("flag=%d\n", flag);
        //cpu1有数据写入ocm
        if(flag != 0)
        {
            lseek(param_fd, 0, SEEK_SET);
            write(param_fd, (const void *)storage_virAddr, HAL_STORAGE_SIZE);
  //          printf("write %x %x %x\n", storage_virAddr[0], storage_virAddr[1], storage_virAddr[2]);
            *(volatile uint32_t *)tx_flag = flag - 1;
        } 

        usleep(500000);
    }

    return 0;
}
