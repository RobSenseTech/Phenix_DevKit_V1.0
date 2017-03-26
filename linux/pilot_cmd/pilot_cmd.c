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
#include <signal.h>
#include "../../pilot_cpu1/include/config/ocm_config.h"

#define PAGE_SIZE ((size_t)getpagesize())

void cmd_exit(int sig)
{
    printf("pilot debug command line exit\n");
    signal(SIGINT, SIG_DFL);
    exit(1);
}

void cmd_usage()
{
    printf("usage: driver name + [cmd] + [argument]\n");
} 

int main()
{
    char cmd[MAX_CMD_LEN] ={0};
	int mem_fd;
	volatile uint8_t *cmd_virAddr;

    signal(SIGINT, cmd_exit);
	mem_fd = open("/dev/mem", O_RDWR|O_SYNC);
	if (mem_fd < 0) {
		printf("open(/dev/mem) failed (%d)\n", errno);
		return -1;
	}

	cmd_virAddr = mmap(NULL, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, CMD_ADDR);
	if (cmd_virAddr == MAP_FAILED) {
		printf("mmap64(0x4@0x%x) failed (%d)\n", (uint32_t)(CMD_ADDR), errno);
		return -1;
	}
    memset(cmd_virAddr, 0, PAGE_SIZE);

    while(1)
    {
        memset(cmd, 0, sizeof(cmd));    
        //等待cpu1清空ocm
        while(*cmd_virAddr != 0)
        {
            usleep(100000);
        }

        printf("FreeRTOS:");
        if(fgets(cmd, MAX_CMD_LEN, stdin) != NULL) //fgets会把最后的回车符也读进来
        {
//            printf("what I got:%s strlen=%d\n", cmd, strlen(cmd));
            if(!strcmp(cmd, "?\n") || !strcmp(cmd, "help\n"))
                cmd_usage();
            else
            {
                memcpy(cmd_virAddr, cmd, sizeof(cmd));
            }
        }
        else
            printf("cmd too long, no more than 50 chars\n");
    }

    return 0;
}
