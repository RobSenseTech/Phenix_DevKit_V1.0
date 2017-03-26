/*
 * Copyright (c) 2012 Xilinx, Inc.  All rights reserved.
 *
 * Xilinx, Inc.
 * XILINX IS PROVIDING THIS DESIGN, CODE, OR INFORMATION "AS IS" AS A
 * COURTESY TO YOU.  BY PROVIDING THIS DESIGN, CODE, OR INFORMATION AS
 * ONE POSSIBLE   IMPLEMENTATION OF THIS FEATURE, APPLICATION OR
 * STANDARD, XILINX IS MAKING NO REPRESENTATION THAT THIS IMPLEMENTATION
 * IS FREE FROM ANY CLAIMS OF INFRINGEMENT, AND YOU ARE RESPONSIBLE
 * FOR OBTAINING ANY RIGHTS YOU MAY REQUIRE FOR YOUR IMPLEMENTATION.
 * XILINX EXPRESSLY DISCLAIMS ANY WARRANTY WHATSOEVER WITH RESPECT TO
 * THE ADEQUACY OF THE IMPLEMENTATION, INCLUDING BUT NOT LIMITED TO
 * ANY WARRANTIES OR REPRESENTATIONS THAT THIS IMPLEMENTATION IS FREE
 * FROM CLAIMS OF INFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

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

#define COMM_BASE       0xFFFF9000
#define COMM_TX_FLAG_OFFSET    0x00
#define PAGE_SIZE ((size_t)getpagesize())

int main()
{
	int fd;
	volatile uint8_t *virAddr;
	uint32_t flag=0;
	uint32_t lastFlag=0;

	fd = open("/dev/mem", O_RDWR|O_SYNC);
	if (fd < 0) {
		printf("open(/dev/mem) failed (%d)\n", errno);
		return -1;
	}

	virAddr = mmap(NULL, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, COMM_BASE);
	if (virAddr == MAP_FAILED) {
		printf("mmap64(0x%x@0x%x) failed (%d)\n", PAGE_SIZE, (uint32_t)(COMM_BASE), errno);
		return -1;
	}

	while(1)
	{
		flag = *(volatile uint32_t *)(virAddr + COMM_TX_FLAG_OFFSET);

		if(lastFlag != flag)
		{
			printf("linux:flag=%d\n", flag);
			lastFlag = flag;
		}
		else
		{
			printf("linux:flag not change\n");
		}

		usleep(1000000);
	}

    return 0;
}
