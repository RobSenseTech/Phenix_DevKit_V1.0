#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <poll.h>
#include "task.h"
#include "driver.h"

static int init_flag = 0;
static int fd = -1;

void pilot_log(char *buf)
{
    if(init_flag == 0)
    {
        fd = open("/fs/microsd/logtest", O_RDWR|O_CREAT);
        init_flag = 1;
    }

    write(fd, buf, strlen(buf));   
    memset(buf, 0, strlen(buf));
}

