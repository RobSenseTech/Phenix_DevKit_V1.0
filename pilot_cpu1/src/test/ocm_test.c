#include "ocm/ocm.h"
#include "pilot_print.h"
#include <stdio.h>
#include "driver.h"
#include "ringbuffer.h"
#include "task.h"

#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <poll.h>


static int channel = 0;

static void ocm_write_task(void *pvParameters)
{
    int i;
    uint8_t a[100];

    for(i = 0; i<100; i++)
    {
        a[i] = i;
    }

    while(1)
    {
        int32_t bytes = 0;
        int32_t write_len = sizeof(a);
        while(write_len > 0)
        {
            bytes = ocm_msg_send(0, &a[sizeof(a)-write_len], write_len);
            pilot_warn("write %d bytes\n", bytes);
            write_len -= bytes;
            vTaskDelay( 100 / portTICK_RATE_MS );
        }
        vTaskDelay( 100 / portTICK_RATE_MS );
    }


}

static void ocm_read_task(void *pvParameters)
{
    uint8_t buf[13];

    while(1)
    {
        int32_t bytes = 0;
        bytes = ocm_msg_recv(channel, buf, sizeof(buf));
        if(bytes != 0)
            pilot_info("recv %d bytes:\n", bytes);
        for(int i = 1; i < bytes; i++)
        {
            char delt = 0;
            delt = buf[i] - buf[i-1];
            if(delt != 1 && buf[i-1] != 99 && buf[i] != 0)        
                pilot_err("lose data:%d %d\n", buf[i-1], buf[i]);
        }

		vTaskDelay( 1000 / portTICK_RATE_MS );
    }
}

void ocm_test()
{
    ocm_msg_chn_init(channel, "ocm test");

    pilot_info("create ocm write task:%d\n", xTaskCreate(ocm_write_task, "ocm test", 4000, NULL, 1, NULL));
//    pilot_info("create ocm read task:%d\n", xTaskCreate(ocm_read_task, "ocm test", 4000, NULL, 1, NULL));
}

void ocmfs_test()
{
    int fd = 0;
    char a[512];
    struct stat st;
    DIR *dir = NULL;
    struct dirent *p_dir = NULL;

    for(int i=0; i<512; i++)
        a[i] = i;

    fd = open("/fs/microsd/ocmtest", O_CREAT|O_RDWR, 0777);

    write(fd, a, 512);
    pilot_err("lseek ret=%d\n", lseek(fd, 10, SEEK_SET));
    close(fd);
    fd = open("/fs/microsd/ocmtest", O_CREAT|O_RDWR, 0777);
    read(fd, a, 512);
    stat("/fs/microsd/ocmtest", &st);

    mkdir("/fs/microsd/dirtest", 0777);
    rename("/fs/microsd/ocmtest", "/fs/microsd/ocmrename");
    dir = opendir("/fs/microsd");
    while((p_dir = readdir(dir)) != NULL)
    {
        pilot_info("type=%d d_name=%s\n", p_dir->d_type, p_dir->d_name);
    }
}
