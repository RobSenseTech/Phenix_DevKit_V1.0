#include <stdio.h>
#include "driver.h"
#include "drv_accel.h"
#include "drv_gyro.h"
#include "drv_mag.h"
#include "drv_baro.h"
#include "uORB/uORB.h"
#include "ringbuffer.h"
#include "uORB/topics/sensor_gyro.h"
#include "uORB/topics/sensor_baro.h"
#include "task.h"

#include <unistd.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <poll.h>


static void prvGyroTask(void *pvParameters)
{
    int gyro_sub = orb_subscribe_multi(ORB_ID(sensor_gyro), 0);//订阅第一个陀螺仪的数据
    int ret = 0;
    struct sensor_gyro_s gyro;


    while(1)
    {
        struct pollfd fds[1];
        memset(fds, 0, sizeof(fds));
        fds[0].fd = gyro_sub;
        fds[0].events = POLLIN;

//        Print_Err("poll start:gyro_sub=%x\n", (int)gyro_sub);
        ret = poll(fds, sizeof(fds)/sizeof(fds[0]), 500);
//        Print_Err("poll over:gyro_sub=%x\n", (int)gyro_sub);
        if(ret < 0)
            Print_Err("poll error\n");
        else if(ret == 0)
            Print_Warn("poll timeout");
        else
        {
            if(fds[0].revents & POLLIN)
            {
                orb_copy(ORB_ID(sensor_gyro), gyro_sub, &gyro);
                Print_Info("get data:rawxyz[%04x,%04x,%04x] x=%f y=%f z=%f size=%x\n", gyro.x_raw, gyro.y_raw, gyro.z_raw, gyro.x, gyro.y, gyro.z, sizeof(struct sensor_gyro_s));
            }
            else
            {
                Print_Warn("revents=%x\n", fds[0].revents);
            }
        }

		vTaskDelay( 300 / portTICK_RATE_MS );
   
    }
}

static void prvAccelTask(void *pvParameters)
{
    int accel_sub = orb_subscribe_multi(ORB_ID(sensor_accel), 0);
    int ret = 0;
    struct sensor_accel_s accel;


    while(1)
    {
        struct pollfd fds[1];
        memset(fds, 0, sizeof(fds));
        fds[0].fd = accel_sub;
        fds[0].events = POLLIN;

//        Print_Err("poll start:accel_sub=%x\n", (int)accel_sub);
        ret = poll(fds, sizeof(fds)/sizeof(fds[0]), 500);
 //       Print_Err("poll over:accel_sub=%x\n", (int)accel_sub);
        if(ret < 0)
            Print_Err("poll error\n");
        else if(ret == 0)
            Print_Warn("poll timeout");
        else
        {
            if(fds[0].revents & POLLIN)
            {
                orb_copy(ORB_ID(sensor_accel), accel_sub, &accel);
                Print_Warn("accel data:rawxyz[%04x,%04x,%04x] x=%f y=%f z=%f size=%x\n", accel.x_raw, accel.y_raw, accel.z_raw, accel.x, accel.y, accel.z, sizeof(struct sensor_accel_s));
            }
            else
            {
                Print_Warn("revents=%x\n", fds[0].revents);
            }
        }

		vTaskDelay( 300 / portTICK_RATE_MS );
   
    }
}

static void prvMagTask(void *pvParameters)
{
    int mag_sub = orb_subscribe_multi(ORB_ID(sensor_mag), 0);
    int ret = 0;
    struct sensor_mag_s mag;


    while(1)
    {
        struct pollfd fds[1];
        memset(fds, 0, sizeof(fds));
        fds[0].fd = mag_sub;
        fds[0].events = POLLIN;

//        Print_Info("poll start:mag_sub=%x\n", (int)mag_sub);
        ret = poll(fds, sizeof(fds)/sizeof(fds[0]), 500);
        if(ret < 0)
            Print_Err("poll error\n");
        else if(ret == 0)
            Print_Warn("poll timeout");
        else
        {
            if(fds[0].revents & POLLIN)
            {
                orb_copy(ORB_ID(sensor_mag), mag_sub, &mag);
                Print_Err("mag data:rawxyz[%04x,%04x,%04x] x=%f y=%f z=%f size=%x\n", mag.x_raw, mag.y_raw, mag.z_raw, mag.x, mag.y, mag.z, sizeof(struct sensor_mag_s));
            }
            else
            {
                Print_Warn("revents=%x\n", fds[0].revents);
            }
        }

		vTaskDelay( 300 / portTICK_RATE_MS );
   
    }

}

#if 0
static void prvBaroTask(void *pvParameters)
{
    int baro_sub = orb_subscribe_multi(ORB_ID(sensor_baro), 0);
    int ret = 0;
    struct sensor_baro_s baro;


    while(1)
    {
        struct pollfd fds[1];
        memset(fds, 0, sizeof(fds));
        fds[0].fd = baro_sub;
        fds[0].events = POLLIN;

//        Print_Info("poll start:baro_sub=%x\n", (int)baro_sub);
        ret = poll(fds, sizeof(fds)/sizeof(fds[0]), 500);
        if(ret < 0)
            Print_Err("poll error\n");
        else if(ret == 0)
            Print_Warn("poll timeout");
        else
        {
            if(fds[0].revents & POLLIN)
            {
                orb_copy(ORB_ID(sensor_baro), baro_sub, &baro);
                Print_Err("baro data:pressure=%f altitude=%f\n", baro.pressure, baro.altitude);
            }
            else
            {
                Print_Warn("revents=%x\n", fds[0].revents);
            }
        }

		vTaskDelay( 300 / portTICK_RATE_MS );
   
    }

}
#else
static void prvBaroTask(void *pvParameters)
{
    int fd = open(BARO_BASE_DEVICE_PATH"0", O_RDONLY);
    while(1)
    {
        struct baro_report baro_report;
        
        read(fd, (char*)&baro_report, sizeof(baro_report));
        Print_Info("altitude=%f pressure=%f\n", baro_report.altitude, baro_report.pressure);

		vTaskDelay( 300 / portTICK_RATE_MS );
    }
}
#endif

void ImuTest()
{
    Print_Info("create gyro test task:%d\n", xTaskCreate(prvGyroTask, "gyro test", 4000, NULL, 1, NULL));
    Print_Info("create accel test task:%d\n", xTaskCreate(prvAccelTask, "accel test", 4000, NULL, 1, NULL));
    Print_Info("create mag test task:%d\n", xTaskCreate(prvMagTask, "mag test", 4000, NULL, 1, NULL));
    Print_Info("create baro test task:%d\n", xTaskCreate(prvBaroTask, "baro test", 4000, NULL, 1, NULL));
}

