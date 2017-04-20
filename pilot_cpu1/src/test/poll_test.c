#include "driver.h"
#include "drv_accel.h"
#include "FreeRTOSConfig.h"
#include "pilot_print.h"
#include "task.h"
#include <poll.h>

struct pollfd _poll_fds[1];

static void prvPollTask( void *pvParameters )
{
    int iRet = -1;
    void *pvFd = pvParameters;

    while(1)
    {
        _poll_fds[0].fd = pvFd;
        _poll_fds[0].events = POLLIN;

        iRet = poll(_poll_fds, sizeof(_poll_fds)/sizeof(struct pollfd), 1000);
        if(iRet < 0)
            pilot_err("poll error:%d\n", iRet);
        else if(iRet == 0)
            pilot_warn("poll timout\n");
        else
            pilot_info("poll success\n");
    }
}

void PollTest()
{
    void *pvFd;

    pvFd = open(ACCEL0_DEVICE_PATH, 0);

	pilot_info("create poll task:%d\n", xTaskCreate(prvPollTask, "poll task", configMINIMAL_STACK_SIZE, pvFd, 1, NULL));
}
