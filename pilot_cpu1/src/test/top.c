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

#define TOP_BUF_SIZE    10000
static void top_task(void *pvParameters)
{
    char *buf = NULL;

    buf = pvPortMalloc(TOP_BUF_SIZE);
    memset(buf, 0, TOP_BUF_SIZE);

    while(1)
    {
        uint32_t len = 0;
        vTaskGetRunTimeStats(buf); 
        //clear terminal
        printf("\033[2J\033[1H");
        printf("\n%s\n", buf);
        memset(buf, 0, len);

        vTaskDelay(1000 / portTICK_RATE_MS );
    }

    
}

int top_main()
{

    pilot_info("create top test task:%d\n", xTaskCreate(top_task, "top test", 4000, NULL, 1, NULL));
    return 0;
}


