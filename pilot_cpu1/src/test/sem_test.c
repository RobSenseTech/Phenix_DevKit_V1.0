#include "FreeRTOSConfig.h"
#include "pilot_print.h"
#include "task.h"

SemaphoreHandle_t xSemaphore;

static void prvTask1( void *pvParameters )
{
    while(1)
    {
        pilot_info("prepare to give sem\n");
        xSemaphoreGive(xSemaphore);

		vTaskDelay( 1000 / portTICK_RATE_MS );
    }
}


static void prvTask2( void *pvParameters )
{
    while(1)
    {
        xSemaphoreTake(xSemaphore, portMAX_DELAY);
        pilot_info("get sem now\n");
    }
}

void SemTest()
{
    xSemaphore = xSemaphoreCreateBinary();
    
	pilot_info("create task1:%d\n", xTaskCreate(prvTask1, "task1", configMINIMAL_STACK_SIZE, NULL, 1, NULL));
	pilot_info("create task2:%d\n", xTaskCreate(prvTask2, "task2", configMINIMAL_STACK_SIZE, NULL, 1, NULL));
}
