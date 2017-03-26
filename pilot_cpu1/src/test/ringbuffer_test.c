#include "FreeRTOS_Print.h" 
#include "ringbuffer.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

RingBuffer_t xRingBuf;
static void prvRingBufferWriteTask( void *pvParameters )
{
	int j = 0; 
	while(1)
	{
		if(xRingBufferPut(&xRingBuf, &j, sizeof(int)) == 0)
		{
			j++;
		}
		vTaskDelay( 1000 / portTICK_RATE_MS );
	}

}

static void prvRingBufferReadTask( void *pvParameters )
{
	int j = 0;
	while(1)
	{
		if(xRingBufferGet(&xRingBuf, &j, sizeof(int)) == 0)
		{
			Print_Info("read from ringbuffer:%d filled:%d empty:%d\n", j, iRingBufferGetAvailable(&xRingBuf), iRingBufferGetSpace(&xRingBuf));
		}
		vTaskDelay( 2000 / portTICK_RATE_MS );
	}

}

int iRingBufferTest()
{
	Print_Info("Start ring buffer test\n");
	iRingBufferInit(&xRingBuf, 10, sizeof(int));

	Print_Info("create write task:%d\n", xTaskCreate(prvRingBufferWriteTask, "write-ringbuffer", configMINIMAL_STACK_SIZE, NULL, 1, NULL));
	Print_Info("create read task:%d\n", xTaskCreate(prvRingBufferReadTask, "read-ringbuffer", configMINIMAL_STACK_SIZE, NULL, 1, NULL));
//	xTaskCreate(prvRingBufferReadTask, "read-ringbuffer", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

	return 0;
}
