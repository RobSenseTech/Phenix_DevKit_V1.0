#include "FreeRTOS_Print.h" 
#include "ringbuffer.h"
#include "xuartps.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "driver.h"

#include <unistd.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>


static void prvUartSendTask( void *pvParameters )
{
	int iFd = (int)pvParameters;
	char TestString[] = {1,2,3,4,5,6,7,8,9};
    int iNum = 0;

	while(1)
	{
        ioctl(iFd, FIONWRITE, (unsigned long)&iNum);
        if(iNum >= sizeof(TestString))      
    		write(iFd, TestString, sizeof(TestString));

		vTaskDelay( 300 / portTICK_RATE_MS );
	}
}

static void prvUartRecvTask( void *pvParameters )
{
	int iRet = 0;
	int iNum = 0;
	int iFd = (int)pvParameters;
	char RecvBuffer[30] = {0};
    char last = 0;

	while(1)
	{
		iNum = 0;
		iRet = ioctl(iFd, FIONREAD, (unsigned long)&iNum);
		if(iRet != XST_SUCCESS)
		{
			Print_Err("ioctl failed\n");
		}
		else
		{
			Print_Warn("iNum=%d\n", iNum);
			if(iNum != 0)
			{
				int i;
				if(iNum >= 30)
					iNum = 30;
				memset(RecvBuffer, 0, sizeof(RecvBuffer));
				iNum = read(iFd, RecvBuffer, iNum);	
				for(i = 0; i< iNum; i++)
                {
            //        if((RecvBuffer[i] - last == 1) || (last == 9 && RecvBuffer[i] == 1))
                    {
					    Print_Info("recv: %d\n", RecvBuffer[i]);
                    }
             //     else
             //         Print_Err("recv err:last=%d now=%d\n", last, RecvBuffer[i]);

                    last = RecvBuffer[i];
                }
			}
		}

		vTaskDelay( 500 / portTICK_RATE_MS );
	}
}

void UartTest()
{
	int iFd;
	int iMode = UART_MODE_LOOP;	

	iFd = open("/dev/uartns0", O_RDWR);
	if(iFd < 0)
	{
		Print_Err("open uart driver failed:%d\n", iFd);
		return;
	}

	ioctl(iFd, UART_IOC_SET_MODE, &iMode);
	
	Print_Info("create uart send task:%d\n", xTaskCreate(prvUartSendTask, "uart-send", configMINIMAL_STACK_SIZE*2, (void *)iFd, 1, NULL));
	Print_Info("create uart recv task:%d\n", xTaskCreate(prvUartRecvTask, "uart-recv", configMINIMAL_STACK_SIZE*2, (void *)iFd, 1, NULL));
}

