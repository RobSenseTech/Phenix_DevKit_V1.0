#include "pilot_print.h" 
#include "ringbuffer.h"
#include "xuartps.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "rc/sbus.h"
#include "uORB/topics/input_rc.h"
#include "board_config.h"

static void prvSbusTask( void *pvParameters )
{
   int sbus_fd = 0; 

    sbus_fd = sbus_init(RC_SERIAL_PORT, true);
    if(sbus_fd == 0)
    {
        pilot_err("sbus init error!!\n");
    }

	while(1)
	{
        int i;
        bool sbus_failsafe, sbus_frame_drop;
        uint16_t raw_rc_values[RC_INPUT_MAX_CHANNELS];
        uint16_t raw_rc_count;
        bool sbus_updated = 0;

        sbus_updated = sbus_input(sbus_fd, raw_rc_values, &raw_rc_count, &sbus_failsafe, &sbus_frame_drop, RC_INPUT_MAX_CHANNELS);
        if(sbus_updated)
        {
            for(i=0; i<raw_rc_count; i++)
            {
               pilot_info("raw_rc_values[%d]=%d\n", i, raw_rc_values[i]);
            }
        }

		vTaskDelay( 20 / portTICK_RATE_MS );
	}
}

void SbusTest()
{

	pilot_info("create sbus task:%d\n", xTaskCreate(prvSbusTask, "subs", configMINIMAL_STACK_SIZE*2, NULL, 1, NULL));
}

