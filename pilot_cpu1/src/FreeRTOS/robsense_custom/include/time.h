#ifndef CUSTOM_TIME_H
#define CUSTOM_TIME_H

#include <stdint.h>

typedef uint32_t  time_t;         /* Holds time in seconds */

struct timeval
{
    time_t tv_sec;                   /* Seconds */
    long tv_usec;                    /* Microseconds */
};

#define up_delay(m)     vTaskDelay(((m<1000)?1000:m)/1000/portTICK_RATE_MS);
        

#endif
