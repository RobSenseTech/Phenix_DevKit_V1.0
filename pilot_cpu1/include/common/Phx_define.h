#ifndef __PHX_DEFINE_H
#define __PHX_DEFINE_H

#include "FreeRTOS.h"

#define PX4_TICKS_PER_SEC 1000L/portTICK_RATE_MS
#define USEC_PER_TICK (1000000UL/PX4_TICKS_PER_SEC)
#define USEC2TICK(x) ((x)/USEC_PER_TICK)

#define TICK2USEC(tick)       ((tick)*USEC_PER_TICK)                     /* Exact */


#define OK 0
#define ERROR -1

#endif
