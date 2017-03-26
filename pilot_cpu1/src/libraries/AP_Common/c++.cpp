// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//
// C++ runtime support not provided by Arduino
//
// Note: use new/delete with caution.  The heap is small and
// easily fragmented.

#include <AP_HAL/AP_HAL.h>
#include "FreeRTOS.h"
#include <stdlib.h>
#include <FreeRTOS_Print.h>

/*
  globally override new and delete to ensure that we always start with
  zero memory. This ensures consistent behaviour.
 */
void * operator new(size_t size)
{
    void *p = NULL;
    if (size < 1) {
        size = 1;
    }

    p = pvPortMalloc(size);
    memset(p, 0, size);
    
    return p;
}

void operator delete(void *p)
{
    if (p) vPortFree(p);
}

void * operator new[](size_t size)
{
    void *p = NULL;
    if (size < 1) {
        size = 1;
    }

    p = pvPortMalloc(size);
    memset(p, 0, size);

    return p;
}

void operator delete[](void * ptr)
{
    if (ptr) vPortFree(ptr);//free(ptr);
}



