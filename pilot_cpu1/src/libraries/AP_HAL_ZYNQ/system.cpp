/*
 * system.cpp
 *
 *  Created on: 2016年7月27日
 *      Author: RST011
 */
#include <stdarg.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>

#include <hrt/drv_hrt.h>        //等待底层提供

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/system.h>
#include "device/cdev.h"

extern const AP_HAL::HAL& hal;

extern bool _px4_thread_should_exit;

namespace AP_HAL {

void init()
{
}

void panic(const char *errormsg, ...)
{
    Print_Err("%s\n", errormsg);
    return;
    va_list ap;

    va_start(ap, errormsg);
    vdprintf(1, errormsg, ap);
    va_end(ap);
    write(1, "\n", 1);           /////////////

    hal.scheduler->delay_microseconds(10000);
    _px4_thread_should_exit = true;
    exit(1);
}

uint32_t micros()
{
    return micros64() & 0xFFFFFFFF;
}

uint32_t millis()
{
    return millis64() & 0xFFFFFFFF;
}

uint64_t micros64()
{
    return hrt_absolute_time();
}

uint64_t millis64()
{
    return micros64() / 1000;
}

} // namespace AP_HAL




