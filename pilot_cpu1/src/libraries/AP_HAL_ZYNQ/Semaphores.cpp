#include <AP_HAL/AP_HAL.h>


#include "Semaphores.h"

extern const AP_HAL::HAL& hal;

using namespace PX4;

bool Semaphore::give() 
{
    //return pthread_mutex_unlock(&_lock) == 0;
    //return xSemaphoreTake(&_lock) == 0;
	return xSemaphoreGive(_lock) == pdTRUE;      //xSemaphoreGive函数释放锁成功返回pdTRUE
}

bool Semaphore::take(uint32_t timeout_ms)
{
    if (timeout_ms == 0) {
        //return pthread_mutex_lock(&_lock) == 0;
    	return xSemaphoreTake(_lock, portMAX_DELAY) == pdTRUE;   //如果获取到信号量返回pdTRUE
    }


    if (take_nonblocking()) {
        return true;
    }
    uint64_t start = AP_HAL::micros64();
    do {
        hal.scheduler->delay_microseconds(200);
        if (take_nonblocking()) {
            return true;
        }
    } while ((AP_HAL::micros64() - start) < timeout_ms*1000);
    return false;

}

bool Semaphore::take_nonblocking() 
{
    //return pthread_mutex_trylock(&_lock) == 0;
	return xSemaphoreTake(_lock,(TickType_t) 0) == pdTRUE;
}

