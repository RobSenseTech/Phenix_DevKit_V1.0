#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#include "AP_HAL_PX4.h"

class PX4::Semaphore : public AP_HAL::Semaphore {
public:
    Semaphore() {
        //pthread_mutex_init(&_lock, NULL);
    	_lock = xSemaphoreCreateMutex();	//创建一个互斥锁
    }
    bool give();
    bool take(uint32_t timeout_ms);
    bool take_nonblocking();
private:
    //pthread_mutex_t _lock;
    SemaphoreHandle_t _lock;           //定义一个互斥锁
};
