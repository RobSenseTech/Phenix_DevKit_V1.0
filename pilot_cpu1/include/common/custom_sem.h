#ifndef _CUSTOM_SEM_H_
#define _CUSTOM_SEM_H_
/*
 * FreeRTOS的信号量函数实际是用宏定义的队列接口，
 * 无法通过函数指针调用，此头文件的作用是封装信号量函数
 * 使其可以使用函数指针调用
 */

#include "semphr.h"

typedef SemaphoreHandle_t sem_t;

static inline sem_t sem_init(uint32_t max_count, uint32_t start_value)
{
    return xSemaphoreCreateCounting(max_count, start_value); 
}

static inline void sem_del(sem_t sem)
{
    vSemaphoreDelete((SemaphoreHandle_t)sem);
}

static inline int32_t sem_wait(sem_t sem)
{
    return xSemaphoreTake((SemaphoreHandle_t)sem, portMAX_DELAY);//等待解锁
}

static inline int32_t sem_post(sem_t sem)
{
    portBASE_TYPE xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    return xSemaphoreGiveFromISR((SemaphoreHandle_t)sem, &xHigherPriorityTaskWoken);
}

#endif 
