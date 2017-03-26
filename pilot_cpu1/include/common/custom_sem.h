#ifndef _CUSTOM_SEM_H_
#define _CUSTOM_SEM_H_
/*
 * FreeRTOS的信号量函数实际是用宏定义的队列接口，
 * 无法通过函数指针调用，此头文件的作用是封装信号量函数
 * 使其可以使用函数指针调用
 */

#include "FreeRTOS.h"
#include "semphr.h"
#include <sys/types.h>
#include <limits.h>
#include <errno.h>

#define SEM_VALUE_MAX   0x7fff

typedef SemaphoreHandle_t sem_t;

/*
static inline sem_t sem_init(uint32_t max_count, uint32_t start_value)
{
    return xSemaphoreCreateCounting(max_count, start_value); 
}
*/
static inline int sem_init(sem_t *sem, int pshared, unsigned int start_value)
{

    if (sem && start_value <= SEM_VALUE_MAX)
    {
        *sem = xSemaphoreCreateCounting(SEM_VALUE_MAX, start_value); 
        return OK;
    }
    else
    {
        set_errno(EINVAL);
        return ERROR;
    }

}


static inline int sem_destroy (sem_t *sem)
{
    if (sem)
    {
        vSemaphoreDelete((SemaphoreHandle_t)*sem);
        return OK;
    }
    else
    {
        errno = -EINVAL;
        return ERROR;
    }
}

static inline int32_t sem_wait(sem_t *sem)
{
    if(xSemaphoreTake((SemaphoreHandle_t)*sem, portMAX_DELAY))//等待解锁
        return OK;
    else
        return ERROR;
}

static inline int32_t sem_post(sem_t *sem)
{
    portBASE_TYPE xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    if(xSemaphoreGiveFromISR((SemaphoreHandle_t)*sem, &xHigherPriorityTaskWoken))
        return OK;
    else
        return ERROR;
}

#endif 
