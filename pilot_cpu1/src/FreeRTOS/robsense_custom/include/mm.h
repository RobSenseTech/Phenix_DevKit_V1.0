#include <string.h>
#include "FreeRTOS.h"

#define kfree(p)                vPortFree(p)
#define kufree(p)               vPortFree(p)
#define kumalloc(s)             pvPortMalloc(s)
#define kuzalloc(s)             kzalloc(s)
#define kmalloc(s)             pvPortMalloc(s)

void *kzalloc(size_t size);
