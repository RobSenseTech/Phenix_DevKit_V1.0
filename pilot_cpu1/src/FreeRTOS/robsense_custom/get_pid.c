#include "FreeRTOS.h"
#include "task.h"
#include <sys/types.h>
#include <unistd.h>

pid_t getpid(void)
{
  /* Return the task ID from the TCB at the head of the
   * ready-to-run task list
   */
    TaskHandle_t current_handle = xTaskGetCurrentTaskHandle();

    return uxTaskGetTaskNumber((void *)current_handle);
}
