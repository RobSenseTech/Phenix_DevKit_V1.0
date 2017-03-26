/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>
#include "device/cdev.h"
#include "AP_HAL_PX4.h"
#include "Scheduler.h"

#include "hrt/drv_hrt.h"
#include "custom_sem.h"

#include "UARTDriver.h"
#include "AnalogIn.h"
#include "Storage.h"
#include "RCOutput.h"
#include "RCInput.h"
//#include <AP_Scheduler/AP_Scheduler.h>

using namespace PX4;

extern const AP_HAL::HAL& hal;

extern bool _px4_thread_should_exit;

PX4Scheduler::PX4Scheduler() 
{}

void PX4Scheduler::init()
{
    _main_task_handle = xTaskGetCurrentTaskHandle();

    // setup the timer thread - this will call tasks at 1kHz
    xTaskCreate(&PX4Scheduler::_timer_thread, "timer thread", 2048, this, APM_TIMER_PRIORITY, &_timer_thread_ctx);

    // the UART thread runs at a medium priority
    xTaskCreate(&PX4Scheduler::_uart_thread, "uart thread", 2048, this, APM_UART_PRIORITY, &_uart_thread_ctx);

    // the IO thread runs at lower priority
    xTaskCreate(&PX4Scheduler::_io_thread, "io thread", 2048, this, APM_IO_PRIORITY, &_io_thread_ctx);

    // the storage thread runs at just above IO priority

    xTaskCreate(&PX4Scheduler::_storage_thread, "storate thread", 2048, this, APM_STORAGE_PRIORITY, &_storage_thread_ctx);
}

/**
   delay for a specified number of microseconds using a semaphore wait
 */
void PX4Scheduler::delay_microseconds_semaphore(uint16_t usec) 
{
    sem_t wait_semaphore;
    struct hrt_call wait_call;
    
    wait_semaphore = sem_init(1, 0);
    memset(&wait_call, 0, sizeof(wait_call));
    hrt_call_after(&wait_call, usec, (hrt_callout)sem_post, (void *)wait_semaphore);
    sem_wait(wait_semaphore);
    sem_del(wait_semaphore);
}


void PX4Scheduler::delay_microseconds(uint16_t usec) 
{
    delay_microseconds_semaphore(usec);
}

/*
  wrapper around sem_post that boosts main thread priority
 */
static void sem_post_boost(sem_t *sem)
{
    hal_px4_set_priority(APM_MAIN_PRIORITY_BOOST);
    sem_post(sem);
}

/*
  return the main thread to normal priority
 */
static void set_normal_priority(void *sem)
{
    hal_px4_set_priority(APM_MAIN_PRIORITY);
}


/*
  a varient of delay_microseconds that boosts priority to
  APM_MAIN_PRIORITY_BOOST for APM_MAIN_PRIORITY_BOOST_USEC
  microseconds when the time completes. This significantly improves
  the regularity of timing of the main loop as it takes 
 */
void PX4Scheduler::delay_microseconds_boost(uint16_t usec) 
{
    sem_t wait_semaphore;
    static struct hrt_call wait_call;
    wait_semaphore = sem_init(1, 0);
    hrt_call_after(&wait_call, usec, (hrt_callout)sem_post_boost, wait_semaphore);
    sem_wait(wait_semaphore);
    hrt_call_after(&wait_call, APM_MAIN_PRIORITY_BOOST_USEC, (hrt_callout)set_normal_priority, NULL);

    sem_del(wait_semaphore);
}


void PX4Scheduler::delay(uint16_t ms)
{
    if (in_timerprocess()) {
        Print_Err("ERROR: delay() from timer process\n");
        return;
    }
	uint64_t start = AP_HAL::micros64();
    
    while ((AP_HAL::micros64() - start)/1000 < ms && 
           !_px4_thread_should_exit) {
        delay_microseconds_semaphore(1000);
        if (_min_delay_cb_ms <= ms) {
            if (_delay_cb) {
                _delay_cb();
            }
        }
    }

    if (_px4_thread_should_exit) {
        return;
    }
}

void PX4Scheduler::register_delay_callback(AP_HAL::Proc proc,
                                            uint16_t min_time_ms) 
{
    _delay_cb = proc;
    _min_delay_cb_ms = min_time_ms;
}

void PX4Scheduler::register_timer_process(AP_HAL::MemberProc proc) 
{
    for (uint8_t i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i] == proc) {
            return;
        }
    }

    if (_num_timer_procs < PX4_SCHEDULER_MAX_TIMER_PROCS) {
        _timer_proc[_num_timer_procs] = proc;
        _num_timer_procs++;
    } else {
      //  hal.console->printf("Out of timer processes\n");
      Print_Info("Out of timer processes\n");
    }
}

void PX4Scheduler::register_io_process(AP_HAL::MemberProc proc) 
{
    for (uint8_t i = 0; i < _num_io_procs; i++) {
        if (_io_proc[i] == proc) {
            return;
        }
    }

    if (_num_io_procs < PX4_SCHEDULER_MAX_TIMER_PROCS) {
        _io_proc[_num_io_procs] = proc;
        _num_io_procs++;
    } else {
        hal.console->printf("Out of IO processes\n");
    }
}

void PX4Scheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us) 
{
    _failsafe = failsafe;
}

void PX4Scheduler::suspend_timer_procs() 
{
    _timer_suspended = true;
}

void PX4Scheduler::resume_timer_procs() 
{
    _timer_suspended = false;
    if (_timer_event_missed == true) {
        _run_timers(false);
        _timer_event_missed = false;
    }
}

void PX4Scheduler::reboot(bool hold_in_bootloader) 
{
    Print_Warn("Called system reset");
//	px4_systemreset(hold_in_bootloader);
}

void PX4Scheduler::_run_timers(bool called_from_timer_thread)
{
    if (_in_timer_proc) {
        return;
    }
    _in_timer_proc = true;

    if (!_timer_suspended) {
        // now call the timer based drivers
        for (int i = 0; i < _num_timer_procs; i++) {
            if (_timer_proc[i]) {
                _timer_proc[i]();
            }
        }
    } else if (called_from_timer_thread) {
        _timer_event_missed = true;
    }

    // and the failsafe, if one is setup
    if (_failsafe != NULL) {
        _failsafe();
    }

    // process analog input
    ((PX4AnalogIn *)hal.analogin)->_timer_tick();

    _in_timer_proc = false;
}

extern bool px4_ran_overtime;

void PX4Scheduler::_timer_thread(void *arg)
{
    PX4Scheduler *sched = (PX4Scheduler *)arg;
    uint32_t last_ran_overtime = 0;

    while (!sched->_hal_initialized) {
        poll(NULL, 0, 1);        
    }
    while (!_px4_thread_should_exit) {
        sched->delay_microseconds_semaphore(1000);

        // run registered timers
        sched->_run_timers(true);

        // process any pending RC output requests
        ((PX4RCOutput *)hal.rcout)->_timer_tick();

        // process any pending RC input requests
        ((PX4RCInput *)hal.rcin)->_timer_tick();

        if (px4_ran_overtime && AP_HAL::millis() - last_ran_overtime > 2000) {
            last_ran_overtime = AP_HAL::millis();
    //        printf("Overtime in task %d\n", (int)AP_Scheduler::current_task);
     //       hal.console->printf("Overtime in task %d\n", (int)AP_Scheduler::current_task);
        }
    }
}

void PX4Scheduler::_run_io(void)
{
    if (_in_io_proc) {
        return;
    }
    _in_io_proc = true;

    if (!_timer_suspended) {
        // now call the IO based drivers
        for (int i = 0; i < _num_io_procs; i++) {
            if (_io_proc[i]) {
                _io_proc[i]();
            }
        }
    }

    _in_io_proc = false;
}

void PX4Scheduler::_uart_thread(void *arg)
{
    PX4Scheduler *sched = (PX4Scheduler *)arg;

    while (!sched->_hal_initialized) {
        poll(NULL, 0, 1);
    }
    while (!_px4_thread_should_exit) {
        sched->delay_microseconds_semaphore(1000);

        // process any pending serial bytes
        ((PX4UARTDriver *)hal.uartA)->_timer_tick();
        ((PX4UARTDriver *)hal.uartB)->_timer_tick();
        ((PX4UARTDriver *)hal.uartC)->_timer_tick();
        ((PX4UARTDriver *)hal.uartD)->_timer_tick();
        ((PX4UARTDriver *)hal.uartE)->_timer_tick();
    }
}

void PX4Scheduler::_io_thread(void *arg)
{
    PX4Scheduler *sched = (PX4Scheduler *)arg;

    while (!sched->_hal_initialized) {
        poll(NULL, 0, 1);
    }
    while (!_px4_thread_should_exit) {
        poll(NULL, 0, 1);

        // run registered IO processes
        sched->_run_io();
    }
    return ;
}

void PX4Scheduler::_storage_thread(void *arg)
{
    PX4Scheduler *sched = (PX4Scheduler *)arg;

    while (!sched->_hal_initialized) {
        poll(NULL, 0, 1);
    }
    while (!_px4_thread_should_exit) {
        poll(NULL, 0, 10);

        // process any pending storage writes
        ((PX4Storage *)hal.storage)->_timer_tick();
    }
    return ;
}


bool PX4Scheduler::in_timerprocess() 
{
    return _main_task_handle != xTaskGetCurrentTaskHandle();
}

void PX4Scheduler::system_initialized() {
    if (_initialized) {
        AP_HAL::panic("PANIC: scheduler::system_initialized called"
                   "more than once");
    }
    _initialized = true;
}

