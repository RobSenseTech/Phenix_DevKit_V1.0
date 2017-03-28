/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>


#include "AP_HAL_PX4.h"
#include "AP_HAL_PX4_Namespace.h"
#include "HAL_PX4_Class.h"
#include "Scheduler.h"
#include "UARTDriver.h"
#include "Storage.h"
#include "RCInput.h"
#include "RCOutput.h"
#include "AnalogIn.h"
#include "Util.h"
#include "GPIO.h"
#include "I2CDriver.h"

#include <AP_HAL_Empty/AP_HAL_Empty.h>
#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>

#include <stdlib.h>
#include <stdio.h>
#include <hrt/drv_hrt.h>

using namespace PX4;

static PX4I2CDriver i2cDriver;
static Empty::SPIDeviceManager spiDeviceManager;
//static Empty::GPIO gpioDriver;

static PX4Scheduler schedulerInstance;
static PX4Storage storageDriver;
static PX4RCInput rcinDriver;
static PX4RCOutput rcoutDriver;
static PX4AnalogIn analogIn;
static PX4Util utilInstance;
static PX4GPIO gpioDriver;

static Empty::I2CDeviceManager i2c_mgr_instance;

#define UARTA_DEFAULT_DEVICE "/dev/uartns2"         //uart4 maclink console
#define UARTB_DEFAULT_DEVICE "/dev/uartns3"         //1st GPS
#define UARTC_DEFAULT_DEVICE "/dev/uartns0"         //uart2 telem1
#define UARTD_DEFAULT_DEVICE "/dev/uartns1"         //uart3 telem2
#define UARTE_DEFAULT_DEVICE "/dev/uartns4"         //uart6 2and GPS

// 3 UART drivers, for GPS plus two mavlink-enabled devices
static PX4UARTDriver uartADriver(UARTA_DEFAULT_DEVICE, "APM_uartA");
static PX4UARTDriver uartBDriver(UARTB_DEFAULT_DEVICE, "APM_uartB");
static PX4UARTDriver uartCDriver(UARTC_DEFAULT_DEVICE, "APM_uartC");
static PX4UARTDriver uartDDriver(UARTD_DEFAULT_DEVICE, "APM_uartD");
static PX4UARTDriver uartEDriver(UARTE_DEFAULT_DEVICE, "APM_uartE");

HAL_PX4::HAL_PX4() :
    AP_HAL::HAL(
        &uartADriver,  /* uartA */
        &uartBDriver,  /* uartB */
        &uartCDriver,  /* uartC */
        &uartDDriver,  /* uartD */
        &uartEDriver,  /* uartE */
        &i2c_mgr_instance,
        &i2cDriver, /* i2c */
        NULL,   /* only one i2c */
        NULL,   /* only one i2c */
        &spiDeviceManager, /* spi */
        &analogIn, /* analogin */
        &storageDriver, /* storage */
        &uartADriver, /* console */
        &gpioDriver, /* gpio */
        &rcinDriver,  /* rcinput */
        &rcoutDriver, /* rcoutput */
        &schedulerInstance, /* scheduler */
        &utilInstance, /* util */
        NULL)    /* no onboard optical flow */
{}

bool _px4_thread_should_exit = false;        /**< Daemon exit flag */
static bool thread_running = false;        /**< Daemon status flag */
static TaskHandle_t daemon_task;                /**< Handle of daemon task / thread */
bool px4_ran_overtime;

extern const AP_HAL::HAL& hal;

/*
  set the priority of the main APM task
 */
void hal_px4_set_priority(uint8_t priority)
{
//    vTaskPrioritySet(daemon_task, priority);
}

/*
  this is called when loop() takes more than 1 second to run. If that
  happens then something is blocking for a long time in the main
  sketch - probably waiting on a low priority driver. Set the priority
  of the APM task low to let the driver run.
 */
static void loop_overtime(xTimerHandle xTimer, void *)
{
    hal_px4_set_priority(APM_OVERTIME_PRIORITY);
    px4_ran_overtime = true;
}

static AP_HAL::HAL::Callbacks* g_callbacks;

static void main_loop(void *pvParameters)
{
    hal.uartA->begin(115200);
    hal.uartB->begin(57600);
    hal.uartC->begin(57600);
    hal.uartD->begin(57600);
    hal.uartE->begin(57600);
    hal.scheduler->init();
    hal.rcin->init();
    hal.rcout->init();
//    hal.analogin->init();
 //   hal.gpio->init();

    Print_Info("in main loop\n");

    /*
      run setup() at low priority to ensure CLI doesn't hang the
      system, and to allow initial sensor read loops to run
     */
    hal_px4_set_priority(APM_STARTUP_PRIORITY);

    schedulerInstance.hal_initialized();

    g_callbacks->setup();
    hal.scheduler->system_initialized();

//    struct hrt_call loop_overtime_call;
    xTimerHandle loop_overtime_call;

    thread_running = true;

    /*
      switch to high priority for main loop
     */
    hal_px4_set_priority(APM_MAIN_PRIORITY);

//    loop_overtime_call = xTimerCreate("loop overtime", 100/portTICK_RATE_MS, pdTRUE, NULL, loop_overtime, NULL);
 //   xTimerStart(loop_overtime_call, portMAX_DELAY);
    Print_Info("main loop start thread\n");
    while (!_px4_thread_should_exit) {
        
        /*
          this ensures a tight loop waiting on a lower priority driver
          will eventually give up some time for the driver to run. It
          will only ever be called if a loop() call runs for more than
          0.1 second
        hrt_call_after(&loop_overtime_call, 100000, (hrt_callout)loop_overtime, NULL);
         */

        g_callbacks->loop();//call AP_Scheduler::run in this func

        if (px4_ran_overtime) {
            /*
              we ran over 1s in loop(), and our priority was lowered
              to let a driver run. Set it back to high priority now.
             */
            hal_px4_set_priority(APM_MAIN_PRIORITY);
            px4_ran_overtime = false;
        }


        /*
          give up 250 microseconds of time, to ensure drivers get a
          chance to run. This relies on the accurate semaphore wait
          using hrt in semaphore.cpp
         */
        hal.scheduler->delay_microseconds(250);
    }
    thread_running = false;
}

static void usage(void)
{
    Print_Info("Usage: %s [options] {start,stop,status}\n", SKETCHNAME);
    Print_Info("Options:\n");
    Print_Info("\t-d  DEVICE         set terminal device (default %s)\n", UARTA_DEFAULT_DEVICE);
    Print_Info("\t-d2 DEVICE         set second terminal device (default %s)\n", UARTC_DEFAULT_DEVICE);
    Print_Info("\t-d3 DEVICE         set 3rd terminal device (default %s)\n", UARTD_DEFAULT_DEVICE);
    Print_Info("\t-d4 DEVICE         set 2nd GPS device (default %s)\n", UARTE_DEFAULT_DEVICE);
    Print_Info("\n");
}


void HAL_PX4::run(int argc, char * const argv[], Callbacks* callbacks) const
{
    int i;
    const char *deviceA = UARTA_DEFAULT_DEVICE;
    const char *deviceC = UARTC_DEFAULT_DEVICE;
    const char *deviceD = UARTD_DEFAULT_DEVICE;
    const char *deviceE = UARTE_DEFAULT_DEVICE;

    if (argc < 1) {
        Print_Info("%s: missing command (try '%s start')", 
               SKETCHNAME, SKETCHNAME);
        usage();
        return;
    }

    configASSERT(callbacks);
    g_callbacks = callbacks;

    for (i=0; i<argc; i++) {
        if (strcmp(argv[i], "start") == 0) {
            if (thread_running) {
                Print_Info("%s already running\n", SKETCHNAME);
                /* this is not an error */
                return;
            }

            uartADriver.set_device_path(deviceA);
            uartCDriver.set_device_path(deviceC);
            uartDDriver.set_device_path(deviceD);
            uartEDriver.set_device_path(deviceE);
            Print_Info("Starting %s uartA=%s uartC=%s uartD=%s uartE=%s\n", 
                   SKETCHNAME, deviceA, deviceC, deviceD, deviceE);

            _px4_thread_should_exit = false;

            xTaskCreate(main_loop, "main loop", APM_MAIN_THREAD_STACK_SIZE, NULL, APM_MAIN_PRIORITY, &daemon_task);

            return;
        }

        if (strcmp(argv[i], "stop") == 0) {
            _px4_thread_should_exit = true;
            return;
        }
 
        if (strcmp(argv[i], "status") == 0) {
            if (_px4_thread_should_exit && thread_running) {
                Print_Info("\t%s is exiting\n", SKETCHNAME);
            } else if (thread_running) {
                Print_Info("\t%s is running\n", SKETCHNAME);
            } else {
                Print_Info("\t%s is not started\n", SKETCHNAME);
            }
            return;
        }

        if (strcmp(argv[i], "-d") == 0) {
            // set terminal device
            if (argc > i + 1) {
                deviceA = strdup(argv[i+1]);
            } else {
                Print_Info("missing parameter to -d DEVICE\n");
                usage();
                return;
            }
        }

        if (strcmp(argv[i], "-d2") == 0) {
            // set uartC terminal device
            if (argc > i + 1) {
                deviceC = strdup(argv[i+1]);
            } else {
                Print_Warn("missing parameter to -d2 DEVICE\n");
                usage();
                return;
            }
        }

        if (strcmp(argv[i], "-d3") == 0) {
            // set uartD terminal device
            if (argc > i + 1) {
                deviceD = strdup(argv[i+1]);
            } else {
                Print_Warn("missing parameter to -d3 DEVICE\n");
                usage();
                return;
            }
        }

        if (strcmp(argv[i], "-d4") == 0) {
            // set uartE 2nd GPS device
            if (argc > i + 1) {
                deviceE = strdup(argv[i+1]);
            } else {
                Print_Warn("missing parameter to -d4 DEVICE\n");
                usage();
                return;
            }
        }
    }
 
    usage();
    return;
}

const AP_HAL::HAL& AP_HAL::get_HAL() {
    static const HAL_PX4 hal_px4;
    return hal_px4;
}

