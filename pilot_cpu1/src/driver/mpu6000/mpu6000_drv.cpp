#include <stdio.h>
#include "driver.h"
#include "drv_accel.h"
#include "drv_gyro.h"
#include "device/cdev.h"
#include "uORB/uORB.h"
#include "ringbuffer.h"
#include "uORB/topics/sensor_accel.h"
#include "task.h"

extern "C" {int mpu6000_main(int argc, char *argv[]);}
extern "C" {static void prvMpu6000Task(void *pvParameters);}

class MPU6000 : public device::CDev
{
public:
    MPU6000(const char *path);
    virtual ~MPU6000();

    virtual int init();
	virtual size_t	read(Handle_t *pHandle, char *pcBuffer, size_t xBufLen);
	virtual size_t	write(Handle_t *pHandle, const char *pcBuffer, size_t xBufLen);
	virtual int	ioctl(Handle_t *pHandle, int iCmd, void *pvArg);
	virtual int	poll(Handle_t *pHandle, struct pollfd *fds, bool setup);

    void measure_trampoline(void *arg);
    RingBuffer_t *_accel_reports;
private:
    struct hrt_call _call;
    int _accel_class_instance;
    orb_advert_t    _accel_topic;
    int _accel_orb_class_instance;
};

MPU6000::MPU6000(const char *path) :
    CDev("MPU6000_accel", path),
    _accel_class_instance(-1),
    _accel_orb_class_instance(-1)
{
   memset(&_call, 0, sizeof(_call));
}

MPU6000::~MPU6000()
{
    if(_accel_class_instance != -1)
        unregister_class_devname(ACCEL_BASE_DEVICE_PATH, _accel_class_instance);
}

static uint64_t hb = 0;
void MPU6000::measure_trampoline(void *arg)
{
    struct sensor_accel_s arb;

    hb++;
    arb.timestamp = hb;

  //  xRingBufferForce(_accel_reports, &arb, sizeof(arb)); 

    poll_notify(POLLIN);
    orb_publish(ORB_ID(sensor_accel), _accel_topic, &arb);
}

int MPU6000::init()
{
    int ret = -1;

    ret = CDev::init();
    if(ret != 0)
    {
        Print_Err("cdev init error\n");
        goto out;
    }

    _accel_class_instance = register_class_devname(ACCEL_BASE_DEVICE_PATH);

    _accel_reports = (RingBuffer_t *)pvPortMalloc(sizeof(RingBuffer_t));
    iRingBufferInit(_accel_reports, 2, sizeof(struct sensor_accel_s));

    struct sensor_accel_s arp;
    xRingBufferGet(_accel_reports, &arp, sizeof(arp));

    _accel_topic = orb_advertise_multi(ORB_ID(sensor_accel), &arp, &_accel_orb_class_instance, ORB_PRIO_HIGH);
    if(_accel_topic == NULL)
        Print_Err("ADVERT FAIL\n");

    hrt_call_every(&_call, 1000, 1000, (hrt_callout)&MPU6000::measure_trampoline, this);

out:
    return ret;
}

size_t MPU6000::read(Handle_t *pHandle, char *pcBuffer, size_t xBufLen)
{
    Print_Info("mpu6000 read\n");
    return 0;
}

size_t MPU6000::write(Handle_t *pHandle, const char *pcBuffer, size_t xBufLen)
{
    Print_Info("mpu6000 write\n");
    return 0;
}

int MPU6000::ioctl(Handle_t *pHandle, int iCmd, void *pvArg)
{
    Print_Info("mpu6000 ioctl\n");
    return 0;
}

int	MPU6000::poll(Handle_t *pHandle, struct pollfd *fds, bool setup)
{
    Print_Info("mpu6000 poll\n");
    return 0;
}

static void prvMpu6000Task(void *pvParameters)
{
    int accel_sub = orb_subscribe_multi(ORB_ID(sensor_accel), 0);
    int ret = 0;
    struct sensor_accel_s accel0;

    while(1)
    {
        struct pollfd fds[1];
        memset(fds, 0, sizeof(fds));
        fds[0].fd = accel_sub;
        fds[0].events = POLLIN;

//        Print_Info("poll start:accel_sub=%x\n", (int)accel_sub);
        ret = poll(fds, sizeof(fds)/sizeof(fds[0]), 500);
        if(ret < 0)
            Print_Err("poll error\n");
        else if(ret == 0)
            Print_Warn("poll timeout");
        else
        {
            if(fds[0].revents & POLLIN)
            {
                orb_copy(ORB_ID(sensor_accel), accel_sub, &accel0);
                Print_Info("get data:%d size=%x\n", accel0.timestamp, sizeof(struct sensor_accel_s));
            }
            else
            {
                Print_Warn("revents=%x\n", fds[0].revents);
            }
        }

		vTaskDelay( 300 / portTICK_RATE_MS );
    }
}

static void prvMpu6000Task2(void *pvParameters)
{
    int accel_sub = orb_subscribe_multi(ORB_ID(sensor_accel), 0);
    int ret = 0;
    struct sensor_accel_s accel0;

    while(1)
    {
        struct pollfd fds[1];
        memset(fds, 0, sizeof(fds));
        fds[0].fd = accel_sub;
        fds[0].events = POLLIN;

//        Print_Info("poll start:accel_sub=%x\n", (int)accel_sub);
        ret = poll(fds, sizeof(fds)/sizeof(fds[0]), 500);
        if(ret < 0)
            Print_Err("poll error\n");
        else if(ret == 0)
            Print_Warn("poll timeout");
        else
        {
            if(fds[0].revents & POLLIN)
            {
                orb_copy(ORB_ID(sensor_accel), accel_sub, &accel0);
                Print_Info("task 2 get data:%d size=%x\n", accel0.timestamp, sizeof(struct sensor_accel_s));
            }
            else
            {
                Print_Warn("revents=%x\n", fds[0].revents);
            }
        }

		vTaskDelay( 300 / portTICK_RATE_MS );
    }
}

int mpu6000_main(int argc, char *argv[])
{
    int iFd = 0;
    MPU6000 *accel_dev = new MPU6000("/dev/mpu6000");

    accel_dev->init();
   
	iFd = open(ACCEL0_DEVICE_PATH, 0);
    if(-1 == iFd)
    {
        Print_Err("open %s failed\n", ACCEL0_DEVICE_PATH);
        return -1;
    }

//    Print_Err("argv=%s\n", argv[1]);

    write(iFd, 0, 0);
    read(iFd, 0, 0);
    ioctl(iFd, 0, 0);

//    Print_Info("create uorb recv task:%d\n", xTaskCreate(prvMpu6000Task, "uorb-recv", 4000, NULL, 1, NULL));
 //   Print_Info("create uorb recv task2:%d\n", xTaskCreate(prvMpu6000Task2, "uorb-recv2", 4000, NULL, 1, NULL));
#if 0
    delete accel_dev;

    iFd = open(ACCEL0_DEVICE_PATH, 0);
    if(-1 == iFd)
    {
        Print_Err("open %s failed\n", ACCEL0_DEVICE_PATH);
    }
#endif

    return 0;
}

