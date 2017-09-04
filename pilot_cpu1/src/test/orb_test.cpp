#include "device/cdev.h"
#include "uORB/uORB.h"

class ORBTest : public device::CDev
{
public:
    ORBTest(const char *path);
    virtual ~ORBTest();

    virtual int init();
	virtual size_t	read(Handle_t *pHandle, char *pcBuffer, size_t xBufLen);
	virtual size_t	write(Handle_t *pHandle, const char *pcBuffer, size_t xBufLen);
	virtual int	ioctl(Handle_t *pHandle, int iCmd, void *pvArg);

private:
    int _class_instance;
    orb_advert_t    _test_topic;
    int _orb_class_instance;

};

ORBTest::ORBTest(const char *path) :
    CDev("ORBTest_accel", path),
    _class_instance(-1)
{
   pilot_info("ORBTest construct func\n");
}

ORBTest::~ORBTest()
{
    if(_class_instance != -1)
        unregister_class_devname("/dev/orbtest", _class_instance);
}

int ORBTest::init()
{
    int ret = -1;

    ret = CDev::init();
    if(ret != 0)
    {
        pilot_err("cdev init error\n");
        goto out;
    }

    _class_instance = register_class_devname("/dev/orbtest");
    pilot_info("_class_instance = %d\n", _class_instance);

    struct sensor_accel_s ;

out:
    return ret;
}

size_t ORBTest::read(Handle_t *pHandle, char *pcBuffer, size_t xBufLen)
{
    pilot_info("ORBTest read\n");
    return 0;
}

size_t ORBTest::write(Handle_t *pHandle, const char *pcBuffer, size_t xBufLen)
{
    pilot_info("ORBTest write\n");
    return 0;
}

int ORBTest::ioctl(Handle_t *pHandle, int iCmd, void *pvArg)
{
    pilot_info("ORBTest ioctl\n");
    return 0;
}

void uorb_test()
{
    ORBTest *test_dev = new ORBTest("/dev/ORBTest");
    
    
}

