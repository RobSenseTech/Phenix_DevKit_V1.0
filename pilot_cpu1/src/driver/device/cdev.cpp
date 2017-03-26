#include "device/cdev.h"
#include "drv_device.h"

namespace device
{

/*
 * The standard NuttX operation dispatch table can't call C++ member functions
 * directly, so we have to bounce them through this dispatch table.
 */
static int	cdev_open(Handle_t *pHandle);
static int	cdev_close(Handle_t *pHandle);
static size_t	cdev_read(Handle_t *pHandle, char *pcBuffer, size_t xBufLen);
static size_t	cdev_write(Handle_t *pHandle, const char *pcBuffer, size_t xBufLen);
static int cdev_seek(Handle_t *pHandle, off_t xOffset, int iWhence);
static int	cdev_ioctl(Handle_t *pHandle, int iCmd, void *pvArg);
static int	cdev_poll(Handle_t *pHandle, struct pollfd *fds, bool setup);

/**
 * Character device indirection table.
 *
 * Every cdev we register gets the same function table; we use the private data
 * field in the inode to store the instance pointer.
 *
 * Note that we use the GNU extension syntax here because we don't get designated
 * initialisers in gcc 4.6.
 */
/* C++结构体初始化必须按成员定义的顺序 */
const DriverOps_t CDev::fops = {
open	: cdev_open,
close	: cdev_close,
read	: cdev_read,
write	: cdev_write,
ioctl	: cdev_ioctl,
poll	: cdev_poll,
lseek	: cdev_seek,
};

CDev::CDev(const char *name,
	   const char *devname, /*驱动路径名*/
	   int irq) : /*irq没用,防止编译问题所以留着*/
	// public
	// protected
	_pub_blocked(false),
	// private
	_devname(devname),
	_registered(false),
	_open_count(0)
{
	for (unsigned i = 0; i < _max_pollwaiters; i++) {
		_pollset[i] = NULL;
	}

     _lock = xSemaphoreCreateMutex(); 
}

CDev::~CDev()
{
	if (_registered) {
        DriverUnregister(_devname);
	}
    
    vSemaphoreDelete(_lock);
}

int
CDev::register_class_devname(const char *class_devname)
{
	int class_instance = 0;
    int ret = -ENOSPC;

	if (class_devname == NULL) {
        Print_Err("Device name is NULL\n");
		return -EINVAL;
	}

    while(class_instance < 4)
    {
        char name[MAX_DRIVER_NAME_LEN];

        snprintf(name, sizeof(name), "%s%d", class_devname, class_instance);
        ret = DriverRegister(name, &fops, (void *)this);
    
        if(ret == 0)
            break;
        
        class_instance++;
    }

   if(class_instance == 4)
      return ret; 

	return class_instance;
}

int
CDev::unregister_class_devname(const char *class_devname, unsigned class_instance)
{
	char name[32];
	snprintf(name, sizeof(name), "%s%u", class_devname, class_instance);
    Print_Info("unregister dev:%s\n", name);
	return DriverUnregister(name);
}

int
CDev::init()
{
    int ret = -ENOMEM;
	// now register the driver
	if (_devname != NULL) {
		ret = DriverRegister(_devname, &fops, (void *)this);

		if (ret < 0) {
			goto out;
		}

		_registered = true;
	}

out:
	return ret;
}

/*
 * Default implementations of the character device interface
 */
int
CDev::open(Handle_t *pHandle)
{
	int ret = 0;

    lock();
	/* increment the open count */
	_open_count++;

	if (_open_count == 1) {

		/* first-open callback may decline the open */
		ret = open_first(pHandle);

		if (ret != 0) {
			_open_count--;
		}
	}

    unlock();

	return ret;
}

int
CDev::open_first(Handle_t *pHandle)
{
	return 0;
}

int
CDev::close(Handle_t *pHandle)
{
	int ret = 0;

    lock();

	if (_open_count > 0) {
		/* decrement the open count */
		_open_count--;

		/* callback cannot decline the close */
		if (_open_count == 0) {
			ret = close_last(pHandle);
		}

	} else {
		ret = -EBADF;
	}

    unlock();

	return ret;
}

int
CDev::close_last(Handle_t *pHandle)
{
	return 0;
}

size_t
CDev::read(Handle_t *pHandle, char *pcBuffer, size_t xBufLen)
{
    Print_Err("No such system api\n");
	return -ENOSYS;
}

size_t
CDev::write(Handle_t *pHandle, const char *pcBuffer, size_t xBufLen)
{
    Print_Err("No such system api\n");
	return -ENOSYS;
}

int
CDev::seek(Handle_t *pHandle, off_t xOffset, int iWhence)
{
    Print_Err("No such system api\n");
	return -ENOSYS;
}

int
CDev::ioctl(Handle_t *pHandle, int iCmd, void *pvArg)
{
    switch(iCmd)
    {
        case DEVIOCGDEVICEID:
           return (int)_device_id.devid; //各个传感器在构造函数中会个给这个赋值,上层通过获取id号来配置不同传感器,原先放在Device类中            
    }
	return -ENOSYS;
}

int
CDev::poll(Handle_t *pHandle, struct pollfd *fds, bool setup)
{
	int ret = 0;

	/*
	 * Lock against pollnotify() (and possibly other callers)
	 */
	lock();

	if (setup) {
		/*
		 * Save the file pointer in the pollfd for the subclass'
		 * benefit.
		 */
		fds->priv = (void *)pHandle;

		/*
		 * Handle setup requests.
		 */
		ret = store_poll_waiter(fds);

		if (ret == 0) {

			/*
			 * Check to see whether we should send a poll notification
			 * immediately.
			 */
			fds->revents |= fds->events & poll_state(pHandle);

			/* yes? post the notification */
			if (fds->revents != 0) {
                portBASE_TYPE xHigherPriorityTaskWoken;
                xHigherPriorityTaskWoken = pdFALSE;
				xSemaphoreGiveFromISR(fds->sem, &xHigherPriorityTaskWoken);
			}
		}

	} else {
		/*
		 * Handle a teardown request.
		 */
		ret = remove_poll_waiter(fds);
	}

	unlock();

	return ret;
}

void
CDev::poll_notify(pollevent_t events)
{
	/* lock against poll() as well as other wakeups */
	irqstate_t state = irqsave();

	for (unsigned i = 0; i < _max_pollwaiters; i++)
		if (NULL != _pollset[i]) {
			poll_notify_one(_pollset[i], events);
		}

	irqrestore(state);
}

void
CDev::poll_notify_one(struct pollfd *fds, pollevent_t events)
{
	/* update the reported event set */
	fds->revents |= fds->events & events;

	/* if the state is now interesting, wake the waiter if it's still asleep */
	/* XXX semcount check here is a vile hack; counting semphores should not be abused as cvars */
	if (fds->revents != 0) {
        portBASE_TYPE xHigherPriorityTaskWoken;
        xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(fds->sem, &xHigherPriorityTaskWoken);
	}
}

pollevent_t
CDev::poll_state(Handle_t *pHandle)
{
	/* by default, no poll events to report */
	return 0;
}

int
CDev::store_poll_waiter(struct pollfd *fds)
{
	/*
	 * Look for a free slot.
	 */
	for (unsigned i = 0; i < _max_pollwaiters; i++) {
		if (NULL == _pollset[i]) {

			/* save the pollfd */
			_pollset[i] = fds;

			return 0;
		}
	}

    Print_Err("Too much poll waiters!!\n");
	return -ENOMEM;
}

int
CDev::remove_poll_waiter(struct pollfd *fds)
{
	for (unsigned i = 0; i < _max_pollwaiters; i++) {
		if (fds == _pollset[i]) {

			_pollset[i] = NULL;
			return 0;

		}
	}

	Print_Err("poll: bad fd state");
	return -EINVAL;
}

static int
cdev_open(Handle_t *pHandle)
{
	CDev *cdev = (CDev *)pHandle->xInode.pvPrivate;

	return cdev->open(pHandle);
}

static int
cdev_close(Handle_t *pHandle)
{
	CDev *cdev = (CDev *)pHandle->xInode.pvPrivate;

	return cdev->close(pHandle);
}

static size_t
cdev_read(Handle_t *pHandle, char *pcBuffer, size_t xBufLen)
{
	CDev *cdev = (CDev *)pHandle->xInode.pvPrivate;

	return cdev->read(pHandle, pcBuffer, xBufLen);
}

static size_t
cdev_write(Handle_t *pHandle, const char *pcBuffer, size_t xBufLen)
{
	CDev *cdev = (CDev *)pHandle->xInode.pvPrivate;

	return cdev->write(pHandle, pcBuffer, xBufLen);
}

static int 
cdev_seek(Handle_t *pHandle, off_t xOffset, int iWhence)
{
	CDev *cdev = (CDev *)pHandle->xInode.pvPrivate;

	return cdev->seek(pHandle, xOffset, iWhence);
}

static int
cdev_ioctl(Handle_t *pHandle, int iCmd, void *pvArg)
{
	CDev *cdev = (CDev *)pHandle->xInode.pvPrivate;

	return cdev->ioctl(pHandle, iCmd, pvArg);
}

static int
cdev_poll(Handle_t *pHandle, struct pollfd *fds, bool setup)
{
	CDev *cdev = (CDev *)pHandle->xInode.pvPrivate;

	return cdev->poll(pHandle, fds, setup);
}

} // namespace device

