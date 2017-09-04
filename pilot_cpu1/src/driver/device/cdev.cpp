#include "device/cdev.h"
#include "drv_device.h"
#include <sys/ioctl.h>
#include "pilot_print.h"
#include "irq.h"

namespace device
{

/*
 * The standard NuttX operation dispatch table can't call C++ member functions
 * directly, so we have to bounce them through this dispatch table.
 */
static int	cdev_open(file_t *filp);
static int	cdev_close(file_t *filp);
static ssize_t	cdev_read(file_t *filp, char *buffer, size_t buflen);
static ssize_t	cdev_write(file_t *filp, const char *buffer, size_t buflen);
static off_t	cdev_seek(file_t *filp, off_t offset, int whence);
static int	cdev_ioctl(file_t *filp, int cmd, unsigned long arg);
static int	cdev_poll(file_t *filp, struct pollfd *fds, bool setup);


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
const struct file_operations CDev::fops = {
open	: cdev_open,
close	: cdev_close,
read	: cdev_read,
write	: cdev_write,
seek	: cdev_seek,
ioctl	: cdev_ioctl,
poll	: cdev_poll,
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
		unregister_driver(_devname);
	}
    
    vSemaphoreDelete(_lock);
}

int
CDev::register_class_devname(const char *class_devname)
{
	int class_instance = 0;
    int ret = -ENOSPC;

	if (class_devname == NULL) {
        pilot_err("Device name is NULL\n");
		return -EINVAL;
	}

    while(class_instance < 4)
    {
        char name[32];

        snprintf(name, sizeof(name), "%s%d", class_devname, class_instance);
		ret = register_driver(name, &fops, 0666, (void *)this);
    
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
	return unregister_driver(name);
}

int
CDev::init()
{
    int ret = -ENOMEM;
	// now register the driver
	if (_devname != NULL) {
		ret = register_driver(_devname, &fops, 0666, (void *)this);

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
CDev::open(file_t *filp)
{
	int ret = 0;

    lock();
	/* increment the open count */
	_open_count++;

	if (_open_count == 1) {

		/* first-open callback may decline the open */
		ret = open_first(filp);

		if (ret != 0) {
			_open_count--;
		}
	}

    unlock();

	return ret;
}

int
CDev::open_first(file_t *filp)
{
	return 0;
}

int
CDev::close(file_t *filp)
{
	int ret = 0;

    lock();

	if (_open_count > 0) {
		/* decrement the open count */
		_open_count--;

		/* callback cannot decline the close */
		if (_open_count == 0) {
			ret = close_last(filp);
		}

	} else {
		ret = -EBADF;
	}

    unlock();

	return ret;
}

int
CDev::close_last(file_t *filp)
{
	return 0;
}

ssize_t
CDev::read(file_t *filp, char *buffer, size_t buflen)
{
    pilot_err("No such system api\n");
	return -ENOSYS;
}

ssize_t
CDev::write(file_t *filp, const char *buffer, size_t buflen)
{
    pilot_err("No such system api\n");
	return -ENOSYS;
}

off_t
CDev::seek(file_t *filp, off_t offset, int whence)
{
    pilot_err("No such system api\n");
	return -ENOSYS;
}

int
CDev::ioctl(file_t *filp, int cmd, unsigned long arg)
{
    switch(cmd)
    {
        case DEVIOCGDEVICEID:
           return (int)_device_id.devid; //各个传感器在构造函数中会个给这个赋值,上层通过获取id号来配置不同传感器,原先放在Device类中            
    }
	return -ENOSYS;
}

int
CDev::poll(file_t *filp, struct pollfd *fds, bool setup)
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
		fds->priv = (void *)filp;

		/*
		 * Handle setup requests.
		 */
		ret = store_poll_waiter(fds);

		if (ret == 0) {

			/*
			 * Check to see whether we should send a poll notification
			 * immediately.
			 */
			fds->revents |= fds->events & poll_state(filp);

			/* yes? post the notification */
			if (fds->revents != 0) {
				sem_post(fds->sem);
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
	if ((fds->revents != 0) /*&& (fds->sem->semcount <= 0)*/) {
		sem_post(fds->sem);
	}
}

pollevent_t
CDev::poll_state(file_t *filp)
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

    pilot_err("Too much poll waiters!!\n");
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

	pilot_err("poll: bad fd state");
	return -EINVAL;
}

static int
cdev_open(file_t *filp)
{
	CDev *cdev = (CDev *)(filp->f_inode->i_private);

	return cdev->open(filp);
}

static int
cdev_close(file_t *filp)
{
	CDev *cdev = (CDev *)(filp->f_inode->i_private);

	return cdev->close(filp);
}

static ssize_t
cdev_read(file_t *filp, char *buffer, size_t buflen)
{
	CDev *cdev = (CDev *)(filp->f_inode->i_private);

	return cdev->read(filp, buffer, buflen);
}

static ssize_t
cdev_write(file_t *filp, const char *buffer, size_t buflen)
{
	CDev *cdev = (CDev *)(filp->f_inode->i_private);

	return cdev->write(filp, buffer, buflen);
}

static off_t
cdev_seek(file_t *filp, off_t offset, int whence)
{
	CDev *cdev = (CDev *)(filp->f_inode->i_private);

	return cdev->seek(filp, offset, whence);
}

static int
cdev_ioctl(file_t *filp, int cmd, unsigned long arg)
{
	CDev *cdev = (CDev *)(filp->f_inode->i_private);

	return cdev->ioctl(filp, cmd, arg);
}

static int
cdev_poll(file_t *filp, struct pollfd *fds, bool setup)
{
	CDev *cdev = (CDev *)(filp->f_inode->i_private);

	return cdev->poll(filp, fds, setup);
}

} // namespace device
