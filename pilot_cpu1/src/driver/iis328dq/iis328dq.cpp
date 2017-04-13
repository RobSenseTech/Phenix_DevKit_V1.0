#include "device/cdev.h"
#include "xil_cache.h"
#include "ringbuffer.h"
#include "drv_accel.h"
#include <uORB/uORB.h>
#include "conversion/rotation.h"
#include "FreeRTOS_Print.h"
#include "math.h"
#include "driver_define.h"
#include "drv_unistd/drv_unistd.h"
#include "board_config.h"
#include "Filter/LowPassFilter2p.h"
#include "Phx_define.h"
#include "timers.h"
#include "driver.h"

#include <unistd.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>

#define IIS328DQ_DEVICE_PATH "/dev/iis328dq"
#define IIS328DQ_EXT_DEVICE_PATH "/dev/iis328dq_ext"
#define IIS328DQ_MAX_INSTANCE 2 //若联飞控，目前最多一款传感器接两个


/* SPI protocol address bits */
#define DIR_READ				(1<<7)
#define DIR_WRITE				(0<<7)
#define ADDR_INCREMENT				(1<<6)

/* register addresses */
#define ADDR_WHO_AM_I			0x0F
#define WHO_I_AM				0x32

#define ADDR_CTRL_REG1			0x20
#define REG1_RATE_LP_MASK			0xF0 /* Mask to guard partial register update */

/* keep lowpass low to avoid noise issues */
#define RATE_50HZ_LP_37HZ		(0<<4) | (0<<3)
#define RATE_100HZ_LP_74HZ		(0<<4) | (1<<3)
#define RATE_400HZ_LP_292HZ		(1<<4) | (0<<3)
#define RATE_1000HZ_LP_780HZ	(1<<4) | (1<<3)

#define ADDR_CTRL_REG2			0x21
#define ADDR_CTRL_REG3			0x22
#define ADDR_CTRL_REG4			0x23
#define REG4_RANGE_MASK				0x30 /* Mask to guard partial register update */

#define ADDR_CTRL_REG5			0x24
#define ADDR_HP_FILTER_RESETE	0x25
#define ADDR_OUT_REFERENCE		0x26
#define ADDR_STATUS_REG			0x27
#define ADDR_OUT_X_L			0x28
#define ADDR_OUT_X_H			0x29
#define ADDR_OUT_Y_L			0x2A
#define ADDR_OUT_Y_H			0x2B
#define ADDR_OUT_Z_L			0x2C
#define ADDR_OUT_Z_H			0x2D
#define ADDR_INT1_CFG			0x30
#define ADDR_INT1_SRC			0x31
#define ADDR_INT1_TSH			0x32
#define ADDR_INT1_DURATION		0x33
#define ADDR_INT2_CFG			0x34
#define ADDR_INT2_SRC			0x35
#define ADDR_INT2_TSH			0x36
#define ADDR_INT2_DURATION		0x37


/* Internal configuration values */
#define REG1_POWER_NORMAL			((0<<7) | (0<<6) | (1<<5))
#define REG1_Z_ENABLE				(1<<2)
#define REG1_Y_ENABLE				(1<<1)
#define REG1_X_ENABLE				(1<<0)

#define REG4_BDU				(1<<7)
#define REG4_BLE				(1<<6)
#define REG4_FULL_SCALE_BITS	((1<<5) | (1<<4))
#define REG4_FULL_SCALE_2G		((0<<5) | (0<<4))
#define REG4_FULL_SCALE_4G		((0<<5) | (1<<4))
#define REG4_FULL_SCALE_8G		((1<<5) | (1<<4))

#define STATUS_ZYXOR			(1<<7)
#define STATUS_ZOR				(1<<6)
#define STATUS_YOR				(1<<5)
#define STATUS_XOR				(1<<4)
#define STATUS_ZYXDA			(1<<3)
#define STATUS_ZDA				(1<<2)
#define STATUS_YDA				(1<<1)
#define STATUS_XDA				(1<<0)

#define IIS328DQ_ACCEL_DEFAULT_RANGE_G			8
#define IIS328DQ_DEFAULT_RATE			1000
#define IIS328DQ_DEFAULT_ONCHIP_FILTER_FREQ		780
#define IIS328DQ_DEFAULT_DRIVER_FILTER_FREQ		30	

#define IIS328DQ_ONE_G					9.80665f

#ifdef PX4_SPI_BUS_EXT
#define EXTERNAL_BUS PX4_SPI_BUS_EXT
#else
#define EXTERNAL_BUS 0
#endif

/*
  we set the timer interrupt to run a bit faster than the desired
  sample rate and then throw away duplicates using the data ready bit.
  This time reduction is enough to cope with worst case timing jitter
  due to other timers
 */
#define IIS328DQ_TIMER_REDUCTION				200

extern "C" { __EXPORT int iis328dq_main(int argc, char *argv[]); }

class IIS328DQ : public device::CDev
{
public:
	IIS328DQ(int bus, const char* path, ESpi_device_id device, enum Rotation rotation);
	virtual ~IIS328DQ();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

	// print register dump
	void			print_registers();

	// trigger an error
	void			test_error();

protected:
	virtual int		probe();

private:

#ifdef USE_HRT
	struct hrt_call		_call;
#else
    xTimerHandle    _call;
#endif
	unsigned		_call_interval;

	RingBuffer_t	*_reports;

	struct accel_scale	_accel_scale;
	unsigned		_accel_range_m_s2;
	float			_accel_range_scale;
	unsigned		_accel_samplerate;
	unsigned		_accel_onchip_filter_bandwidth;

	orb_advert_t		_accel_topic;
	int			_orb_class_instance;
	int			_class_instance;


	unsigned		_read;

	uint8_t			_register_wait;

	LowPassFilter2p<float>	_accel_filter_x;
	LowPassFilter2p<float>	_accel_filter_y;
	LowPassFilter2p<float>	_accel_filter_z;


	enum Rotation		_rotation;

	struct SDeviceViaSpi _devInstance;

	// values used to
	float			_last_accel[3];
	uint8_t			_constant_accel_count;

	// this is used to support runtime checking of key
	// configuration registers to detect SPI bus errors and sensor
	// reset
#define IIS328DQ_NUM_CHECKED_REGISTERS 6
	static const uint8_t	_checked_registers[IIS328DQ_NUM_CHECKED_REGISTERS];
	uint8_t			_checked_values[IIS328DQ_NUM_CHECKED_REGISTERS];
	uint8_t			_checked_next;

	/**
	 * Start automatic measurement.
	 */
	void			start();

	/**
	 * Stop automatic measurement.
	 */
	void			stop();

	/**
	 * Reset the driver
	 */
	void			reset();

	/**
	 * disable I2C on the chip
	 */
	void			disable_i2c();

	/**
	 * Get the internal / external state
	 *
	 * @return true if the sensor is not on the main MCU board
	 */
	bool			is_external() { return 0;} /*(_bus == EXTERNAL_BUS);*/

	/**
	 * Static trampoline from the hrt_call context; because we don't have a
	 * generic hrt wrapper yet.
	 *
	 * Called by the HRT in interrupt context at the specified rate if
	 * automatic polling is enabled.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
#ifdef USE_HRT
	static void		measure_trampoline(void *arg);
#else

	static void		measure_trampoline(xTimerHandle xTimer, void *arg);
#endif

	/**
	 * check key registers for correct values
	 */
	void			check_registers(void);

	/**
	 * Fetch measurements from the sensor and update the report ring.
	 */
	void			measure();

	/**
	 * Read a register from the IIS328DQ
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	uint8_t			read_reg(unsigned reg);

	/**
	 * Write a register in the IIS328DQ
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_reg(unsigned reg, uint8_t value);

	/**
	 * Modify a register in the IIS328DQ
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg		The register to modify.
	 * @param clearbits	Bits in the register to clear.
	 * @param setbits	Bits in the register to set.
	 */
	void			modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	/**
	 * Write a register in the IIS328DQ, updating _checked_values
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_checked_reg(unsigned reg, uint8_t value);

	/**
	 * Set the IIS328DQ measurement range.
	 *
	 * @param max_dps	The measurement range is set to permit reading at least
	 *			this rate in degrees per second.
	 *			Zero selects the maximum supported range.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			set_range(unsigned max_g);//full-scale selection

	/**
	 * Set the IIS328DQ internal sampling frequency.
	 *
	 * @param frequency	The internal sampling frequency is set to not less than
	 *			this value.
	 *			Zero selects the maximum rate supported.
	 * @return		OK if the value can be supported.
	 */
	int			set_samplerate(unsigned frequency, unsigned bandwidth);

	/**
	 * Set the lowpass filter of the driver
	 *
	 * @param samplerate	The current samplerate
	 * @param frequency	The cutoff frequency for the lowpass filter
	 */
	void			set_driver_lowpass_filter(float samplerate, float bandwidth);

	/**
	 * Self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	 int 			self_test();

	/* this class does not allow copying */
	IIS328DQ(const IIS328DQ&);
	IIS328DQ operator=(const IIS328DQ&);
};

/*
  list of registers that will be checked in check_registers(). Note
  that ADDR_WHO_AM_I must be first in the list.
 */
const uint8_t IIS328DQ::_checked_registers[IIS328DQ_NUM_CHECKED_REGISTERS] = { ADDR_WHO_AM_I,
                                                                           ADDR_CTRL_REG1,
                                                                           ADDR_CTRL_REG2,
                                                                           ADDR_CTRL_REG3,
                                                                           ADDR_CTRL_REG4,
                                                                           ADDR_CTRL_REG5,};

IIS328DQ::IIS328DQ(int bus, const char* path, ESpi_device_id device, enum Rotation rotation) :
	CDev("iis328dq", path),
	_call_interval(0),
	_reports(NULL),
	_accel_range_m_s2(0.0f),
	_accel_range_scale(0.0f),
	_accel_samplerate(0),
	_accel_onchip_filter_bandwidth(0),
	_accel_topic(NULL),
	_orb_class_instance(-1),
	_class_instance(-1),
	_read(0),
	_register_wait(0),
	_accel_filter_x(IIS328DQ_DEFAULT_RATE, IIS328DQ_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_y(IIS328DQ_DEFAULT_RATE, IIS328DQ_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_z(IIS328DQ_DEFAULT_RATE, IIS328DQ_DEFAULT_DRIVER_FILTER_FREQ),
	_rotation(rotation),
	_constant_accel_count(0),
	_checked_next(0)
{

    _device_id.devid_s.devtype = DRV_ACC_DEVTYPE_IIS328DQ;

	// default scale factors
	_accel_scale.x_offset = 0;
	_accel_scale.x_scale  = 1.0f;
	_accel_scale.y_offset = 0;
	_accel_scale.y_scale  = 1.0f;
	_accel_scale.z_offset = 0;
	_accel_scale.z_scale  = 1.0f;

	_devInstance.spi_id = bus;
	DeviceViaSpiCfgInitialize(&_devInstance,
							device, 
							"iis328dq",
							ESPI_CLOCK_MODE_2,
							(11*1000*1000));
}

IIS328DQ::~IIS328DQ()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != NULL) {
		vPortFree(_reports);
	}

	if (_class_instance != -1)
		unregister_class_devname(ACCEL_BASE_DEVICE_PATH, _class_instance);
}

int
IIS328DQ::init()
{
	int ret = DEV_FAILURE;

	/* do SPI init (and probe) first */
	if (CDev::init() != OK)
		goto out;
	
	if (probe() != OK)
		goto out;

	/* allocate basic report buffers */
	if (_reports != NULL) {
		vPortFree(_reports);
		_reports = NULL;
	}
	/* allocate basic report buffers */
	_reports = (RingBuffer_t *) pvPortMalloc (sizeof(RingBuffer_t));
	iRingBufferInit(_reports, 2, sizeof(accel_report));

	if (_reports == NULL)
		goto out;

	_class_instance = register_class_devname(ACCEL_BASE_DEVICE_PATH);

	reset();

	measure();

	/* advertise sensor topic, measure manually to initialize valid report */
	struct accel_report arp;
	xRingBufferGet(_reports, &arp, sizeof(arp));

	_accel_topic = orb_advertise_multi(ORB_ID(sensor_accel), &arp,
		&_orb_class_instance, (is_external()) ? ORB_PRIO_VERY_HIGH : ORB_PRIO_DEFAULT);

	if (_accel_topic == NULL) {
		DEVICE_DEBUG("failed to create sensor_accel publication");
	}

	ret = OK;
out:
	return ret;
}

int
IIS328DQ::probe()
{
	/* read dummy value to void to clear SPI statemachine on sensor */
	(void)read_reg(ADDR_WHO_AM_I);

	bool success = false;
	uint8_t v = 0;

	/* verify that the device is attached and functioning, accept
	 * iis328dq*/
	if ((v=read_reg(ADDR_WHO_AM_I)) == WHO_I_AM) {
		success = true;
	}
	
	if (success) {
		_checked_values[0] = v;
		return OK;
	}

	return -EIO;
}

ssize_t
IIS328DQ::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct accel_report);
	struct accel_report *abuf = reinterpret_cast<struct accel_report *>(buffer);//指针类型强转
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1)
		return -ENOSPC;

	/* if automatic measurement is enabled */
	if (_call_interval > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the measurement code while we are doing this;
		 * we are careful to avoid racing with it.
		 */
		while (count--) {
			if (0 == xRingBufferGet(_reports, abuf, sizeof(*abuf))) {
				ret += sizeof(*abuf);
				abuf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement */
	vRingBufferFlush(_reports);
	measure();

	/* measurement will have generated a report, copy it out */
	if (0 == xRingBufferGet(_reports, abuf, sizeof(*abuf)))  {
		ret = sizeof(*abuf);
	}

	return ret;
}

int
IIS328DQ::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

				/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_call_interval = 0;
				return OK;

				/* external signalling not supported */
			case SENSOR_POLLRATE_EXTERNAL:

				/* zero would be bad */
			case 0:
				return -EINVAL;

				/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT:
				return ioctl(filp, SENSORIOCSPOLLRATE, IIS328DQ_DEFAULT_RATE);

				/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_call_interval == 0);

					/* convert hz to hrt interval via microseconds */
					unsigned ticks = 1000000 / arg;

					/* check against maximum sane rate */
					if (ticks < 1000)
						return -EINVAL;

					/* update interval for next measurement */
					/* XXX this is a bit shady, but no other way to adjust... */
					_call_interval = ticks;

                #ifdef USE_HRT
                    _call.period = _call_interval - IIS328DQ_TIMER_REDUCTION;
                #endif

					/* adjust filters */
					float cutoff_freq_hz = _accel_filter_x.get_cutoff_freq();
					float sample_rate = 1.0e6f/ticks;
					set_driver_lowpass_filter(sample_rate, cutoff_freq_hz);

					/* if we need to start the poll state machine, do it */
					if (want_start)
						start();

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_call_interval == 0)
			return SENSOR_POLLRATE_MANUAL;

		return 1000000 / _call_interval;

	case SENSORIOCSQUEUEDEPTH: {
		/* lower bound is mandatory, upper bound is a sanity check */
		if ((arg < 1) || (arg > 100))
			return -EINVAL;

		irqstate_t flags = irqsave();
		if (!xRingBufferResize(_reports, arg)) {
			irqrestore(flags);
			return -ENOMEM;
		}
		irqrestore(flags);

		return OK;
	}

	case SENSORIOCGQUEUEDEPTH:
		return iRingBufferSize(_reports);

	case SENSORIOCRESET:
		reset();
		return OK;

	case ACCELIOCSSAMPLERATE:
		return set_samplerate(arg, _accel_onchip_filter_bandwidth);

	case ACCELIOCGSAMPLERATE:
		return _accel_samplerate;

	case ACCELIOCSLOWPASS: {
		// set the software lowpass cut-off in Hz
		float cutoff_freq_hz = arg;
		float sample_rate = 1.0e6f / _call_interval;
		set_driver_lowpass_filter(sample_rate, cutoff_freq_hz);

		return OK;
	}

	case ACCELIOCGLOWPASS:
		return static_cast<int>(_accel_filter_x.get_cutoff_freq());

	case ACCELIOCSSCALE:
		/* copy scale in */
		memcpy(&_accel_scale, (struct accel_scale *) arg, sizeof(_accel_scale));
		return OK;

	case ACCELIOCGSCALE:
		/* copy scale out */
		memcpy((struct accel_scale *) arg, &_accel_scale, sizeof(_accel_scale));
		return OK;

	case ACCELIOCSRANGE:
		/* arg should be in dps accel	*/
		return set_range(arg);

	case ACCELIOCGRANGE:
		/* convert to m/s^2 and return rounded in G */
		return (unsigned long)((_accel_range_m_s2)/IIS328DQ_ONE_G + 0.5f);

	case ACCELIOCSELFTEST:
		return self_test();

	case ACCELIOCSHWLOWPASS:
		return set_samplerate(_accel_samplerate, arg);

	case ACCELIOCGHWLOWPASS:
		return _accel_onchip_filter_bandwidth;//set_samplerate函数中赋值

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}

uint8_t
IIS328DQ::read_reg(unsigned reg)
{
	uint8_t cmd[2];

	cmd[0] = reg | DIR_READ;
	cmd[1] = 0;

	SpiTransfer(&_devInstance, cmd, cmd, sizeof(cmd));

	return cmd[1];
}

void
IIS328DQ::write_reg(unsigned reg, uint8_t value)
{
	uint8_t	cmd[2];

	cmd[0] = reg | DIR_WRITE;
	cmd[1] = value;

	SpiTransfer(&_devInstance, cmd, NULL, sizeof(cmd));
}

void
IIS328DQ::write_checked_reg(unsigned reg, uint8_t value)
{
	write_reg(reg, value);
	for (uint8_t i=0; i<IIS328DQ_NUM_CHECKED_REGISTERS; i++) {
		if (reg == _checked_registers[i]) {
			_checked_values[i] = value;
		}
	}
}


void
IIS328DQ::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val;

	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_checked_reg(reg, val);
}

int
IIS328DQ::set_range(unsigned max_g)
{
	uint8_t setbits = 0;
	uint8_t clearbits = REG4_FULL_SCALE_BITS;
	float new_scale_g_digit = 0.0f;

	if (max_g == 0)
		max_g = 8;

	if (max_g <= 2) {
		_accel_range_m_s2 = 2;
		setbits |= REG4_FULL_SCALE_2G;
		new_scale_g_digit = 0.061e-3f;

	} else if (max_g <= 4) {
		_accel_range_m_s2 = 4;
		setbits |= REG4_FULL_SCALE_4G;
		new_scale_g_digit = 0.122e-3f;

	} else if (max_g <= 8) {
		_accel_range_m_s2 = 8;
		setbits |= REG4_FULL_SCALE_8G;
		new_scale_g_digit = 0.244e-3f;

	}else {
		return -EINVAL;
	}

	_accel_range_scale = new_scale_g_digit * IIS328DQ_ONE_G;

	modify_reg(ADDR_CTRL_REG4, clearbits, setbits);

	return OK;

}

int
IIS328DQ::set_samplerate(unsigned frequency, unsigned bandwidth)
{
	uint8_t bits = REG1_POWER_NORMAL | REG1_Z_ENABLE | REG1_Y_ENABLE | REG1_X_ENABLE;

	if (frequency == 0 || frequency == IIS328DQ_DEFAULT_RATE) {
		frequency = 1000;
	}

	/*
	 * Use limits good for H or non-H models. Rates are slightly different
	 * for L3G4200D part but register settings are the same.
	 */
	if (frequency <= 50) {
		_accel_samplerate =50;
		_accel_onchip_filter_bandwidth = 37;
		bits |= RATE_50HZ_LP_37HZ;
	} else if (frequency <= 100) {
		_accel_samplerate = 100;
		_accel_onchip_filter_bandwidth = 70;
		bits |= RATE_100HZ_LP_74HZ;
		
	} else if (frequency <= 400) {
		_accel_samplerate = 400;

		_accel_onchip_filter_bandwidth = 292;
		bits |= RATE_400HZ_LP_292HZ;
		
	} else if (frequency <= 1000) {
		_accel_samplerate = 1000;
		_accel_onchip_filter_bandwidth = 780;
		bits |= RATE_1000HZ_LP_780HZ;

	} else {
		return -EINVAL;
	}

	write_checked_reg(ADDR_CTRL_REG1, bits);

	return OK;
}

void
IIS328DQ::set_driver_lowpass_filter(float samplerate, float bandwidth)
{
	_accel_filter_x.set_cutoff_frequency(samplerate, bandwidth);
	_accel_filter_y.set_cutoff_frequency(samplerate, bandwidth);
	_accel_filter_z.set_cutoff_frequency(samplerate, bandwidth);
}

void
IIS328DQ::start()
{
	/* make sure we are stopped first */
	//stop();

	/* reset the report ring */
	vRingBufferFlush(_reports);

	/* start polling at the specified rate */
#ifdef USE_HRT
	hrt_call_every(&_call,
                       1000,
                       _call_interval - IIS328DQ_TIMER_REDUCTION,
                       (hrt_callout)&IIS328DQ::measure_trampoline, this);
#else
    int ticks = USEC2TICK(_call_interval);
    if(ticks == 0)
        ticks = 1;//定时器时间间隔不可为0
	/* reset the report ring and state machine */
	_call = xTimerCreate("accel timer", USEC2TICK(_call_interval), pdTRUE, NULL, &IIS328DQ::measure_trampoline, this);
	xTimerStart(_call, portMAX_DELAY);
#endif
}

void
IIS328DQ::stop()
{
#ifdef USE_HRT
	hrt_cancel(&_call);
#else
    xTimerDelete(_call, portMAX_DELAY);
#endif
}

void
IIS328DQ::disable_i2c(void)
{
#if 0
	uint8_t retries = 10;
	while (retries--) {
		// add retries
		uint8_t a = read_reg(0x05);
		write_reg(0x05, (0x20 | a));
		if (read_reg(0x05) == (a | 0x20)) {
			// this sets the I2C_DIS bit on the
			// IIS328DQ. The l3gd20 datasheet doesn't
			// mention this register, but it does seem to
			// accept it.
			write_checked_reg(ADDR_LOW_ODR, 0x08);
			return;
		}
	}
	DEVICE_DEBUG("FAILED TO DISABLE I2C");
#endif
}

void
IIS328DQ::reset()
{
	/* set default configuration */
	write_checked_reg(ADDR_CTRL_REG1,
                          REG1_POWER_NORMAL | REG1_Z_ENABLE | REG1_Y_ENABLE | REG1_X_ENABLE);
	write_checked_reg(ADDR_CTRL_REG2, 0);		/* disable high-pass filters */
	write_checked_reg(ADDR_CTRL_REG3, 0x02);        /* DRDY enable */
	write_checked_reg(ADDR_CTRL_REG4, REG4_BDU);

	set_samplerate(0, _accel_onchip_filter_bandwidth); //1000Hz
	set_range(IIS328DQ_ACCEL_DEFAULT_RANGE_G);
	//设置软件滤波器截止频率
	set_driver_lowpass_filter(IIS328DQ_DEFAULT_RATE, IIS328DQ_DEFAULT_DRIVER_FILTER_FREQ);

	_read = 0;
}

#ifdef USE_HRT
void IIS328DQ::measure_trampoline(void *arg)
#else
void IIS328DQ::measure_trampoline(xTimerHandle xTimer, void *arg)
#endif
{
	IIS328DQ *dev = (IIS328DQ *)arg;

	/* make another measurement */
	dev->measure();
}

void
IIS328DQ::check_registers(void)
{
	uint8_t v;
	if ((v=read_reg(_checked_registers[_checked_next])) != _checked_values[_checked_next]) {

		/*
		  try to fix the bad register value. We only try to
		  fix one per loop to prevent a bad sensor hogging the
		  bus. We skip zero as that is the WHO_AM_I, which
		  is not writeable
		 */
		if (_checked_next != 0) {
			write_reg(_checked_registers[_checked_next], _checked_values[_checked_next]);
		}
		_register_wait = 20;
        }
        _checked_next = (_checked_next+1) % IIS328DQ_NUM_CHECKED_REGISTERS;
}

void
IIS328DQ::measure()
{
	/* status register and data as read back from the device */
	struct {
		uint8_t		cmd;
		uint8_t		status;
		int16_t		x;
		int16_t		y;
		int16_t		z;
	} raw_accel_report;
    uint8_t raw_data[8];

	accel_report accel_report = {0};

	check_registers();

	if (_register_wait != 0) {
		// we are waiting for some good transfers before using
		// the sensor again.
		_register_wait--;
		return;
	}

	/* fetch data from the sensor */
	memset(&raw_data, 0, sizeof(raw_data));
	raw_data[0] = ADDR_STATUS_REG | DIR_READ | ADDR_INCREMENT;
	SpiTransfer(&_devInstance, raw_data, raw_data, sizeof(raw_data));

    raw_accel_report.status = raw_data[1];
    raw_accel_report.x = ((int16_t)raw_data[3] << 8) | raw_data[2];
    raw_accel_report.y = ((int16_t)raw_data[5] << 8) | raw_data[4];
    raw_accel_report.z = ((int16_t)raw_data[7] << 8) | raw_data[6];

        if (!(raw_accel_report.status & STATUS_ZYXDA)) {
		return;
        }

	/*
	 * 1) Scale raw value to SI units using scaling from datasheet.
	 * 2) Subtract static offset (in SI units)
	 * 3) Scale the statically calibrated values with a linear
	 *    dynamically obtained factor
	 *
	 * Note: the static sensor offset is the number the sensor outputs
	 * 	 at a nominally 'zero' input. Therefore the offset has to
	 * 	 be subtracted.
	 *
	 *	 Example: A gyro outputs a value of 74 at zero angular rate
	 *	 	  the offset is 74 from the origin and subtracting
	 *		  74 from all measurements centers them around zero.
	 */


	accel_report.timestamp = hrt_absolute_time();

	accel_report.x_raw = raw_accel_report.x;
	accel_report.y_raw = raw_accel_report.y;
	accel_report.z_raw = raw_accel_report.z;

	float xraw_f = raw_accel_report.x;
	float yraw_f = raw_accel_report.y;
	float zraw_f = raw_accel_report.z;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	float x_in_new = ((xraw_f * _accel_range_scale) - _accel_scale.x_offset) * _accel_scale.x_scale;
	float y_in_new = ((yraw_f * _accel_range_scale) - _accel_scale.y_offset) * _accel_scale.y_scale;
	float z_in_new = ((zraw_f * _accel_range_scale) - _accel_scale.z_offset) * _accel_scale.z_scale;

	/*
	  we have logs where the accelerometers get stuck at a fixed
	  large value. We want to detect this and mark the sensor as
	  being faulty
	 */
	if (fabsf(_last_accel[0] - x_in_new) < 0.001f &&
	    fabsf(_last_accel[1] - y_in_new) < 0.001f &&
	    fabsf(_last_accel[2] - z_in_new) < 0.001f &&
	    fabsf(x_in_new) > 20 &&
	    fabsf(y_in_new) > 20 &&
	    fabsf(z_in_new) > 20) {
		_constant_accel_count += 1;
	} else {
		_constant_accel_count = 0;
	}
	if (_constant_accel_count > 100) {
		// we've had 100 constant accel readings with large
		// values. The sensor is almost certainly dead. We
		// will raise the error_count so that the top level
		// flight code will know to avoid this sensor, but
		// we'll still give the data so that it can be logged
		// and viewed
		_constant_accel_count = 0;
	}

	_last_accel[0] = x_in_new;
	_last_accel[1] = y_in_new;
	_last_accel[2] = z_in_new;

	accel_report.x = _accel_filter_x.apply(x_in_new);
	accel_report.y = _accel_filter_y.apply(y_in_new);
	accel_report.z = _accel_filter_z.apply(z_in_new);

	accel_report.scaling = _accel_range_scale;
	accel_report.range_m_s2 = _accel_range_m_s2;

    //Print_Warn("accel data:rawxyz[%04x,%04x,%04x] size=%d\n", accel_report.x_raw, accel_report.y_raw, accel_report.z_raw, sizeof(raw_accel_report));
	xRingBufferForce(_reports, &accel_report, sizeof(accel_report));

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

//	if (!(_pub_blocked)) 
    {
		/* publish it */
		orb_publish(ORB_ID(sensor_accel), _accel_topic, &accel_report);
	}

	_read++;
}

void
IIS328DQ::print_info()
{
	printf("accel reads:          %u\n", _read);
	vRingBufferPrintInfo(_reports, "report queue");
        ::printf("checked_next: %u\n", _checked_next);
        for (uint8_t i=0; i<IIS328DQ_NUM_CHECKED_REGISTERS; i++) {
            uint8_t v = read_reg(_checked_registers[i]);
            if (v != _checked_values[i]) {
                ::printf("reg %02x:%02x should be %02x\n",
                         (unsigned)_checked_registers[i],
                         (unsigned)v,
                         (unsigned)_checked_values[i]);
            }
        }
}

void
IIS328DQ::print_registers()
{
	printf("iis328dq registers\n");
	for (uint8_t reg=0; reg<=0x40; reg++) {
		uint8_t v = read_reg(reg);
		printf("%02x:%02x ",(unsigned)reg, (unsigned)v);
		if ((reg+1) % 16 == 0) {
			printf("\n");
		}
	}
	printf("\n");
}

void
IIS328DQ::test_error()
{
	// trigger a deliberate error
        write_reg(ADDR_CTRL_REG3, 0);
}

int
IIS328DQ::self_test()
{
	/* inspect accel offsets */
	if (fabsf(_accel_scale.x_offset) < 0.000001f)
		return 1;
	if (fabsf(_accel_scale.x_scale - 1.0f) > 0.4f || fabsf(_accel_scale.x_scale - 1.0f) < 0.000001f)
		return 1;

	if (fabsf(_accel_scale.y_offset) < 0.000001f)
		return 1;
	if (fabsf(_accel_scale.y_scale - 1.0f) > 0.4f || fabsf(_accel_scale.y_scale - 1.0f) < 0.000001f)
		return 1;

	if (fabsf(_accel_scale.z_offset) < 0.000001f)
		return 1;
	if (fabsf(_accel_scale.z_scale - 1.0f) > 0.4f || fabsf(_accel_scale.z_scale - 1.0f) < 0.000001f)
		return 1;


	return 0;
}

/**
 * Local functions in support of the shell command.
 */
namespace iis328dq
{

IIS328DQ	*g_dev[IIS328DQ_MAX_INSTANCE];//用extern_bus作为索引

void	usage();
void	start(bool external_bus, enum Rotation rotation);
void	test();
void	reset();
void	info();
void	regdump();
void	test_error();

/**
 * Start the driver.
 *
 * This function call only returns once the driver
 * started or failed to detect the sensor.
 */
void
start(bool external_bus, enum Rotation rotation)
{
	int fd;

	if (g_dev[external_bus] != NULL) {
		errx(0, "already started");
		return;
	}

	/* create the driver */
    if (external_bus) {
		errx(0, "External SPI not available");
		return;
	} else {
		g_dev[external_bus] = new IIS328DQ(SPI_DEVICE_ID_FOR_SENSOR, IIS328DQ_DEVICE_PATH, (ESpi_device_id)ESPI_DEVICE_TYPE_ACCEL, rotation);
	}

	if (g_dev[external_bus] == NULL)
		goto fail;

	if (OK != g_dev[external_bus]->init())
		goto fail;

	/* set the poll rate to default, starts automatic data collection */
	if(external_bus)
		fd = open(IIS328DQ_EXT_DEVICE_PATH, O_RDONLY);
	else
		fd = open(IIS328DQ_DEVICE_PATH, O_RDONLY);
		

	if (fd == -1)
		goto fail;

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		goto fail;

    close(fd);
	return;
fail:

	if (g_dev[external_bus] != NULL) {
		delete g_dev[external_bus];
		g_dev[external_bus] = NULL;
	}

	errx(1, "driver start failed");
	return;
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test(int external_bus)
{
	int fd_accel;
	struct accel_report accel_report;
	size_t sz;
	int ret;

	if(external_bus)
	{
		/* get the driver */
		fd_accel = open(IIS328DQ_EXT_DEVICE_PATH, O_RDONLY);

		if (fd_accel < 0) {
			err(1, "%s open failed\n", IIS328DQ_EXT_DEVICE_PATH);
			return;
		}
	}
	else
	{
		/* get the driver */
		fd_accel = open(IIS328DQ_DEVICE_PATH, O_RDONLY);

		if (fd_accel < 0) {
			err(1, "%s open failed\n", IIS328DQ_DEVICE_PATH);
			return;
		}
	}

	/* do a simple demand read */
	sz = read(fd_accel, (char*)&accel_report, sizeof(accel_report));

	if (sz != sizeof(accel_report)) {
		err(1, "immediate read failed\n");
		return;
	}

	warnx("accel x: \t% 9.5f\tm/s^2\n", (double)accel_report.x);
	warnx("accel y: \t% 9.5f\tm/s^2\n", (double)accel_report.y);
	warnx("accel z: \t% 9.5f\tm/s^2\n", (double)accel_report.z);
	warnx("accel x: \t%d\traw\n", (int)accel_report.x_raw);
	warnx("accel y: \t%d\traw\n", (int)accel_report.y_raw);
	warnx("accel z: \t%d\traw\n", (int)accel_report.z_raw);

	warnx("accel range: %8.4f m/s^2\n", (double)accel_report.range_m_s2);
	if (ERROR == (ret = ioctl(fd_accel, ACCELIOCGLOWPASS, 0)))
		warnx("accel antialias filter bandwidth: fail\n");
	else
		warnx("accel antialias filter bandwidth: %d Hz\n", ret);

	close(fd_accel);
	errx(0, "PASS\n");
	return;
}

/**
 * Reset the driver.
 */
void
reset(int external_bus)
{
	int fd;
	
	if(external_bus)
		fd = open(IIS328DQ_EXT_DEVICE_PATH, O_RDONLY);
	else
		fd = open(IIS328DQ_DEVICE_PATH, O_RDONLY);
	
	if (fd < 0){
		err(1, "failed ");
		return;
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0){
		err(1, "driver reset failed");
		return;
	}
	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "accel pollrate reset failed");
		return;
	}

    close(fd);

	return;
}

/**
 * Print a little info about the driver.
 */
void
info(int external_bus)
{
	if (g_dev[external_bus] == NULL) {
		errx(1, "driver not running\n");
		return;
	}

	printf("state @ %p\n", g_dev[external_bus]);
	g_dev[external_bus]->print_info();

	return;
}

/**
 * Dump the register information
 */
void
regdump(int external_bus)
{
	if (g_dev[external_bus] == NULL) {
		errx(1, "driver not running");
		return;
	}

	printf("regdump @ %p\n", g_dev[external_bus]);
	g_dev[external_bus]->print_registers();

	return;
}

/**
 * trigger an error
 */
void
test_error(int external_bus)
{
	if (g_dev[external_bus] == NULL) {
		errx(1, "driver not running");
		return;
	}

	printf("regdump @ %p\n", g_dev[external_bus]);
	g_dev[external_bus]->test_error();

	return;
}

void
usage()
{
	warnx("missing command: try 'start', 'info', 'test', 'reset', 'testerror' or 'regdump'");
	warnx("options:");
	warnx("    -X    (external bus)");
	warnx("    -R rotation");
}

} // namespace

int
iis328dq_main(int argc, char *argv[])
{
	bool external_bus = false;
	int ch;
	enum Rotation rotation = ROTATION_NONE;

	/* jump over start/off/etc and look at options first */
	while ((ch = drv_getopt(argc, argv, "XR:")) != EOF) {
		switch (ch) {
		case 'X':
			external_bus = true;
			break;
		case 'R':
			rotation = (enum Rotation)atoi(drv_optarg);
			break;
		default:
			iis328dq::usage();
			return 0;
		}
	}

	const char *verb = argv[drv_optind];

	/*
	 * Start/load the driver.

	 */
	if (!strcmp(verb, "start")) {
		iis328dq::start(external_bus, rotation);
		return 0;
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		iis328dq::test(external_bus);
		return 0;
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		iis328dq::reset(external_bus);
		return 0;
	}	

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		iis328dq::info(external_bus);
		return 0;
	}

	/*
	 * Print register information.
	 */
	if (!strcmp(verb, "regdump")) {
		iis328dq::regdump(external_bus);
		return 0;
	}
		

	/*
	 * trigger an error
	 */
	if (!strcmp(verb, "testerror")) {
		iis328dq::test_error(external_bus);
		return 0;
	}
		

	errx(1, "unrecognized command, try 'start', 'test', 'reset', 'info', 'testerror' or 'regdump'");
	return 0;
}
