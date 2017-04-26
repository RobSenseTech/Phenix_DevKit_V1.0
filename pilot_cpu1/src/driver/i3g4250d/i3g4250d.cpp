/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file i3g4250d.cpp
 * Driver for the ST i3g4250d MEMS and I3G4250DH mems gyros connected via SPI.
 *
 * Note: With the exception of the self-test feature, the ST L3G4200D is
 *       also supported by this driver.
 */

#include "device/cdev.h"
#include "ringbuffer.h"
#include "drv_gyro.h"
#include <uORB/uORB.h>
#include "conversion/rotation.h"
#include "pilot_print.h"
#include "math.h"
#include "driver_define.h"
#include "drv_unistd/drv_unistd.h"
#include "board_config.h"
#include "Filter/LowPassFilter2p.h"
#include "sleep.h"
#include "task.h"
#include "timers.h"
#include "Phx_define.h"
#include "driver.h"

#include <unistd.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>

#define I3G4250D_DEVICE_PATH "/dev/i3g4250d"
#define I3G4250D_EXT_DEVICE_PATH "/dev/i3g4250d_ext"
#define I3G4250D_MAX_INSTANCE 2 //若联飞控，目前最多一款传感器接两个


/* Orientation on board */
#define SENSOR_BOARD_ROTATION_000_DEG	0
#define SENSOR_BOARD_ROTATION_090_DEG	1
#define SENSOR_BOARD_ROTATION_180_DEG	2
#define SENSOR_BOARD_ROTATION_270_DEG	3
#define SENSOR_BOARD_ROTATION_270_DEG_ROLL_180 4

/* SPI protocol address bits */
#define DIR_READ				(1<<7)
#define DIR_WRITE				(0<<7)
#define ADDR_INCREMENT				(1<<6)

/* register addresses */
#define ADDR_WHO_AM_I			0x0F
#define WHO_I_AM				0xD3

#define ADDR_CTRL_REG1			0x20
#define REG1_RATE_LP_MASK			0xF0 /* Mask to guard partial register update */

/* keep lowpass low to avoid noise issues */
#define RATE_100HZ_LP_25HZ		((0<<7) | (0<<6) | (0<<5) | (1<<4))
#define RATE_200HZ_LP_25HZ		((0<<7) | (1<<6) | (0<<5) | (1<<4))
#define RATE_200HZ_LP_50HZ		((0<<7) | (1<<6) | (1<<5) | (0<<4))
#define RATE_200HZ_LP_70HZ		((0<<7) | (1<<6) | (1<<5) | (1<<4))
#define RATE_400HZ_LP_20HZ		((1<<7) | (0<<6) | (1<<5) | (0<<4))
#define RATE_400HZ_LP_25HZ		((1<<7) | (0<<6) | (0<<5) | (1<<4))
#define RATE_400HZ_LP_50HZ		((1<<7) | (0<<6) | (1<<5) | (0<<4))
#define RATE_400HZ_LP_100HZ		((1<<7) | (0<<6) | (1<<5) | (1<<4))
#define RATE_800HZ_LP_30HZ		((1<<7) | (1<<6) | (0<<5) | (0<<4))
#define RATE_800HZ_LP_35HZ		((1<<7) | (1<<6) | (0<<5) | (1<<4))
#define RATE_800HZ_LP_50HZ		((1<<7) | (1<<6) | (1<<5) | (0<<4))
#define RATE_800HZ_LP_100HZ		((1<<7) | (1<<6) | (1<<5) | (1<<4))

#define ADDR_CTRL_REG2			0x21
#define ADDR_CTRL_REG3			0x22
#define ADDR_CTRL_REG4			0x23
#define REG4_RANGE_MASK				0x30 /* Mask to guard partial register update */
#define RANGE_250DPS				(0<<4)
#define RANGE_500DPS				(1<<4)
#define RANGE_2000DPS				(3<<4)

#define ADDR_CTRL_REG5			0x24
#define ADDR_REFERENCE			0x25
#define ADDR_OUT_TEMP			0x26
#define ADDR_STATUS_REG			0x27
#define ADDR_OUT_X_L			0x28
#define ADDR_OUT_X_H			0x29
#define ADDR_OUT_Y_L			0x2A
#define ADDR_OUT_Y_H			0x2B
#define ADDR_OUT_Z_L			0x2C
#define ADDR_OUT_Z_H			0x2D
#define ADDR_FIFO_CTRL_REG		0x2E
#define ADDR_FIFO_SRC_REG		0x2F
#define ADDR_INT1_CFG			0x30
#define ADDR_INT1_SRC			0x31
#define ADDR_INT1_TSH_XH		0x32
#define ADDR_INT1_TSH_XL		0x33
#define ADDR_INT1_TSH_YH		0x34
#define ADDR_INT1_TSH_YL		0x35
#define ADDR_INT1_TSH_ZH		0x36
#define ADDR_INT1_TSH_ZL		0x37
#define ADDR_INT1_DURATION		0x38
#define ADDR_LOW_ODR			0x39


/* Internal configuration values */
#define REG1_POWER_NORMAL			(1<<3)
#define REG1_Z_ENABLE				(1<<2)
#define REG1_Y_ENABLE				(1<<1)
#define REG1_X_ENABLE				(1<<0)

#define REG4_BLE				(1<<6)
//#define REG4_SPI_3WIRE			(1<<0)

#define REG5_FIFO_ENABLE			(1<<6)
#define REG5_REBOOT_MEMORY			(1<<7)

#define STATUS_ZYXOR				(1<<7)
#define STATUS_ZOR				(1<<6)
#define STATUS_YOR				(1<<5)
#define STATUS_XOR				(1<<4)
#define STATUS_ZYXDA				(1<<3)
#define STATUS_ZDA				(1<<2)
#define STATUS_YDA				(1<<1)
#define STATUS_XDA				(1<<0)

#define FIFO_CTRL_BYPASS_MODE			(0<<5)
#define FIFO_CTRL_FIFO_MODE			(1<<5)
#define FIFO_CTRL_STREAM_MODE			(1<<6)
#define FIFO_CTRL_STREAM_TO_FIFO_MODE		(3<<5)
#define FIFO_CTRL_BYPASS_TO_STREAM_MODE		(1<<7)

#define I3G4250D_DEFAULT_RATE			800 //data output frequency
#define I3G4250D_DEFAULT_RANGE_DPS		2000
#define I3G4250D_DEFAULT_FILTER_FREQ		35
#define I3G4250D_TEMP_OFFSET_CELSIUS		40

#ifdef PX4_SPI_BUS_EXT
#define EXTERNAL_BUS PX4_SPI_BUS_EXT
#else
#define EXTERNAL_BUS 0
#endif

#ifndef SENSOR_BOARD_ROTATION_DEFAULT
//#define SENSOR_BOARD_ROTATION_DEFAULT		SENSOR_BOARD_ROTATION_270_DEG
#define SENSOR_BOARD_ROTATION_DEFAULT		SENSOR_BOARD_ROTATION_270_DEG_ROLL_180 
#endif

/*
  we set the timer interrupt to run a bit faster than the desired
  sample rate and then throw away duplicates using the data ready bit.
  This time reduction is enough to cope with worst case timing jitter
  due to other timers
 */
#define I3G4250D_TIMER_REDUCTION				600

extern "C" { __EXPORT int i3g4250d_main(int argc, char *argv[]); }

class I3G4250D : public device::CDev
{
public:
	I3G4250D(int bus, const char* path, ESpi_device_id device, enum Rotation rotation);
	virtual ~I3G4250D();

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

    xTimerHandle    _call;
	unsigned		_call_interval;

	RingBuffer_t	*_reports;

	struct gyro_scale	_gyro_scale;
	float			_gyro_range_scale;
	float			_gyro_range_rad_s;
	orb_advert_t		_gyro_topic;
	int			_orb_class_instance;
	int			_class_instance;

	unsigned		_current_rate;
	unsigned		_current_bandwidth;
	unsigned		_orientation;

	unsigned		_read;

	uint8_t			_register_wait;

	LowPassFilter2p<float>	_gyro_filter_x;
	LowPassFilter2p<float>	_gyro_filter_y;
	LowPassFilter2p<float>	_gyro_filter_z;

	enum Rotation		_rotation;

	// this is used to support runtime checking of key
	// configuration registers to detect SPI bus errors and sensor
	// reset

	struct SDeviceViaSpi _devInstance;

#define I3G4250D_NUM_CHECKED_REGISTERS 8
	static const uint8_t	_checked_registers[I3G4250D_NUM_CHECKED_REGISTERS];
	uint8_t			_checked_values[I3G4250D_NUM_CHECKED_REGISTERS];
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
	bool			is_external() { return 0;/*(_bus == EXTERNAL_BUS); */}

	/**
	 * Static trampoline from the hrt_call context; because we don't have a
	 * generic hrt wrapper yet.
	 *
	 * Called by the HRT in interrupt context at the specified rate if
	 * automatic polling is enabled.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		measure_trampoline(xTimerHandle xTimer);
	/**
	 * check key registers for correct values
	 */
	void			check_registers(void);

	/**
	 * Fetch measurements from the sensor and update the report ring.
	 */
	void			measure();

	/**
	 * Read a register from the I3G4250D
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	uint8_t			read_reg(unsigned reg);

	/**
	 * Write a register in the I3G4250D
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_reg(unsigned reg, uint8_t value);

	/**
	 * Modify a register in the I3G4250D
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg		The register to modify.
	 * @param clearbits	Bits in the register to clear.
	 * @param setbits	Bits in the register to set.
	 */
	void			modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	/**
	 * Write a register in the I3G4250D, updating _checked_values
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_checked_reg(unsigned reg, uint8_t value);

	/**
	 * Set the I3G4250D measurement range.
	 *
	 * @param max_dps	The measurement range is set to permit reading at least
	 *			this rate in degrees per second.
	 *			Zero selects the maximum supported range.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			set_range(unsigned max_dps);

	/**
	 * Set the I3G4250D internal sampling frequency.
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
	I3G4250D(const I3G4250D&);
	I3G4250D operator=(const I3G4250D&);
};

/*
  list of registers that will be checked in check_registers(). Note
  that ADDR_WHO_AM_I must be first in the list.
 */
const uint8_t I3G4250D::_checked_registers[I3G4250D_NUM_CHECKED_REGISTERS] = { ADDR_WHO_AM_I,
                                                                           ADDR_CTRL_REG1,
                                                                           ADDR_CTRL_REG2,
                                                                           ADDR_CTRL_REG3,
                                                                           ADDR_CTRL_REG4,
                                                                           ADDR_CTRL_REG5,
                                                                           ADDR_FIFO_CTRL_REG,
									   ADDR_LOW_ODR };

I3G4250D::I3G4250D(int bus, const char* path, ESpi_device_id device, enum Rotation rotation) :
	CDev("i3g4250d", path),
    _call(NULL),
	_call_interval(0),
	_reports(NULL),
	_gyro_range_scale(0.0f),
	_gyro_range_rad_s(0.0f),
	_gyro_topic(NULL),
	_orb_class_instance(-1),
	_class_instance(-1),
	_current_rate(0),
	_current_bandwidth(50),
	_orientation(SENSOR_BOARD_ROTATION_DEFAULT),
	_read(0),
	_register_wait(0),
	_gyro_filter_x(I3G4250D_DEFAULT_RATE, I3G4250D_DEFAULT_FILTER_FREQ),
	_gyro_filter_y(I3G4250D_DEFAULT_RATE, I3G4250D_DEFAULT_FILTER_FREQ),
	_gyro_filter_z(I3G4250D_DEFAULT_RATE, I3G4250D_DEFAULT_FILTER_FREQ),
	_rotation(rotation),
	_checked_next(0)
{	

    _device_id.devid_s.devtype = DRV_GYR_DEVTYPE_I3G4250D;

	// default scale factors
	_gyro_scale.x_offset = 0;
	_gyro_scale.x_scale  = 1.0f;
	_gyro_scale.y_offset = 0;
	_gyro_scale.y_scale  = 1.0f;
	_gyro_scale.z_offset = 0;
	_gyro_scale.z_scale  = 1.0f;

	_devInstance.spi_id = bus;
	DeviceViaSpiCfgInitialize(&_devInstance,
							device, 
							"i3g4250d",
							ESPI_CLOCK_MODE_2,
							(11*1000*1000));
}

I3G4250D::~I3G4250D()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != NULL) {
		vPortFree(_reports);
	}

	if (_class_instance != -1)
		unregister_class_devname(GYRO_BASE_DEVICE_PATH, _class_instance);
}

int
I3G4250D::init()
{
	int ret = ERROR;

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
	_reports = (RingBuffer_t *) pvPortMalloc (sizeof(RingBuffer_t));
	iRingBufferInit(_reports, 2, sizeof(gyro_report));

	if (_reports == NULL)
		goto out;

	_class_instance = register_class_devname(GYRO_BASE_DEVICE_PATH);

	reset();

	measure();

	/* advertise sensor topic, measure manually to initialize valid report */
	struct gyro_report grp;
	xRingBufferGet(_reports, &grp, sizeof(struct gyro_report));

	_gyro_topic = orb_advertise_multi(ORB_ID(sensor_gyro), &grp,
		&_orb_class_instance, (is_external()) ? ORB_PRIO_VERY_HIGH : ORB_PRIO_DEFAULT);

	if (_gyro_topic == NULL) {
		DEVICE_DEBUG("failed to create sensor_gyro publication");
	}

	ret = OK;
out:
	return ret;
}

int
I3G4250D::probe()
{
	/* read dummy value to void to clear SPI statemachine on sensor */
	(void)read_reg(ADDR_WHO_AM_I);

	bool success = false;
	uint8_t v = 0;

	/* verify that the device is attached and functioning, accept
	 * i3g4250d*/
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
I3G4250D::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct gyro_report);
	struct gyro_report *gbuf = reinterpret_cast<struct gyro_report *>(buffer);
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
			if (0 == xRingBufferGet(_reports, gbuf, sizeof(*gbuf))) {
				ret += sizeof(*gbuf);
				gbuf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement */
	vRingBufferFlush(_reports);
	measure();

	/* measurement will have generated a report, copy it out */
	if (0 == xRingBufferGet(_reports, gbuf, sizeof(*gbuf))) {
		ret = sizeof(*gbuf);
	}

	return ret;
}

int
I3G4250D::ioctl(struct file *filp, int cmd, unsigned long arg)
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
				return ioctl(filp, SENSORIOCSPOLLRATE, I3G4250D_DEFAULT_RATE);

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

					/* adjust filters */
					float cutoff_freq_hz = _gyro_filter_x.get_cutoff_freq();
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

	case GYROIOCSSAMPLERATE:
		return set_samplerate(arg, _current_bandwidth);

	case GYROIOCGSAMPLERATE:
		return _current_rate;

	case GYROIOCSLOWPASS: {
		// set the software lowpass cut-off in Hz
		float cutoff_freq_hz = arg;
		float sample_rate = 1.0e6f / _call_interval;
		set_driver_lowpass_filter(sample_rate, cutoff_freq_hz);

		return OK;
	}

	case GYROIOCGLOWPASS:
		return static_cast<int>(_gyro_filter_x.get_cutoff_freq());

	case GYROIOCSSCALE:
		/* copy scale in */
		memcpy(&_gyro_scale, (struct gyro_scale *) arg, sizeof(_gyro_scale));
		return OK;

	case GYROIOCGSCALE:
		/* copy scale out */
		memcpy((struct gyro_scale *) arg, &_gyro_scale, sizeof(_gyro_scale));
		return OK;

	case GYROIOCSRANGE:
		/* arg should be in dps */
		return set_range(arg);

	case GYROIOCGRANGE:
		/* convert to dps and round */
		return (unsigned long)(_gyro_range_rad_s * 180.0f / M_PI_F + 0.5f);

	case GYROIOCSELFTEST:
		return self_test();

	case GYROIOCSHWLOWPASS:
		return set_samplerate(_current_rate, arg);

	case GYROIOCGHWLOWPASS:
		return _current_bandwidth;

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}

uint8_t
I3G4250D::read_reg(unsigned reg)
{
	uint8_t cmd[2];

	cmd[0] = reg | DIR_READ;
	cmd[1] = 0;

	SpiTransfer(&_devInstance, cmd, cmd, sizeof(cmd));

	return cmd[1];
}

void
I3G4250D::write_reg(unsigned reg, uint8_t value)
{
	uint8_t	cmd[2];

	cmd[0] = reg | DIR_WRITE;
	cmd[1] = value;

	SpiTransfer(&_devInstance, cmd, NULL, sizeof(cmd));
}

void
I3G4250D::write_checked_reg(unsigned reg, uint8_t value)
{
	write_reg(reg, value);
	for (uint8_t i=0; i<I3G4250D_NUM_CHECKED_REGISTERS; i++) {
		if (reg == _checked_registers[i]) {
			_checked_values[i] = value;
		}
	}
}


void
I3G4250D::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val;

	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_checked_reg(reg, val);
}

int
I3G4250D::set_range(unsigned max_dps)
{
	uint8_t bits = 0;
	float new_range_scale_dps_digit;
	float new_range;

	if (max_dps == 0) {
		max_dps = 2000;
	}
	if (max_dps <= 250) {
		new_range = 250;
		bits |= RANGE_250DPS;
		new_range_scale_dps_digit = 8.75e-3f;

	} else if (max_dps <= 500) {
		new_range = 500;
		bits |= RANGE_500DPS;
		new_range_scale_dps_digit = 17.5e-3f;

	} else if (max_dps <= 2000) {
		new_range = 2000;
		bits |= RANGE_2000DPS;
		new_range_scale_dps_digit = 70e-3f;

	} else {
		return -EINVAL;
	}

	_gyro_range_rad_s = new_range / 180.0f * M_PI_F;
	_gyro_range_scale = new_range_scale_dps_digit / 180.0f * M_PI_F;
	write_checked_reg(ADDR_CTRL_REG4, bits);

	return OK;
}

int
I3G4250D::set_samplerate(unsigned frequency, unsigned bandwidth)
{
	uint8_t bits = REG1_POWER_NORMAL | REG1_Z_ENABLE | REG1_Y_ENABLE | REG1_X_ENABLE;

	if (frequency == 0 || frequency == GYRO_SAMPLERATE_DEFAULT) {
		frequency = 800;
	}

	/*
	 * Use limits good for H or non-H models. Rates are slightly different
	 * for L3G4200D part but register settings are the same.
	 */
	if (frequency <= 100) {
		_current_rate =100;
		_current_bandwidth = 25;
		bits |= RATE_100HZ_LP_25HZ;
	} else if (frequency <= 200) {
		_current_rate = 200;

		if (bandwidth <= 25) {
			_current_bandwidth = 25;
			bits |= RATE_200HZ_LP_25HZ;
		} else if (bandwidth <= 50) {
			_current_bandwidth = 50;
			bits |= RATE_200HZ_LP_50HZ;
		} else {
			_current_bandwidth = 70;
			bits |= RATE_200HZ_LP_70HZ;
		}
	} else if (frequency <= 400) {
		_current_rate = 400;

		if (bandwidth <= 25) {
			_current_bandwidth = 25;
			bits |= RATE_400HZ_LP_25HZ;
		} else if (bandwidth <= 50) {
			_current_bandwidth = 50;
			bits |= RATE_400HZ_LP_50HZ;
		} else {
			_current_bandwidth = 110;
			bits |= RATE_400HZ_LP_100HZ;
		}
	} else if (frequency <= 800) {
		_current_rate = 800;
		if (bandwidth <= 30) {
			_current_bandwidth = 30;
			bits |= RATE_800HZ_LP_30HZ;
		} else if (bandwidth <= 50) {
			_current_bandwidth = 50;
			bits |= RATE_800HZ_LP_50HZ;
		} else {
			_current_bandwidth = 110;
			bits |= RATE_800HZ_LP_100HZ;
		}

	} else {
		return -EINVAL;
	}

	write_checked_reg(ADDR_CTRL_REG1, bits);

	return OK;
}

void
I3G4250D::set_driver_lowpass_filter(float samplerate, float bandwidth)
{
	_gyro_filter_x.set_cutoff_frequency(samplerate, bandwidth);
	_gyro_filter_y.set_cutoff_frequency(samplerate, bandwidth);
	_gyro_filter_z.set_cutoff_frequency(samplerate, bandwidth);
}

void
I3G4250D::start()
{
	/* make sure we are stopped first */
//	stop();

	/* reset the report ring */
	vRingBufferFlush(_reports);

	/* start polling at the specified rate */
    int ticks = USEC2TICK(_call_interval);
    if(ticks == 0)
        ticks = 1;//定时器时间间隔不可为0
	/* reset the report ring and state machine */
	_call = xTimerCreate("accel timer", USEC2TICK(_call_interval), pdTRUE, this, &I3G4250D::measure_trampoline);
	xTimerStart(_call, portMAX_DELAY);
	
}

void
I3G4250D::stop()
{
    if(_call != NULL)
        xTimerDelete(_call, portMAX_DELAY);
}

void
I3G4250D::disable_i2c(void)
{
#if 0
	uint8_t retries = 10;
	while (retries--) {
		// add retries
		uint8_t a = read_reg(0x05);
		write_reg(0x05, (0x20 | a));
		if (read_reg(0x05) == (a | 0x20)) {
			// this sets the I2C_DIS bit on the
			// I3G4250D. The l3gd20 datasheet doesn't
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
I3G4250D::reset()
{
	/* set default configuration */
	write_checked_reg(ADDR_CTRL_REG1,
                          REG1_POWER_NORMAL | REG1_Z_ENABLE | REG1_Y_ENABLE | REG1_X_ENABLE);
	write_checked_reg(ADDR_CTRL_REG2, 0);		/* disable high-pass filters */
	write_checked_reg(ADDR_CTRL_REG3, 0x08);        /* DRDY enable */
	write_checked_reg(ADDR_CTRL_REG4, 0);
	write_checked_reg(ADDR_CTRL_REG5, 0);
	write_checked_reg(ADDR_CTRL_REG5, REG5_FIFO_ENABLE);		/* disable wake-on-interrupt */

	/* disable FIFO. This makes things simpler and ensures we
	 * aren't getting stale data. It means we must run the hrt
	 * callback fast enough to not miss data. */
	write_checked_reg(ADDR_FIFO_CTRL_REG, FIFO_CTRL_BYPASS_MODE);

	set_samplerate(0, _current_bandwidth); //800Hz
	set_range(I3G4250D_DEFAULT_RANGE_DPS);
	set_driver_lowpass_filter(I3G4250D_DEFAULT_RATE, I3G4250D_DEFAULT_FILTER_FREQ);

	_read = 0;
}

void I3G4250D::measure_trampoline(xTimerHandle xTimer)
{
    void *timer_id = pvTimerGetTimerID(xTimer);
	I3G4250D *dev = (I3G4250D *)timer_id;

	/* make another measurement */
	dev->measure();
}

void
I3G4250D::check_registers(void)
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
        _checked_next = (_checked_next+1) % I3G4250D_NUM_CHECKED_REGISTERS;
}

void
I3G4250D::measure()
{
	/* status register and data as read back from the device */
	struct {
		uint8_t		cmd;
		int8_t		temp;
		uint8_t		status;
		int16_t		x;
		int16_t		y;
		int16_t		z;
	} raw_report;
    uint8_t raw_data[9];

	gyro_report report = {0};

        check_registers();

	/* fetch data from the sensor */
	memset(raw_data, 0, sizeof(raw_data));
	raw_data[0] = ADDR_OUT_TEMP | DIR_READ | ADDR_INCREMENT;
	SpiTransfer(&_devInstance, raw_data, raw_data, sizeof(raw_data));

    raw_report.temp = raw_data[1];
    raw_report.status = raw_data[2];
    raw_report.x = ((int16_t)raw_data[4] << 8) | raw_data[3];
    raw_report.y = ((int16_t)raw_data[6] << 8) | raw_data[5];
    raw_report.z = ((int16_t)raw_data[8] << 8) | raw_data[7];

    if (!(raw_report.status & STATUS_ZYXDA)) {
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
	report.timestamp = hrt_absolute_time();

	switch (_orientation) {

		case SENSOR_BOARD_ROTATION_000_DEG:
			/* keep axes in place */
			report.x_raw = raw_report.x;
			report.y_raw = raw_report.y;
            report.z_raw = raw_report.z;
			break;

		case SENSOR_BOARD_ROTATION_090_DEG:
			/* swap x and y */
			report.x_raw = raw_report.y;
			report.y_raw = raw_report.x;
            report.z_raw = raw_report.z;
			break;

		case SENSOR_BOARD_ROTATION_180_DEG:
			/* swap x and y and negate both */
			report.x_raw = ((raw_report.x == -32768) ? 32767 : -raw_report.x);
			report.y_raw = ((raw_report.y == -32768) ? 32767 : -raw_report.y);
            report.z_raw = raw_report.z;
			break;

		case SENSOR_BOARD_ROTATION_270_DEG:
			/* swap x and y and negate y */
			report.x_raw = raw_report.y;
			report.y_raw = ((raw_report.x == -32768) ? 32767 : -raw_report.x);
            report.z_raw = raw_report.z;
            break;
		case SENSOR_BOARD_ROTATION_270_DEG_ROLL_180:
			report.x_raw = raw_report.y;
			report.y_raw = raw_report.x;
			report.z_raw = ((raw_report.z == -32768) ? 32767 : -raw_report.z);
			break;
	}

//	report.z_raw = raw_report.z;

	report.temperature_raw = raw_report.temp;

	float xraw_f = report.x_raw;
	float yraw_f = report.y_raw;
	float zraw_f = report.z_raw;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	report.x = ((xraw_f * _gyro_range_scale) - _gyro_scale.x_offset) * _gyro_scale.x_scale;
	report.y = ((yraw_f * _gyro_range_scale) - _gyro_scale.y_offset) * _gyro_scale.y_scale;
	report.z = ((zraw_f * _gyro_range_scale) - _gyro_scale.z_offset) * _gyro_scale.z_scale;

	report.x = _gyro_filter_x.apply(report.x);
	report.y = _gyro_filter_y.apply(report.y);
	report.z = _gyro_filter_z.apply(report.z);

	report.temperature = I3G4250D_TEMP_OFFSET_CELSIUS - raw_report.temp;

	report.scaling = _gyro_range_scale;
	report.range_rad_s = _gyro_range_rad_s;
//	printf("%9.5f, %9.5f, %9.5f\n\r",report.x, report.y, report.z);

	xRingBufferForce(_reports, &report, sizeof(report));

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	/* publish for subscribers */
//	if (!(_pub_blocked)) 
    {
//    	printf("%x, %x, %x\n\r",x, y, z);
		/* publish it */
		orb_publish(ORB_ID(sensor_gyro), _gyro_topic, &report);
	}

	_read++;
}

void
I3G4250D::print_info()
{
	printf("gyro reads:          %u\n", _read);
	vRingBufferPrintInfo(_reports, "report queue");
        ::printf("checked_next: %u\n", _checked_next);
        for (uint8_t i=0; i<I3G4250D_NUM_CHECKED_REGISTERS; i++) {
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
I3G4250D::print_registers()
{
	printf("i3g4250d registers\n");
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
I3G4250D::test_error()
{
	// trigger a deliberate error
        write_reg(ADDR_CTRL_REG3, 0);
}

int
I3G4250D::self_test()
{
	/* evaluate gyro offsets, complain if offset -> zero or larger than 6 dps */
	if (fabsf(_gyro_scale.x_offset) > 0.1f || fabsf(_gyro_scale.x_offset) < 0.000001f)
		return 1;
	if (fabsf(_gyro_scale.x_scale - 1.0f) > 0.3f)
		return 1;

	if (fabsf(_gyro_scale.y_offset) > 0.1f || fabsf(_gyro_scale.y_offset) < 0.000001f)
		return 1;
	if (fabsf(_gyro_scale.y_scale - 1.0f) > 0.3f)
		return 1;

	if (fabsf(_gyro_scale.z_offset) > 0.1f || fabsf(_gyro_scale.z_offset) < 0.000001f)
		return 1;
	if (fabsf(_gyro_scale.z_scale - 1.0f) > 0.3f)
		return 1;

	return 0;
}

/**
 * Local functions in support of the shell command.
 */
namespace i3g4250d
{

I3G4250D	*g_dev[I3G4250D_MAX_INSTANCE];//用extern_bus作为索引

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
		g_dev[external_bus] = new I3G4250D(SPI_DEVICE_ID_FOR_SENSOR, I3G4250D_DEVICE_PATH, (ESpi_device_id)ESPI_DEVICE_TYPE_GYRO, rotation);
	}
	
	if (g_dev[external_bus] == NULL)
		goto fail;

	if (OK != g_dev[external_bus]->init())
		goto fail;

	/* set the poll rate to default, starts automatic data collection */
	if(external_bus)
		fd = open(I3G4250D_EXT_DEVICE_PATH, O_RDONLY);
	else
		fd = open(I3G4250D_DEVICE_PATH, O_RDONLY);
		

	if (fd < 0)
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
	int fd_gyro;
	struct gyro_report g_report;
	size_t sz;
	

	/* get the driver */
	if(external_bus)
	{
		fd_gyro = open(I3G4250D_EXT_DEVICE_PATH, O_RDONLY);
		if(fd_gyro < 0)
        {
			err(1, "%s open failed", I3G4250D_EXT_DEVICE_PATH);
			return;
		}
	}
	else
	{
		fd_gyro = open(I3G4250D_DEVICE_PATH, O_RDONLY);
		if(fd_gyro < 0)
        {
			err(1, "%s open failed", I3G4250D_DEVICE_PATH);
			return;
		}
	}
	
	/* reset to manual polling */
	if (ioctl(fd_gyro, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MANUAL) < 0){
		err(1, "reset to manual polling");
		return;
	}

	/* do a simple demand read */
	sz = read(fd_gyro, (char*)&g_report, sizeof(g_report));

	if (sz != sizeof(g_report)) {
		err(1, "immediate gyro read failed:%d", sz);
		return;
	}

	warnx("gyro x: \t% 9.5f\trad/s", (double)g_report.x);
	warnx("gyro y: \t% 9.5f\trad/s", (double)g_report.y);
	warnx("gyro z: \t% 9.5f\trad/s", (double)g_report.z);
	warnx("temp: \t%d\tC", (int)g_report.temperature);
	warnx("gyro x: \t%d\traw", (int)g_report.x_raw);
	warnx("gyro y: \t%d\traw", (int)g_report.y_raw);
	warnx("gyro z: \t%d\traw", (int)g_report.z_raw);
	warnx("temp: \t%d\traw", (int)g_report.temperature_raw);
	warnx("gyro range: %8.4f rad/s (%d deg/s)", (double)g_report.range_rad_s,
	      (int)((g_report.range_rad_s / M_PI_F) * 180.0f + 0.5f));

	if (ioctl(fd_gyro, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0){
		err(1, "reset to default polling");
		return;
	}

    close(fd_gyro);

	/* XXX add poll-rate tests here too */
	errx(0, "PASS");
	return;
}

/**
 * Reset the driver.
 */
void
reset(int external_bus)
{
	int fd = 0;
	
	if(external_bus)
		fd = open(I3G4250D_EXT_DEVICE_PATH, O_RDONLY);
	else
		fd = open(I3G4250D_DEVICE_PATH, O_RDONLY);
	
	if (fd < 0)
    {
		err(1, "failed ");
		return;
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0)
    {
		err(1, "driver reset failed");
		return;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0){
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

int i3g4250d_main(int argc, char *argv[])
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
			i3g4250d::usage();
			return 0;
		}
	}

	const char *verb = argv[drv_optind];

	/*
	 * Start/load the driver.

	 */
	if (!strcmp(verb, "start")) {
		i3g4250d::start(external_bus, rotation);
		return 0;
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		i3g4250d::test(external_bus);
		return 0;
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		i3g4250d::reset(external_bus);
		return 0;
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		i3g4250d::info(external_bus);
		return 0;
	}

	/*
	 * Print register information.
	 */
	if (!strcmp(verb, "regdump")) {
		i3g4250d::regdump(external_bus);
		return 0;
	}

	/*
	 * trigger an error
	 */
	if (!strcmp(verb, "testerror")) {
		i3g4250d::test_error(external_bus);
		return 0;
	}

	errx(1, "unrecognized command, try 'start', 'test', 'reset', 'info', 'testerror' or 'regdump'");
	return 0;
}
