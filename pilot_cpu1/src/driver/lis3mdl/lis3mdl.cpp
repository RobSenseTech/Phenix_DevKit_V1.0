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
 * @file lis3mdl.cpp
 *
 * Driver for the LIS3MDL magnetometer connected via I2C or SPI.
 */

#include "device/cdev.h"
#include "ringbuffer.h"
#include "drv_mag.h"
#include "pilot_print.h"
#include <uORB/uORB.h>
#include "conversion/rotation.h"
#include "math.h"
#include "driver_define.h"
#include "drv_unistd/drv_unistd.h"
#include "board_config.h"
#include "Filter/LowPassFilter2p.h"
#include "lis3mdl.h"
#include "Phx_define.h"
#include "sleep.h"
#include "timers.h"
#include "irq.h"
#include "driver.h"
#include "perf/perf_counter.h"

#include <unistd.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>


#define LIS3MDL_DEVICE_PATH "/dev/lis3mdl"
#define LIS3MDL_EXT_DEVICE_PATH "/dev/lis3mdl_ext"
#define LIS3MDL_MAX_INSTANCE 2 //若联飞控，目前最多一款传感器接两??

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

/*
 * LIS3MDL internal constants and data structures.
 */

/* Max measurement rate is 160Hz, however with 160 it will be set to 166 Hz, therefore workaround using 150 */
#define LIS3MDL_CONVERSION_INTERVAL	(1000000 / 150)	/* microseconds */

/* SPI protocol address bits */
#define DIR_READ				(1<<7)
#define DIR_WRITE				(0<<7)
#define ADDR_INCREMENT				(1<<6)

/********* CONFIGURATION ADDRESS (read-write)********/
#define ADDR_CONF_CTRL_REG1			0x20
#define ADDR_CONF_CTRL_REG2			0x21
#define ADDR_CONF_CTRL_REG3			0x22
#define ADDR_CONF_CTRL_REG4			0x23
#define ADDR_CONF_CTRL_REG5			0x24


/******** OUTPUT ADDRESS (read-only) *********/
#define ADDR_DATA_OUT_X_LSB		0x28
#define ADDR_DATA_OUT_X_MSB		0x29
#define ADDR_DATA_OUT_Y_LSB		0x2a
#define ADDR_DATA_OUT_Y_MSB		0x2b
#define ADDR_DATA_OUT_Z_LSB		0x2c
#define ADDR_DATA_OUT_Z_MSB		0x2d
#define ADDR_STATUS			    0x27

/* temperature */
#define ADDR_TEMP_OUT_LSB		0x2e
#define ADDR_TEMP_OUT_MSB		0x2f

/******** INTERRUPTION ADDRESS ********/
#define ADDR_INT_CFG           0x30
#define ADDR_INT_SRC           0x31
#define ADDR_INT_THS_L         0x32
#define ADDR_INT_THS_H         0x33

/******** CONFIGURATION MACRO ********/
/* modes not changeable outside of driver */
#define LIS3MDL_REG1_SELF_TEST_ENABLE	(1 << 0)  /* default 0 */
#define LIS3MDL_REG1_FAST_ODR_ENABLE	(1 << 1)  /* default 0, data rates higher than 80HZ*/

#define LIS3MDL_REG1_OUTPUT_DATA_RATE_CLEAN   0xE3
#define LIS3MDL_REG1_OUTPUT_DATA_10_HZ_RATE   (4 << 2)
#define LIS3MDL_REG1_OUTPUT_DATA_20_HZ_RATE   (5 << 2)
#define LIS3MDL_REG1_OUTPUT_DATA_40_HZ_RATE   (6 << 2)
#define LIS3MDL_REG1_OUTPUT_DATA_80_HZ_RATE   (7 << 2)

#define LIS3MDL_REG1_X_Y_AXES_OPERATING_MODE_CLEAN   0x9F
#define LIS3MDL_REG1_X_Y_AXES_MEDIUM_POWER_OPERATING_MODE   (1 << 5)
#define LIS3MDL_REG1_X_Y_AXES_HIGH_PERFORMANCE_OPERATING_MODE   (2 << 5)
#define LIS3MDL_REG1_X_Y_AXES_ULTRA_HIGH_PERFORMANCE_OPERATING_MODE   (3 << 5)
#define LIS3MDL_REG1_TEMP_SENSOR_ENABLE	(1 << 7)  /* default 0 */

#define LIS3MDL_REG2_SOFT_RESET_ENABLE	(1 << 2)  /* default 0 */
#define LIS3MDL_REG2_REBOOT_MEMORY_MODE	(1 << 3)  /* default 0, 0 is normal mode, 1 is reboot memory content */

#define LIS3MDL_REG3_CONTINUOUS_CONVERSION_MODE      (0 << 0)
#define LIS3MDL_REG3_SINGLE_CONVERSION_MODE          (1 << 0)
#define LIS3MDL_REG3_POWER_DOWN_MODE                 (2 << 0)
#define LIS3MDL_REG3_SPI_3_WIRE_MODE	(1 << 2)  /* default 0, 0: 4-wire interface; 1: 3-wire interface*/
#define LIS3MDL_REG3_LOW_POWER_MODE	    (1 << 5)  /* default 0, If this bit is ‘1’, DO[2:0] is set to 0.625 Hz and the system performs; Once the bit is set to ‘0’, the magnetic data rate is configured by the DO bits */



#define LIS3MDL_REG4_BIG_ENDIAN_DATA_MODE	(1 << 1)  /* default 0, 0: data LSb at lower address; 1: data MSb at lower address*/
#define LIS3MDL_REG1_Z_AXES_OPERATING_MODE_CLEAN   0xF3
#define LIS3MDL_REG1_Z_AXES_MEDIUM_POWER_OPERATING_MODE   (1 << 2)
#define LIS3MDL_REG1_Z_AXES_HIGH_PERFORMANCE_OPERATING_MODE   (2 << 2)
#define LIS3MDL_REG1_Z_AXES_ULTRA_HIGH_PERFORMANCE_OPERATING_MODE   (3 << 2)

#define LIS3MDL_REG5_BLOCK_DATA_UPDATE_ENABLE	(1 << 6)  /* default 0, 0: continuous update; 1: output registers not updated until MSb and LSb have been read*/
#define LIS3MDL_REG5_FAST_READ_ENABLE	        (1 << 7)  /* default 0, 0: FAST_READ disabled; 1: FAST_READ enabled (function: reading the high part of DATA OUT only)*/

#define LIS3MDL_STATUS_X_AXIS_DATA_AVAILABLE    (1 << 0)
#define LIS3MDL_STATUS_Y_AXIS_DATA_AVAILABLE    (1 << 1)
#define LIS3MDL_STATUS_Z_AXIS_DATA_AVAILABLE    (1 << 2)
#define LIS3MDL_STATUS_XYZ_AXIS_DATA_AVAILABLE    (1 << 3)
#define LIS3MDL_STATUS_X_AXIS_DATA_OVERWRITTEN    (1 << 4)
#define LIS3MDL_STATUS_Y_AXIS_DATA_OVERWRITTEN    (1 << 5)
#define LIS3MDL_STATUS_Z_AXIS_DATA_OVERWRITTEN    (1 << 6)
#define LIS3MDL_STATUS_XYZ_AXIS_DATA_OVERWRITTEN    (1 << 7)




extern "C" { __EXPORT int lis3mdl_main(int argc, char *argv[]); }

static const uint8_t INVALID_REGISTER = 0;

//#ifndef CONFIG_SCHED_WORKQUEUE
//# error This requires CONFIG_SCHED_WORKQUEUE.
//#endif

class LIS3MDL : public device::CDev
{
public:
	LIS3MDL(int bus, const char* path, enum Rotation rotation);
	virtual ~LIS3MDL();

	virtual int		init();
	virtual int      probe();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

protected:
//	Device			*_interface;

private:
//	work_s			_work;
    xTimerHandle    _work;
	unsigned		_measure_ticks;

	ringbuf_t	*_reports;
	mag_scale		_scale;
	float 			_range_scale;
	float 			_range_ga;
	bool			_collect_phase;
	int			_class_instance;
	int			_orb_class_instance;

	orb_advert_t		_mag_topic;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_buffer_overflows;
	perf_counter_t		_range_errors;
	perf_counter_t		_conf_errors;

	/* status reporting */
	bool			_sensor_ok;		/**< sensor was found and reports ok */
	bool			_calibrated;		/**< the calibration is valid */

	enum Rotation		_rotation;

	struct mag_report	_last_report;           /**< used for info() */

	uint8_t			_range_bits;
	uint8_t			_conf_reg[5];
	uint8_t			_temperature_counter;
	uint8_t			_temperature_error_count;

    struct spi_node     lis3mdl_spi;
	/**
	 * Initialise the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void			start();

	/**
	 * Stop the automatic measurement state machine.
	 */
	void			stop();

	/**
	 * Reset the device
	 */
	int			reset();

	/**
	 * Perform the on-sensor scale calibration routine.
	 *
	 * @note The sensor will continue to provide measurements, these
	 *	 will however reflect the uncalibrated sensor state until
	 *	 the calibration routine has been completed.
	 *
	 * @param enable set to 1 to enable self-test strap, 0 to disable
	 */
	int			calibrate(struct file *filp, unsigned enable);

	/**
	 * Perform the on-sensor scale calibration routine.
	 *
	 * @note The sensor will continue to provide measurements, these
	 *	 will however reflect the uncalibrated sensor state until
	 *	 the calibration routine has been completed.
	 *
	 * @param enable set to 1 to enable self-test positive strap, -1 to enable
	 *        negative strap, 0 to set to normal mode
	 */
	// int			set_excitement(unsigned enable);

	/**
	 * enable lis3mdl temperature compensation
	 */
	int			set_temperature(unsigned enable);

	/**
	 * Set the sensor range.
	 *
	 * Sets the internal range to handle at least the argument in Gauss.
	 */
	int 			set_range(unsigned range);

	/**
	 * check the sensor range.
	 *
	 * checks that the range of the sensor is correctly set, to
	 * cope with communication errors causing the range to change
	 */
	void 			check_range(void);

	/**
	 * check the sensor configuration.
	 *
	 * checks that the config of the sensor is correctly set, to
	 * cope with communication errors causing the configuration to
	 * change
	 */
	void 			check_conf(void);

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 *
	 * This is the heart of the measurement state machine.  This function
	 * alternately starts a measurement, or collects the data from the
	 * previous measurement.
	 *
	 * When the interval between measurements is greater than the minimum
	 * measurement interval, a gap is inserted between collection
	 * and measurement to provide the most recent measurement possible
	 * at the next interval.
	 */
	void			cycle();

	/**
	 * Static trampoline from the workq context; because we don't have a
	 * generic workq wrapper yet.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		cycle_trampoline(xTimerHandle xTimer);

	/**
	 * Write a register.
	 *
	 * @param reg		The register to write.
	 * @param val		The value to write.
	 * @return		OK on write success.
	 */
	int			write_reg(uint8_t reg, uint8_t val);

	/**
	 * Read a register.
	 *
	 * @param reg		The register to read.
	 * @param val		The value read.
	 * @return		OK on read success.
	 */
	int			read_reg(uint8_t reg, uint8_t &val);

	/**
	 * Issue a measurement command.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			measure();

	/**
	 * Collect the result of the most recent measurement.
	 */
	int			collect();

	/**
	 * Convert a big-endian signed 16-bit value to a float.
	 *
	 * @param in		A signed 16-bit big-endian value.
	 * @return		The floating-point representation of the value.
	 */
	float			meas_to_float(uint8_t in[2]);

	/**
	 * Check the current calibration and update device status
	 *
	 * @return 0 if calibration is ok, 1 else
	 */
	int 			check_calibration();

	/**
	* Check the current scale calibration
	*
	* @return 0 if scale calibration is ok, 1 else
	*/
	int 			check_scale();

	/**
	* Check the current offset calibration
	*
	* @return 0 if offset calibration is ok, 1 else
	*/
	int 			check_offset();

	/**
	* Get Local config register value
	*
	* @return config register value. if not exsit, return 0
	*/

	uint8_t         get_config_register(uint8_t config_register);

	int             set_config_to_device(uint8_t config_register, uint8_t val);
	int             get_config_from_device(uint8_t config_register, uint8_t &val);

	int				spi_read(unsigned address, void *data, unsigned count);
	int				spi_write(unsigned address, void *data, unsigned count);

	/* this class has pointer data members, do not allow copying it */
	LIS3MDL(const LIS3MDL &);
	LIS3MDL operator=(const LIS3MDL &);
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int lis3mdl_main(int argc, char *argv[]);


LIS3MDL::LIS3MDL(int bus, const char* path, enum Rotation rotation) :
	CDev("lis3mdl", path),
//	_interface(interface),
	_work(NULL),
	_measure_ticks(0),
	_reports(NULL),
	_scale{},
	_range_scale(0), /* default range scale from counts to gauss */
	_range_ga(1.3f),
	_collect_phase(false),
	_class_instance(-1),
	_orb_class_instance(-1),
	_mag_topic(NULL),
	_sample_perf(perf_alloc(PC_ELAPSED, "lis3mdl_read")),
	_comms_errors(perf_alloc(PC_COUNT, "lis3mdl_comms_errors")),
	_buffer_overflows(perf_alloc(PC_COUNT, "lis3mdl_buffer_overflows")),
	_range_errors(perf_alloc(PC_COUNT, "lis3mdl_range_errors")),
	_conf_errors(perf_alloc(PC_COUNT, "lis3mdl_conf_errors")),
	_sensor_ok(false),
	_calibrated(false),
	_rotation(rotation),
	_last_report{0},
	_range_bits(0),
	_temperature_counter(0),
	_temperature_error_count(0)
{

    lis3mdl_spi.bus_id = bus;
    lis3mdl_spi.cs_pin = GPIO_SPI_CS_MAG;
    lis3mdl_spi.frequency = 11*1000*1000;     //spi frequency

    spi_cs_init(&lis3mdl_spi);
    spi_register_node(&lis3mdl_spi);

    _device_id.devid_s.devtype = DRV_MAG_DEVTYPE_LIS3MDL;

	// enable debug() calls
//	_debug_enabled = false;

	// default scaling
	_scale.x_offset = 0;
	_scale.x_scale = 1.0f;
	_scale.y_offset = 0;
	_scale.y_scale = 1.0f;
	_scale.z_offset = 0;
	_scale.z_scale = 1.0f;

	// work_cancel in the dtor will explode if we don't do this...
//	memset(&_work, 0, sizeof(_work));
	
	int i = 0;
	for (i = 0; i < 5; i++)
	{
		_conf_reg[i] = 0;
	}
}

LIS3MDL::~LIS3MDL()
{
	/* make sure we are truly inactive */
	stop();

	if (_reports != NULL) {
        ringbuf_deinit(_reports);
		vPortFree(_reports);
	}

	if (_class_instance != -1) {
		unregister_class_devname(MAG_BASE_DEVICE_PATH, _class_instance);
	}

    spi_deregister_node(&lis3mdl_spi);
	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_buffer_overflows);
	perf_free(_range_errors);
	perf_free(_conf_errors);
}

int
LIS3MDL::init()
{
	int ret = ERROR;

	if (CDev::init() != OK)
		goto out;
	/* allocate basic report buffers */
//	_reports = new ringbuffer::RingBuffer(2, sizeof(mag_report));
	if (_reports != NULL) {
        ringbuf_deinit(_reports);
		vPortFree(_reports);
		_reports = NULL;
	}
	_reports = (ringbuf_t *) pvPortMalloc (sizeof(ringbuf_t));
	ringbuf_init(_reports, 2, sizeof(mag_report));
	
	if (_reports == NULL) {
		goto out;
	}
	
	/* reset the device configuration */
	reset();

	_class_instance = register_class_devname(MAG_BASE_DEVICE_PATH);
	ret = OK;
	/* sensor is ok, but not calibrated */
	_sensor_ok = true;
out:
	return ret;
}


int
LIS3MDL::probe()
{
	uint8_t v = 0;
	bool success = false;
	int ret;
	/* read dummy value to void to clear SPI statemachine on sensor */
	ret = read_reg(ADDR_ID,v);

	/* verify that the device is attached and functioning, accept
	 * i3g4250d*/
	ret = read_reg(ADDR_ID,v);
	if (v == ID_WHO_AM_I) {
		success = true;
		pilot_info("LIS3MDL_I2C::probe: ID byte match (%02x)\r\n", v);
		return OK;
	}
	pilot_info("LIS3MDL_I2C:: ID byte mismatch (%02x)\r\n", v);
	return -EIO;
}



int LIS3MDL::set_range(unsigned range)
{
	if (range < 4) {
		_range_bits = 0x00;
		_range_scale = 1.0f / 6842.0f;
		_range_ga = 4.0f;
    } else if (range <= 8) {
		_range_bits = 0x01;
		_range_scale = 1.0f / 3421.0f;
		_range_ga = 8.0f;
    } else if (range <= 12) {
		_range_bits = 0x02;
		_range_scale = 1.0f / 2281.0f;
		_range_ga = 12.0f;
    } else {
		_range_bits = 0x03;
		_range_scale = 1.0f / 1711.0f;
		_range_ga = 16.0f;
	}
	/*
	 * Send the command to set the range
	 */
	int ret;

	ret = write_reg(ADDR_CONF_CTRL_REG2, (_range_bits << 5));

	if (OK != ret) {
		perf_count(_comms_errors);
	}
	uint8_t range_bits_in = 0;
	ret = read_reg(ADDR_CONF_CTRL_REG2, range_bits_in);
	if (OK != ret) {
		perf_count(_comms_errors);
	}

	if ((range_bits_in & 0x60) != (_range_bits << 5))
	{
		return ERROR;
	}
	return OK;
}

/**
   check that the range register has the right value. This is done
   periodically to cope with I2C bus noise causing the range of the
   compass changing.
 */
void LIS3MDL::check_range(void)
{
	int ret;

	uint8_t range_bits_in = 0;
	ret = read_reg(ADDR_CONF_CTRL_REG2, range_bits_in);

	if (OK != ret) {
		perf_count(_comms_errors);
		return;
	}

	if ((range_bits_in & 0x60)!= (_range_bits << 5)) {
		perf_count(_range_errors);
		ret = write_reg(ADDR_CONF_CTRL_REG2, (_range_bits << 5));

		if (OK != ret) {
			perf_count(_comms_errors);
		}
	}
}

/**
   check that the configuration register has the right value. This is
   done periodically to cope with I2C bus noise causing the
   configuration of the compass to change.
 */
void LIS3MDL::check_conf(void)
{
	int ret;
	int i;
	uint8_t conf_reg_in = 0;
	uint8_t config_reg[2] = {
			ADDR_CONF_CTRL_REG1,
			ADDR_CONF_CTRL_REG4
	};

	for (i = 0; i < 2; i++)
	{
		ret = read_reg(config_reg[i], conf_reg_in);

	
		if (OK != ret) {
			perf_count(_comms_errors);
			return;
		}

		uint8_t local_config_reg = get_config_register(config_reg[i]);
		if (local_config_reg != INVALID_REGISTER) {
			perf_count(_conf_errors);
			return;
		}

		if (conf_reg_in != local_config_reg) {
			perf_count(_conf_errors);
			ret = write_reg(config_reg[i], local_config_reg);

			if (OK != ret) {
				perf_count(_comms_errors);
			}
		}
	}
}

uint8_t LIS3MDL::get_config_register(uint8_t config_register)
{
	switch (config_register) {
	case ADDR_CONF_CTRL_REG1:
		return _conf_reg[0];
	case ADDR_CONF_CTRL_REG2:
		return _conf_reg[1];
	case ADDR_CONF_CTRL_REG3:
		return _conf_reg[2];
	case ADDR_CONF_CTRL_REG4:
		return _conf_reg[3];
	case ADDR_CONF_CTRL_REG5:
		return _conf_reg[4];
	default:
		err(1, "invalid config register %d\n", config_register);
		return INVALID_REGISTER;
	}
}

int LIS3MDL::set_config_to_device(uint8_t config_register, uint8_t val)
{
	int ret;

	ret = write_reg(config_register, val);

	if (OK != ret) {
		perf_count(_comms_errors);
		return ERROR;
	}
	return OK;
}

int LIS3MDL::get_config_from_device(uint8_t config_register, uint8_t &val)
{
	int ret;

	ret = read_reg(config_register, val);

	if (OK != ret) {
		perf_count(_comms_errors);
		return ERROR;
	}
	return OK;
}

int LIS3MDL::spi_write(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address | DIR_WRITE;
	memcpy(&buf[1], data, count);
	return spi_transfer(&lis3mdl_spi, &buf[0], &buf[0], count + 1);
}

int LIS3MDL::spi_read(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address | DIR_READ | ADDR_INCREMENT;

	int ret = spi_transfer(&lis3mdl_spi, &buf[0], &buf[0], count + 1);
	memcpy(data, &buf[1], count);
	return ret;
}

ssize_t
LIS3MDL::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct mag_report);
	struct mag_report *mag_buf = reinterpret_cast<struct mag_report *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_measure_ticks > 0) {
		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (0 == ringbuf_get(_reports, mag_buf, sizeof(mag_report))) {	
				ret += sizeof(struct mag_report);
				mag_buf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	/* XXX really it'd be nice to lock against other readers here */
	do {
		ringbuf_flush(_reports);
		/* trigger a measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		/* wait for it to complete */
		usleep(LIS3MDL_CONVERSION_INTERVAL);

		/* run the collection phase */
		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		if (0 == ringbuf_get(_reports, mag_buf, sizeof(struct mag_report))) {
			ret = sizeof(struct mag_report);
		}
	} while (0);

	return ret;
}

int
LIS3MDL::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	unsigned dummy = arg;

	switch (cmd) {
	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_measure_ticks = 0;
				return OK;

			/* external signalling (DRDY) not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(LIS3MDL_CONVERSION_INTERVAL);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					unsigned ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(LIS3MDL_CONVERSION_INTERVAL)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}


					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return 1000000 / TICK2USEC(_measure_ticks);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = irqsave();

			if (!ringbuf_resize(_reports, arg)) {
				irqrestore(flags);
				return -ENOMEM;
			}

			irqrestore(flags);

			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:
		return ringbuf_size(_reports);

	case SENSORIOCRESET:
		return reset();

	case MAGIOCSSAMPLERATE:
		/* same as pollrate because device is in single measurement mode*/
		return ioctl(filp, SENSORIOCSPOLLRATE, arg);

	case MAGIOCGSAMPLERATE:
		/* same as pollrate because device is in single measurement mode*/
		return 1000000 / TICK2USEC(_measure_ticks);

	case MAGIOCSRANGE:
		return set_range(arg);

	case MAGIOCGRANGE:
		return _range_ga;

	case MAGIOCSLOWPASS:
	case MAGIOCGLOWPASS:
		/* not supported, no internal filtering */
		return -EINVAL;

	case MAGIOCSSCALE:
		/* set new scale factors */
		memcpy(&_scale, (mag_scale *)arg, sizeof(_scale));
		/* check calibration, but not actually return an error */
		(void)check_calibration();
		return 0;

	case MAGIOCGSCALE:
		/* copy out scale factors */
		memcpy((mag_scale *)arg, &_scale, sizeof(_scale));
		return 0;

	case MAGIOCCALIBRATE:
		return calibrate(filp, arg);

	case MAGIOCEXSTRAP:
		DEVICE_DEBUG("Unsupport - MAGIOCEXSTRAP");
		return 0;

	case MAGIOCSELFTEST:
		return check_calibration();

	case MAGIOCGEXTERNAL:
		DEVICE_DEBUG("MAGIOCGEXTERNAL in main driver");
		return 0;

	case MAGIOCSTEMPCOMP:
		return set_temperature(arg);

	case DEVIOCGDEVICEID:
		return CDev::ioctl(filp, cmd, dummy);

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}

void
LIS3MDL::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	ringbuf_flush(_reports);

	_work = xTimerCreate("poll_lis3mdl", _measure_ticks, pdTRUE, this, &LIS3MDL::cycle_trampoline);	
	xTimerStart(_work, portMAX_DELAY);
	
}

void
LIS3MDL::stop()
{
    if(_work != NULL)
        xTimerDelete(_work, portMAX_DELAY);
}

int
LIS3MDL::reset()
{
	/* set range */
	return set_range(_range_ga);
}

void
LIS3MDL::cycle_trampoline(xTimerHandle xTimer)
{
    void *timer_id = pvTimerGetTimerID(xTimer);
	LIS3MDL *dev = (LIS3MDL *)timer_id;
	dev->cycle();
}

void
LIS3MDL::cycle()
{
	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		if (OK != collect()) {
			DEVICE_DEBUG("collection error");
			/* restart the measurement state machine */
			start();
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 */
		if (_measure_ticks > USEC2TICK(LIS3MDL_CONVERSION_INTERVAL)) {

			/* schedule a fresh cycle call when we are ready to measure again */
			//work_queue(HPWORK,
			//	   &_work,
			//	   (worker_t)&LIS3MDL::cycle_trampoline,
			//	   this,
			//	   _measure_ticks - USEC2TICK(LIS3MDL_CONVERSION_INTERVAL));

			return;
		}
	}

	/* measurement phase */
	if (OK != measure()) {
		DEVICE_DEBUG("measure error");
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	//work_queue(HPWORK,
	//	   &_work,
	//	   (worker_t)&LIS3MDL::cycle_trampoline,
	//	   this,
	//	   USEC2TICK(LIS3MDL_CONVERSION_INTERVAL));
}

int
LIS3MDL::measure()
{
	int ret;

	/*
	 * Send the command to begin a measurement.
	 */
	ret = write_reg(ADDR_CONF_CTRL_REG3, LIS3MDL_REG3_SINGLE_CONVERSION_MODE);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	return ret;
}

int
LIS3MDL::collect()
{
#pragma pack(push, 1)
	struct { /* status register and data as read back from the device */
		uint8_t		x[2];
		uint8_t		y[2];
		uint8_t		z[2];
	}	magnetometer_report;
#pragma pack(pop)
	struct {
		int16_t		x, y, z;
	} report;

	int	ret;
	uint8_t check_counter;

	perf_begin(_sample_perf);
	struct mag_report new_report = {0};
	bool sensor_is_onboard = false;

	float xraw_f;
	float yraw_f;
	float zraw_f;

	/* this should be fairly close to the end of the measurement, so the best approximation of the time */
	new_report.timestamp = hrt_absolute_time();
	new_report.error_count = perf_event_count(_comms_errors);

	/*
	 * @note  We could read the status register here, which could tell us that
	 *        we were too early and that the output registers are still being
	 *        written.  In the common case that would just slow us down, and
	 *        we're better off just never being early.
	 */

	/* get measurements from the device */
	ret = spi_read(ADDR_DATA_OUT_X_LSB, (uint8_t *)&magnetometer_report, sizeof(magnetometer_report));


	if (ret != OK) {
		perf_count(_comms_errors);
		DEVICE_DEBUG("data/status read error");
		goto out;
	}

	/* swap the data we just received */
	report.x = (((int16_t)magnetometer_report.x[1]) << 8) + magnetometer_report.x[0];
	report.y = (((int16_t)magnetometer_report.y[1]) << 8) + magnetometer_report.y[0];
	report.z = (((int16_t)magnetometer_report.z[1]) << 8) + magnetometer_report.z[0];


	
	
	/*
	 * If any of the values are -25600 ~ 25600, there was an internal math error in the sensor.
	 * Generalise this to a simple range check that will also catch some bit errors.
	 */
	if ((abs(report.x) > 25600) ||
	    (abs(report.y) > 25600) ||
	    (abs(report.z) > 25600)) {
		perf_count(_comms_errors);
		goto out;
	}

	/* get measurements from the device */
	new_report.temperature = 0;

	if (_conf_reg[0] & LIS3MDL_REG1_TEMP_SENSOR_ENABLE) {

		uint8_t raw_temperature[2];

		ret = spi_read(ADDR_TEMP_OUT_LSB, raw_temperature, sizeof(raw_temperature));
		
		if (ret == OK) {
			int16_t temp16 = (((int16_t)raw_temperature[1]) << 8) + raw_temperature[0];
			
			new_report.temperature = 25 + (temp16 / (1 * 8.0f));
			_temperature_error_count = 0;
		} else {
			_temperature_error_count++;

			if (_temperature_error_count == 10) {
				/*
				  it probably really is an old LIS3MDL,
				  and can't do temperature. Disable it
				*/
				_temperature_error_count = 0;
				DEVICE_DEBUG("disabling temperature compensation");
				set_temperature(0);
			}
		}
	} else {
		new_report.temperature = _last_report.temperature;
	}

	/* scale values for output */

	/* the standard external mag by 3DR has x pointing to the
	 * right, y pointing backwards, and z down, therefore switch x
	 * and y and invert y */
	xraw_f = report.x;
	yraw_f = report.y;
	zraw_f = report.z;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	/*
	 * RAW outputs
	 *
	 * to align the sensor axes with the board, x and y need to be flipped
	 * and y needs to be negated
	 */
	new_report.x_raw = xraw_f;
	new_report.y_raw = yraw_f;
	/* z remains z */
	new_report.z_raw = zraw_f;


	new_report.x = ((xraw_f * _range_scale) - _scale.x_offset) * _scale.x_scale;
	/* flip axes and negate value for y */
	new_report.y = ((yraw_f * _range_scale) - _scale.y_offset) * _scale.y_scale;
	/* z remains z */
	new_report.z = ((zraw_f * _range_scale) - _scale.z_offset) * _scale.z_scale;

	
	//pilot_info("report.x  = %f, report.y = %f, report.z = %f\r\n",new_report.x, new_report.y, new_report.z) ;
	
	if (!(_pub_blocked)) {

		if (_mag_topic != NULL) {
			/* publish it */
			orb_publish(ORB_ID(sensor_mag), _mag_topic, &new_report);

		} else {
			_mag_topic = orb_advertise_multi(ORB_ID(sensor_mag), &new_report,
							 &_orb_class_instance, (sensor_is_onboard) ? ORB_PRIO_HIGH : ORB_PRIO_MAX);

			if (_mag_topic == NULL) {
				DEVICE_DEBUG("ADVERT FAIL");
			}
		}
	}

	_last_report = new_report;

	/* post a report to the ring */
	if(ringbuf_force(_reports, &new_report, sizeof(new_report)))
    {
		perf_count(_buffer_overflows);
	}

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	/*
	  periodically check the range register and configuration
	  registers. With a bad I2C cable it is possible for the
	  registers to become corrupt, leading to bad readings. It
	  doesn't happen often, but given the poor cables some
	  vehicles have it is worth checking for.
	 */
	check_counter = perf_event_count(_sample_perf) % 256;

	//if (check_counter == 0) {
	//	check_range();
	//}

	//if (check_counter == 128) {
	//	check_conf();
	//}

	ret = OK;

out:
	perf_end(_sample_perf);
	return ret;
}

int LIS3MDL::calibrate(struct file *filp, unsigned enable)
{
    warnx("LIS3MDL Unsupport calibrate");
    return 0;
}

int LIS3MDL::check_scale()
{
	bool scale_valid;

	if ((-FLT_EPSILON + 1.0f < _scale.x_scale && _scale.x_scale < FLT_EPSILON + 1.0f) &&
	    (-FLT_EPSILON + 1.0f < _scale.y_scale && _scale.y_scale < FLT_EPSILON + 1.0f) &&
	    (-FLT_EPSILON + 1.0f < _scale.z_scale && _scale.z_scale < FLT_EPSILON + 1.0f)) {
		/* scale is one */
		scale_valid = false;

	} else {
		scale_valid = true;
	}

	/* return 0 if calibrated, 1 else */
	return !scale_valid;
}

int LIS3MDL::check_offset()
{
	bool offset_valid;

	if ((-2.0f * FLT_EPSILON < _scale.x_offset && _scale.x_offset < 2.0f * FLT_EPSILON) &&
	    (-2.0f * FLT_EPSILON < _scale.y_offset && _scale.y_offset < 2.0f * FLT_EPSILON) &&
	    (-2.0f * FLT_EPSILON < _scale.z_offset && _scale.z_offset < 2.0f * FLT_EPSILON)) {
		/* offset is zero */
		offset_valid = false;

	} else {
		offset_valid = true;
	}

	/* return 0 if calibrated, 1 else */
	return !offset_valid;
}

int LIS3MDL::check_calibration()
{
	bool offset_valid = (check_offset() == OK);
	bool scale_valid  = (check_scale() == OK);

	if (_calibrated != (offset_valid && scale_valid)) {
		warnx("mag cal status changed %s%s", (scale_valid) ? "" : "scale invalid ",
		      (offset_valid) ? "" : "offset invalid");
		_calibrated = (offset_valid && scale_valid);
	}

	/* return 0 if calibrated, 1 else */
	return (!_calibrated);
}

/*
  enable/disable temperature on the LIS3MDL
*/
int LIS3MDL::set_temperature(unsigned enable)
{
	int ret;
	/* get current config */
	ret = read_reg(ADDR_CONF_CTRL_REG1, _conf_reg[0]);

	if (OK != ret) {
		perf_count(_comms_errors);
		return -EIO;
	}

	if (enable != 0) {
		_conf_reg[0] |= LIS3MDL_REG1_TEMP_SENSOR_ENABLE;

	} else {
		_conf_reg[0] &= ~LIS3MDL_REG1_TEMP_SENSOR_ENABLE;
	}

	ret = write_reg(ADDR_CONF_CTRL_REG1, _conf_reg[0]);

	if (OK != ret) {
		perf_count(_comms_errors);
		return -EIO;
	}

	uint8_t conf_reg_ret = 0;

	if (read_reg(ADDR_CONF_CTRL_REG1, conf_reg_ret) != OK) {
		perf_count(_comms_errors);
		return -EIO;
	}

	return conf_reg_ret == _conf_reg[0];
}

int
LIS3MDL::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t buf = val;
	return spi_write(reg, &buf, 1);
}

int
LIS3MDL::read_reg(uint8_t reg, uint8_t &val)
{
	uint8_t buf = val;
	int ret = spi_read(reg, &buf, 1);
	val = buf;
	return ret;
}

float
LIS3MDL::meas_to_float(uint8_t in[2])
{
	union {
		uint8_t	b[2];
		int16_t	w;
	} u;

	u.b[0] = in[1];
	u.b[1] = in[0];

	return (float) u.w;
}

void
LIS3MDL::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_buffer_overflows);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	printf("output  (%.2f %.2f %.2f)\n", (double)_last_report.x, (double)_last_report.y, (double)_last_report.z);
	printf("offsets (%.2f %.2f %.2f)\n", (double)_scale.x_offset, (double)_scale.y_offset, (double)_scale.z_offset);
	printf("scaling (%.2f %.2f %.2f) 1/range_scale %.2f range_ga %.2f\n",
	       (double)_scale.x_scale, (double)_scale.y_scale, (double)_scale.z_scale,
	       (double)(1.0f / _range_scale), (double)_range_ga);
	printf("temperature %.2f\n", (double)_last_report.temperature);
	ringbuf_printinfo(_reports, "report queue");
}

/**
 * Local functions in support of the shell command.
 */
namespace lis3mdl
{

LIS3MDL	*g_dev[LIS3MDL_MAX_INSTANCE];//?????extern_bus?????o????????


void	start(bool external_bus, enum Rotation rotation);
void	test(bool external_bus);
void	reset(bool external_bus);
int	info(bool external_bus);
int	calibrate(bool external_bus);
int	temp_enable(bool external_bus, bool enable);
void	usage();
void    self_test(bool external_bus);


/**
 * Start the driver.
 *
 * This function call only returns once the driver
 * is either successfully up and running or failed to start.
 */
void
start(bool external_bus, enum Rotation rotation)
{
	int fd;

	if (g_dev[external_bus] != NULL)
	{
		errx(0, "already started");
			return ;
	}
	/* create the driver */
	
	if (external_bus) {
#ifdef PX4_SPI_BUS_EXT
		g_dev[external_bus] = new LIS3MDL(PX4_SPI_BUS_EXT, LIS3MDL_EXT_DEVICE_PATH, rotation);
#else
		errx(0, "External SPI not available");
		return ;
#endif
	} else {
		g_dev[external_bus] = new LIS3MDL(SPI_DEVICE_ID_FOR_SENSOR, LIS3MDL_DEVICE_PATH, rotation);
	}
	
	if (g_dev[external_bus] == NULL)
		goto fail;

	if (OK != g_dev[external_bus]->init())
		goto fail;
	
	if (OK != g_dev[external_bus]->probe())
		goto fail;
	
	/* set the poll rate to default, starts automatic data collection */
	if(external_bus)
		fd = open(LIS3MDL_EXT_DEVICE_PATH, O_RDONLY);
	else
		fd = open(LIS3MDL_DEVICE_PATH, O_RDONLY);
	
	if (fd < 0)
		goto fail;
	
	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		goto fail;
	
	close(fd);
	return ;
	
fail:
	pilot_info("start :go fail \r\n");
	if (g_dev[external_bus] != NULL) {
		delete g_dev[external_bus];
		g_dev[external_bus] = NULL;
	}

	errx(1, "driver start failed");
		return ;

}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test(bool external_bus)
{
	struct mag_report report;
	struct pollfd fds;
	size_t sz;
	int ret;
	int fd;
	unsigned i = 0;	
		pilot_info("LIS3MDL test ------------1\r\n");	
//		vTaskStartScheduler();
	/* get the driver */
	if(external_bus)
	{
		fd = open(LIS3MDL_EXT_DEVICE_PATH, O_RDONLY);
		if (fd < 0)
		{
			err(1, "%s open failed", LIS3MDL_EXT_DEVICE_PATH);
			return ;
		}
	}
	else
	{
		fd = open(LIS3MDL_DEVICE_PATH, O_RDONLY);
		if (fd < 0)
		{
			err(1, "%s open failed", LIS3MDL_DEVICE_PATH);
			return ;
		}
	}	
		pilot_info("LIS3MDL test ------------read\r\n");	
	/* do a simple demand read */
	sz = read(fd, (char*)&report, sizeof(report));
		pilot_info("LIS3MDL test ------------2\r\n");	
	if (sz != sizeof(report)) {
		err(1, "immediate read failed");
			return ;
	}

	pilot_info("LIS3MDL test ------------3\r\n");	
	
	warnx("single read\r\n");
	warnx("measurement: %.6f  %.6f  %.6f\r\n", (double)report.x, (double)report.y, (double)report.z);
	warnx("time:        %lld\r\n", report.timestamp);

	/* check if mag is onboard or external */
	if ((ret = ioctl(fd, MAGIOCGEXTERNAL, 0)) < 0) {
		errx(1, "failed to get if mag is onboard or external\r\n");
			return ;
	}

	pilot_info("ret = %x/r/n",ret);	
	
	warnx("device active: %s", ret ? "external" : "onboard\r\n");

	/* set the queue depth to 5 */
	if (OK != ioctl(fd, SENSORIOCSQUEUEDEPTH, 10)) {
		errx(1, "failed to set queue depth");
			return ;
	}
	pilot_info("LIS3MDL test ------------6\r\n");	
	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		errx(1, "failed to set 2Hz poll rate");
		return ;
	}

	
	pilot_info("LIS3MDL test ------------7\r\n");	

	/* read the sensor 5x and report each value */
	for(i = 0; i < 5; i++) {
		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		pilot_info("LIS3MDL test ------------9\r\n");	
		ret = poll(&fds, 1, 2000);
		pilot_info("LIS3MDL test ------------10\r\n");	
		if (ret != 1) {
			errx(1, "timed out waiting for sensor data");
			return ;
		}
	pilot_info("LIS3MDL test ------------11\r\n");	
		/* now go get it */
		sz = read(fd, (char*)&report, sizeof(report));
	pilot_info("LIS3MDL test ------------12\r\n");	
		if (sz != sizeof(report)) {
			err(1, "periodic read failed");
			return ;
		}

		warnx("periodic read %u", i);
		warnx("measurement: %.6f  %.6f  %.6f", (double)report.x, (double)report.y, (double)report.z);
		warnx("time:        %lld", report.timestamp);
	}

	errx(0, "PASS");
	return ;
}

void
self_test(bool external_bus)
{
    err(1, "funciton is coding");
}

int calibrate(bool external_bus)
{
	int ret;
	int fd;
	if(external_bus)
		fd = open(LIS3MDL_EXT_DEVICE_PATH, O_RDONLY);
	else
		fd = open(LIS3MDL_DEVICE_PATH, O_RDONLY);
	
	if (fd < 0) {
		err(1, "open failed (try 'lis3mdl start' if the driver is not running)\r\n");
		return 0;
	}
	
	if (OK != (ret = ioctl(fd, MAGIOCCALIBRATE, fd))) {
		warnx("failed to enable sensor calibration mode\r\n");
	}
	close(fd);

	return ret;
}

/**
 * Reset the driver.
 */
void
reset(bool external_bus)
{
	int fd;
	
	if(external_bus)
		fd = open(LIS3MDL_EXT_DEVICE_PATH, O_RDONLY);
	else
		fd = open(LIS3MDL_DEVICE_PATH, O_RDONLY);
	
	if (fd < 0) {
		err(1, "failed ");
		return ;
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
		return ;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
		return ;
	}

	return ;
}


/**
 * enable/disable temperature compensation
 */
int
temp_enable(bool external_bus, bool enable)
{
	int fd;
	
	if(external_bus)
		fd = open(LIS3MDL_EXT_DEVICE_PATH, O_RDONLY);
	else
		fd = open(LIS3MDL_DEVICE_PATH, O_RDONLY);
	
	if (fd < 0) {
		err(1, "failed ");
		return 0;
	}
	if (ioctl(fd, MAGIOCSTEMPCOMP, (unsigned)enable) < 0) {
		err(1, "set temperature compensation failed");
		return 0;
	}
	close(fd);
	return 0;
}

/**
 * Print a little info about the driver.
 */
int
info(bool external_bus)
{
	if (g_dev[external_bus] == NULL)
		errx(1, "driver not running\n");

	printf("state @ %p\n", g_dev[external_bus]);
	g_dev[external_bus]->print_info();
	return 0;
	
}

void
usage()
{
	warnx("missing command: try 'start', 'info', 'test', 'reset', 'info', 'calibrate'\r\n");
	warnx("options:\r\n");
	warnx("    -R rotation\r\n");
	warnx("    -C calibrate on start\r\n");
	warnx("    -S only SPI bus\r\n");
	warnx("    -T Temperature on start\r\n");
}

} // namespace

int
lis3mdl_main(int argc, char *argv[])
{
	int ch;
	bool external_bus = false;
	enum Rotation rotation = ROTATION_NONE;
	bool calibrate = false;
	bool temperature_enable = false;

	while ((ch = drv_getopt(argc, argv, "XR:CT")) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (enum Rotation)atoi(drv_optarg);
			break;
			
		case 'X':
			external_bus = true;
			break;

		case 'C':
			calibrate = true;
			break;

		case 'T':
			temperature_enable = true;
			break;

		default:
			lis3mdl::usage();
			return 0;
		}
	}

	const char *verb = argv[drv_optind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		lis3mdl::start(external_bus, rotation);
		if (calibrate && lis3mdl::calibrate(external_bus) != 0) {
			errx(1, "calibration failed\r\n");
			return 1;
		}
		if (temperature_enable) {
			// we consider failing to setup temperature
			// compensation as non-fatal
			lis3mdl::temp_enable(external_bus, true);
		}
		return 0;
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		lis3mdl::test(external_bus);
		return 0;
	}

	if (!strcmp(verb, "selftest")) {
		lis3mdl::self_test(external_bus);
		return 0;
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		lis3mdl::reset(external_bus);
		return 0;
	}

	/*
	 * enable/disable temperature compensation
	 */
	if (!strcmp(verb, "tempoff")) {
		lis3mdl::temp_enable(external_bus, false);
		return 0;
	}

	if (!strcmp(verb, "tempon")) {
		lis3mdl::temp_enable(external_bus, true);
		return 0;
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info") || !strcmp(verb, "status")) {
		lis3mdl::info(external_bus);
		return 0;
	}

	/*
	 * Autocalibrate the scaling
	 */
	if (!strcmp(verb, "calibrate")) {
		if (lis3mdl::calibrate(external_bus) == 0) {
			errx(0, "calibration successful");
			return 0;
		} else {
			errx(1, "calibration failed");
			return 0;
		}
	}

	errx(1, "unrecognized command, try 'start', 'test', 'reset' 'calibrate', 'tempoff', 'tempon' or 'info'");
	return 0;
}
