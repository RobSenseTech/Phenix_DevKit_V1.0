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
 * @file ms5611.cpp
 * Driver for the MS5611 barometric pressure sensor connected via I2C or SPI.
 */

#include "device/cdev.h"
#include "ringbuffer.h"
#include "drv_baro.h"
#include "FreeRTOS_Print.h"
#include <uORB/uORB.h>
#include "conversion/rotation.h"
#include "math.h"
#include "driver_define.h"
#include "drv_unistd/drv_unistd.h"
#include "board_config.h"
#include "Filter/LowPassFilter2p.h"
#include "ms5611.h"
#include "Phx_define.h"
#include "sleep.h"
#include "timers.h"
#include "driver.h"

#include <unistd.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>

#define MS5611_MAX_INSTANCE 2 //若联飞控，目前最多一款传感器接两??
#define MS5611_BARO_DEVICE_PATH_EXT	"/dev/ms5611_ext"
#define MS5611_BARO_DEVICE_PATH_INT	"/dev/ms5611_int"

//using namespace pilot::driver;

#define OK						0
#define DEV_FAILURE				0
#define DEV_SUCCESS				1

/* SPI protocol address bits */
#define DIR_READ				(1<<7)
#define DIR_WRITE				(0<<7)
#define ADDR_INCREMENT				(1<<6)


//#ifndef CONFIG_SCHED_WORKQUEUE
//# error This requires CONFIG_SCHED_WORKQUEUE.
//#endif

/* helper macro for handling report buffer indices */
#define INCREMENT(_x, _lim)	do { __typeof__(_x) _tmp = _x+1; if (_tmp >= _lim) _tmp = 0; _x = _tmp; } while(0)

/* helper macro for arithmetic - returns the square of the argument */
#define POW2(_x)		((_x) * (_x))

/*
 * MS5611 internal constants and data structures.
 */

/* internal conversion time: 9.17 ms, so should not be read at rates higher than 100 Hz */
#define MS5611_CONVERSION_INTERVAL	10000	/* microseconds */
#define MS5611_MEASUREMENT_RATIO	3	/* pressure measurements per temperature measurement */


class MS5611 : public device::CDev
{
public:
	MS5611(int bus, const char* path, ESpi_device_id device);
	virtual ~MS5611();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

protected:


	ms5611::prom_u		_prom;
//	ms5611::prom_u     _prom_u;
	
    xTimerHandle    _work;
	unsigned		_measure_ticks;

	RingBuffer_t	*_reports;

	bool			_collect_phase;
	unsigned		_measure_phase;

	/* intermediate temperature values per MS5611 datasheet */
	int32_t			_TEMP;
	int64_t			_OFF;
	int64_t			_SENS;
	float			_P1;
	float			_T;

	/* altitude conversion calibration */
	unsigned		_msl_pressure;	/* in Pa */

	orb_advert_t		_baro_topic;
	int			_orb_class_instance;
	int			_class_instance;

//	perf_counter_t		_sample_perf;
//	perf_counter_t		_measure_perf;
//	perf_counter_t		_comms_errors;
//	perf_counter_t		_buffer_overflows;

	struct SDeviceViaSpi _devInstance;
	/**
	 * Initialize the automatic measurement state machine and start it.
	 *
	 * @param delay_ticks the number of queue ticks before executing the next cycle
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void			start_cycle(unsigned delay_ticks = 1);

	/**
	 * Stop the automatic measurement state machine.
	 */
	void			stop_cycle();
	int				_read_prom();
	int 			reset_sensor();
	int 			spi_read(unsigned address, void *data, unsigned count);
    uint16_t        _reg16(unsigned reg);
	int				ms5611_spi_read(unsigned offset, void *data, unsigned count);
	int				write_reg(uint8_t reg, uint8_t val)	;
	int 			spi_write(unsigned address, void *data, unsigned count);
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
	 * Get the internal / external state
	 *
	 * @return true if the sensor is not on the main MCU board
	 */
	bool			is_external() { return (_orb_class_instance == 0); /* XXX put this into the interface class */ }

	/**
	 * Static trampoline from the workq context; because we don't have a
	 * generic workq wrapper yet.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		cycle_trampoline(xTimerHandle xTimer);

	/**
	 * Issue a measurement command for the current state.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	virtual int		measure();

	/**
	 * Collect the result of the most recent measurement.
	 */
	virtual int		collect();
};

/*
 * Driver 'main' command.
 */
extern "C"  __EXPORT int ms5611_main(int argc, char *argv[]);

MS5611::MS5611(int bus, const char* path, ESpi_device_id device) :
	CDev("ms5611", path),
	_measure_ticks(0),
	_reports(NULL),
	_collect_phase(false),
	_measure_phase(0),
	_TEMP(0),
	_OFF(0),
	_SENS(0),
	_msl_pressure(101325),
	_baro_topic(NULL),
	_orb_class_instance(-1),
	_class_instance(-1)
//	_sample_perf(perf_alloc(PC_ELAPSED, "ms5611_read")),
//	_measure_perf(perf_alloc(PC_ELAPSED, "ms5611_measure")),
//	_comms_errors(perf_alloc(PC_COUNT, "ms5611_comms_errors")),
//	_buffer_overflows(perf_alloc(PC_COUNT, "ms5611_buffer_overflows"))
//added by prj
{
	_devInstance.spi_id = bus;
	DeviceViaSpiCfgInitialize(&_devInstance,
							  device, 
							  "ms5611",
							  ESPI_CLOCK_MODE_2,
							  (11*1000*1000));
	_class_instance = 0;
}

MS5611::~MS5611()
{
	/* make sure we are truly inactive */
	stop_cycle();

	if (_class_instance != -1) {
		unregister_class_devname(get_devname(), _class_instance);
	}

	/* free any existing reports */
	if (_reports != NULL) {
		vPortFree(_reports);
	}

	// free perf counters
//	perf_free(_sample_perf);
//	perf_free(_measure_perf);
//	perf_free(_comms_errors);
//	perf_free(_buffer_overflows);

}


uint16_t
MS5611::_reg16(unsigned reg)
{
	uint8_t cmd[3] = { (uint8_t)(reg | DIR_READ), 0, 0 };

	SpiTransfer(&_devInstance, cmd, cmd, sizeof(cmd));

	return (uint16_t)(((uint16_t)cmd[1] << 8) | cmd[2]);
}

int MS5611::spi_read(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address | DIR_READ | ADDR_INCREMENT;

	int ret = SpiTransfer(&_devInstance, &buf[0], &buf[0], count + 1);
	memcpy(data, &buf[1], count);
	return ret;
}

int MS5611::spi_write(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address | DIR_WRITE;
	memcpy(&buf[1], data, count);
	return SpiTransfer(&_devInstance, &buf[0], &buf[0], count + 1);
}

int
MS5611::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t buf = val;
	return spi_write(reg, &buf, 1);
}

int
MS5611::_read_prom()
{
	int i;
	/*
	 * Wait for PROM contents to be in the device (2.8 ms) in the case we are
	 * called immediately after reset.
	 */
	usleep(3000);

	/* read and convert PROM words */
	bool all_zero = true;

	uint16_t temp[8];
	
	for(i=0; i<8; i++)
    {
        uint8_t cmd = (ADDR_PROM_SETUP + (i * 2));
        temp[i] = _reg16(cmd);
        if(temp[i] != 0)
        {
            all_zero = false;
        }
        memcpy(&_prom.s, temp, sizeof(temp));
	    Print_Info("temp[%d] = %u\n", i, temp[i]);
    }
	//for (int i = 0; i < 8; i++) {
	//	uint8_t cmd = (ADDR_PROM_SETUP + (i * 2));
	//	_prom.c[i] = _reg16(cmd);
	//	Print_Info("_prom.c[%d] = %d\r\n",i,_prom.c[i]);
	//	if (_prom.c[i] != 0) {
	//		all_zero = false;
	//	}

		//DEVICE_DEBUG("prom[%u]=0x%x", (unsigned)i, (unsigned)_prom.c[i]);
	//}

	/* calculate CRC and return success/failure accordingly */
	int ret = ms5611::crc4(&_prom.c[0]) ? OK : -EIO;

	if (ret != OK) {
		Print_Err("crc failed");
	}

	if (all_zero) {
		Print_Err("prom all zero");
		ret = -EIO;
	}

	return ret;
}


int 
MS5611::reset_sensor()
{
	u8 cmd = ADDR_RESET_CMD | DIR_WRITE;

	  SpiTransfer(&_devInstance, &cmd, &cmd, 1);
	  Print_Info("cmd = %d\r\n",cmd);
	return OK;
//	return	write_reg(cmd,0);
	
}

int
MS5611::init()
{
	int ret = DEV_FAILURE;

	ret = reset_sensor();
	if (ret != OK) {
		Print_Err("reset sensor failed\n");
		goto out;
	}
	
	ret = _read_prom();
	if (ret != OK) {
		Print_Err("read prom failed\n");
		goto out;
	}
	
	ret = CDev::init();

	if (ret != OK) {
		Print_Err("CDev init failed\n");
		goto out;
	}

	/* allocate basic report buffers */
	if (_reports != NULL) {
		vPortFree(_reports);
		_reports = NULL;
	}
	_reports = (RingBuffer_t *) pvPortMalloc (sizeof(RingBuffer_t));
	iRingBufferInit(_reports, 2, sizeof(sensor_baro_s));

//	_reports = new ringbuffer::RingBuffer(2, sizeof(sensor_baro_s));


	if (_reports == NULL) {
		Print_Err("can't get memory for reports\n");
		ret = -ENOMEM;
		goto out;
	}

	/* register alternate interfaces if we have to */
	_class_instance = register_class_devname(BARO_BASE_DEVICE_PATH);

	struct baro_report brp;
	/* do a first measurement cycle to populate reports with valid data */
	_measure_phase = 0;
	vRingBufferFlush(_reports);
//	_reports->flush();

	/* this do..while is goto without goto */
	do {
		/* do temperature first */
		if (OK != measure()) {
            Print_Err("measure temperature failed\n");
			ret = -EIO;
			break;
		}

		usleep(MS5611_CONVERSION_INTERVAL);

		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* now do a pressure measurement */
		if (OK != measure()) {
            Print_Err("measure pressure failed\n");
			ret = -EIO;
			break;
		}

		usleep(MS5611_CONVERSION_INTERVAL);

		if (OK != collect()) {
            Print_Err("collect failed\n");
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		xRingBufferGet(_reports, &brp, sizeof(brp));

		ret = OK;

		_baro_topic = orb_advertise_multi(ORB_ID(sensor_baro), &brp,
						  &_orb_class_instance, (is_external()) ? ORB_PRIO_HIGH : ORB_PRIO_DEFAULT);


		if (_baro_topic == NULL) {
			warnx("failed to create sensor_baro publication");
		}

	} while (0);

out:
	return ret;
}

ssize_t
MS5611::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct baro_report);
	struct baro_report *brp = reinterpret_cast<struct baro_report *>(buffer);
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
			if (0 == xRingBufferGet(_reports, brp, sizeof(struct baro_report))) {
				ret += sizeof(struct baro_report);
				brp++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	do {
		_measure_phase = 0;
		vRingBufferFlush(_reports);	

		/* do temperature first */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		usleep(MS5611_CONVERSION_INTERVAL);

		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* now do a pressure measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		usleep(MS5611_CONVERSION_INTERVAL);

		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
//		if (_reports->get(brp)) {		
		if (0 == xRingBufferGet(_reports, brp, sizeof(struct baro_report))) {
			ret = sizeof(*brp);
		}

	} while (0);

	return ret;
}

int
MS5611::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop_cycle();
				_measure_ticks = 0;
				return OK;

			/* external signalling not supported */
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
					_measure_ticks = USEC2TICK(MS5611_CONVERSION_INTERVAL);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start_cycle();
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
					if (ticks < USEC2TICK(MS5611_CONVERSION_INTERVAL)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start_cycle();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return (1000 / _measure_ticks);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = irqsave();

//			if (!_reports->resize(arg)) {
			if (!xRingBufferResize(_reports, arg)) {
				irqrestore(flags);
				return -ENOMEM;
			}

			irqrestore(flags);
			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:
//		return _reports->size();
		return iRingBufferSize(_reports);
		
	case SENSORIOCRESET:
		/*
		 * Since we are initialized, we do not need to do anything, since the
		 * PROM is correctly read and the part does not need to be configured.
		 */
		return OK;

	case BAROIOCSMSLPRESSURE:

		/* range-check for sanity */
		if ((arg < 80000) || (arg > 120000)) {
			return -EINVAL;
		}

		_msl_pressure = arg;
		return OK;

	case BAROIOCGMSLPRESSURE:
		return _msl_pressure;

	default:
		break;
	}

	/* give it to the bus-specific superclass */
	// return bus_ioctl(filp, cmd, arg);
	return CDev::ioctl(filp, cmd, arg);
}


int
MS5611::ms5611_spi_read(unsigned offset, void *data, unsigned count)
{
	union _cvt {
		uint8_t	b[4];
		uint32_t w;
	} *cvt = (_cvt *)data;
	uint8_t buf[4] = { 0 | DIR_WRITE, 0, 0, 0 };

	/* read the most recent measurement */
	int ret = SpiTransfer(&_devInstance, &buf[0], &buf[0], sizeof(buf));

	if (ret == OK) {
		/* fetch the raw value */
		cvt->b[0] = buf[3];
		cvt->b[1] = buf[2];
		cvt->b[2] = buf[1];
		cvt->b[3] = 0;

		ret = count;
	}

	return ret;
}



void
MS5611::start_cycle(unsigned delay_ticks)
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_measure_phase = 0;
	vRingBufferFlush(_reports);	
	_work = xTimerCreate("poll_ms5611", USEC2TICK(_measure_ticks * 1000), pdTRUE, this, &MS5611::cycle_trampoline);
	xTimerStart(_work, portMAX_DELAY);
	
	/* schedule a cycle to start things */
}

void
MS5611::stop_cycle()
{
	xTimerStop(_work, portMAX_DELAY);
}

void
MS5611::cycle_trampoline(xTimerHandle xTimer)
{
    void *timer_id = pvTimerGetTimerID(xTimer);
	MS5611 *dev = reinterpret_cast<MS5611 *>(timer_id);
	dev->cycle();
}

void
MS5611::cycle()
{
	int ret;
	unsigned dummy;

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		ret = collect();

		if (ret != OK) {
			if (ret == -6) {
				/*
				 * The ms5611 seems to regularly fail to respond to
				 * its address; this happens often enough that we'd rather not
				 * spam the console with a message for this.
				 */
			} else {
				//DEVICE_LOG("collection error %d", ret);
			}

			/* issue a reset command to the sensor */
//added by prj
			uint8_t cmd = ADDR_RESET_CMD | DIR_WRITE;
			SpiTransfer(&_devInstance, &cmd, NULL, 1);
			/* reset the collection state machine and try again - we need
			 * to wait 2.8 ms after issuing the sensor reset command
			 * according to the MS5611 datasheet
			start_cycle(USEC2TICK(2800));
			 */
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 * Don't inject one after temperature measurements, so we can keep
		 * doing pressure measurements at something close to the desired rate.
		 */
		if ((_measure_phase != 0) &&
		    (_measure_ticks > USEC2TICK(MS5611_CONVERSION_INTERVAL))) {

			return;
		}
	}

	/* measurement phase */
	ret = measure();

	if (ret != OK) {
		/* issue a reset command to the sensor */
		//added by  prj
		uint8_t cmd = ADDR_RESET_CMD | DIR_WRITE;
		SpiTransfer(&_devInstance, &cmd, NULL, 1);
		/* reset the collection state machine and try again 
		start_cycle();
        */
		return;
	}

	/* next phase is collection */
	_collect_phase = true;
}

int
MS5611::measure()
{
	int ret;

	/*
	 * In phase zero, request temperature; in other phases, request pressure.
	 */
	uint8_t addr = (_measure_phase == 0) ? ADDR_CMD_CONVERT_D2 : ADDR_CMD_CONVERT_D1;

	/*
	 * Send the command to begin measuring.
	 */
	//added by prj
	addr |= DIR_WRITE;
	ret = SpiTransfer(&_devInstance, &addr, NULL, 1);

	return ret;
}

int
MS5611::collect()
{
	int ret;
	uint32_t raw;

//	perf_begin(_sample_perf);

	struct baro_report report = {0};
	/* this should be fairly close to the end of the conversion, so the best approximation of the time */
	report.timestamp = hrt_absolute_time();
//	report.error_count = perf_event_count(_comms_errors);

	/* read the most recent measurement - read offset/size are hardcoded in the interface */
//added by prj
	ret = ms5611_spi_read(0, (void *)&raw, 0);
	
	if (ret < 0) {
		//perf_count(_comms_errors);
		//perf_end(_sample_perf);
		return ret;
	}

	/* handle a measurement */
	if (_measure_phase == 0) {

		/* temperature offset (in ADC units) */
		int32_t dT = (int32_t)raw - ((int32_t)_prom.s.c5_reference_temp << 8);

		/* absolute temperature in centidegrees - note intermediate value is outside 32-bit range */
		_TEMP = 2000 + (int32_t)(((int64_t)dT * _prom.s.c6_temp_coeff_temp) >> 23);

		/* base sensor scale/offset values */
		_SENS = ((int64_t)_prom.s.c1_pressure_sens << 15) + (((int64_t)_prom.s.c3_temp_coeff_pres_sens * dT) >> 8);
		_OFF  = ((int64_t)_prom.s.c2_pressure_offset << 16) + (((int64_t)_prom.s.c4_temp_coeff_pres_offset * dT) >> 7);

		/* temperature compensation */
		if (_TEMP < 2000) {

			int32_t T2 = POW2(dT) >> 31;

			int64_t f = POW2((int64_t)_TEMP - 2000);
			int64_t OFF2 = 5 * f >> 1;
			int64_t SENS2 = 5 * f >> 2;

			if (_TEMP < -1500) {
				int64_t f2 = POW2(_TEMP + 1500);
				OFF2 += 7 * f2;
				SENS2 += 11 * f2 >> 1;
			}

			_TEMP -= T2;
			_OFF  -= OFF2;
			_SENS -= SENS2;
		}

	} else {

		/* pressure calculation, result in Pa */
		int32_t P = (((raw * _SENS) >> 21) - _OFF) >> 15;
		_P1 = P * 0.01f;
		_T = _TEMP * 0.01f;

		/* generate a new report */
		report.temperature = _TEMP / 100.0f;
		report.pressure = P / 100.0f;		/* convert to millibar */

		/* altitude calculations based on http://www.kansasflyer.org/index.asp?nav=Avi&sec=Alti&tab=Theory&pg=1 */

		/*
		 * PERFORMANCE HINT:
		 *
		 * The single precision calculation is 50 microseconds faster than the double
		 * precision variant. It is however not obvious if double precision is required.
		 * Pending more inspection and tests, we'll leave the double precision variant active.
		 *
		 * Measurements:
		 * 	double precision: ms5611_read: 992 events, 258641us elapsed, min 202us max 305us
		 *	single precision: ms5611_read: 963 events, 208066us elapsed, min 202us max 241us
		 */

		/* tropospheric properties (0-11km) for standard atmosphere */
		const double T1 = 15.0 + 273.15;	/* temperature at base height in Kelvin */
		const double a  = -6.5 / 1000;	/* temperature gradient in degrees per metre */
		const double g  = 9.80665;	/* gravity constant in m/s/s */
		const double R  = 287.05;	/* ideal gas constant in J/kg/K */

		/* current pressure at MSL in kPa */
		double p1 = _msl_pressure / 1000.0;

		/* measured pressure in kPa */
		double p = P / 1000.0;

		/*
		 * Solve:
		 *
		 *     /        -(aR / g)     \
		 *    | (p / p1)          . T1 | - T1
		 *     \                      /
		 * h = -------------------------------  + h1
		 *                   a
		 */
		report.altitude = (((pow((p / p1), (-(a * R) / g))) * T1) - T1) / a;

		/* publish it */
		if (!(_pub_blocked)) {
			/* publish it */
			orb_publish(ORB_ID(sensor_baro), _baro_topic, &report);
		}

		if (xRingBufferForce(_reports, &report, sizeof(report))) {
	//		perf_count(_buffer_overflows);
		}

		/* notify anyone waiting for data */
		poll_notify(POLLIN);
	}

	/* update the measurement state machine */
	INCREMENT(_measure_phase, MS5611_MEASUREMENT_RATIO + 1);

//	perf_end(_sample_perf);

	return OK;
}

void
MS5611::print_info()
{
	//perf_print_counter(_sample_perf);
	//perf_print_counter(_comms_errors);
	//perf_print_counter(_buffer_overflows);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	vRingBufferPrintInfo(_reports, "report queue");
	printf("TEMP:           %d\n", _TEMP);
	printf("SENS:           %lld\n", _SENS);
	printf("OFF:            %lld\n", _OFF);
	printf("P1:              %.3f\n", (double)_P1);
	printf("T:              %.3f\n", (double)_T);
	printf("MSL pressure:   %10.4f\n", (double)(_msl_pressure / 100.f));

	printf("factory_setup             %u\n", _prom.s.factory_setup);
	printf("c1_pressure_sens          %u\n", _prom.s.c1_pressure_sens);
	printf("c2_pressure_offset        %u\n", _prom.s.c2_pressure_offset);
	printf("c3_temp_coeff_pres_sens   %u\n", _prom.s.c3_temp_coeff_pres_sens);
	printf("c4_temp_coeff_pres_offset %u\n", _prom.s.c4_temp_coeff_pres_offset);
	printf("c5_reference_temp         %u\n", _prom.s.c5_reference_temp);
	printf("c6_temp_coeff_temp        %u\n", _prom.s.c6_temp_coeff_temp);
	printf("serial_and_crc            %u\n", _prom.s.serial_and_crc);
}

/**
 * Local functions in support of the shell command.
 */
namespace ms5611
{

MS5611	*g_dev[MS5611_MAX_INSTANCE];//用extern_bus作为索引


extern bool crc4(uint16_t *n_prom);
void	start(bool external_bus);
void	test(bool external_bus);
void	reset(bool external_bus);
void	info(bool external_bus);
void	calibrate(unsigned altitude, bool external_bus);
void	usage();

/**
 * MS5611 crc4 cribbed from the datasheet
 */
bool
crc4(uint16_t *n_prom)
{
	int16_t cnt;
	uint16_t n_rem;
	uint16_t crc_read;
	uint8_t n_bit;

	n_rem = 0x00;

	/* save the read crc */
	crc_read = n_prom[7];

	/* remove CRC byte */
	n_prom[7] = (0xFF00 & (n_prom[7]));

	for (cnt = 0; cnt < 16; cnt++) {
		/* uneven bytes */
		if (cnt & 1) {
			n_rem ^= (uint8_t)((n_prom[cnt >> 1]) & 0x00FF);

		} else {
			n_rem ^= (uint8_t)(n_prom[cnt >> 1] >> 8);
		}

		for (n_bit = 8; n_bit > 0; n_bit--) {
			if (n_rem & 0x8000) {
				n_rem = (n_rem << 1) ^ 0x3000;

			} else {
				n_rem = (n_rem << 1);
			}
		}
	}

	/* final 4 bit remainder is CRC value */
	n_rem = (0x000F & (n_rem >> 12));
	n_prom[7] = crc_read;

	/* return true if CRCs match */
	return (0x000F & crc_read) == (n_rem ^ 0x00);
}


/**
 * Start the driver.
 *
 * This function call only returns once the driver
 * started or failed to detect the sensor.
 */
void
start(bool external_bus)
{
	int fd;
//	prom_u prom_buf;
	Print_Info("MS5611 -------------------------Into start ");
	
	if (g_dev[external_bus] != NULL)
	{
		errx(0, "already started");
		return;
	}
	/* create the driver */
	if (external_bus) {
#ifdef PX4_SPI_BUS_EXT
		g_dev[external_bus] = new MS5611(PX4_SPI_BUS_EXT, MS5611_BARO_DEVICE_PATH_EXT, (ESpi_device_id)ESPI_DEVICE_TYPE_BARO);
#else
		errx(0, "External SPI not available");
		return;
#endif
	} else {
		g_dev[external_bus] = new MS5611(SPI_DEVICE_ID_FOR_SENSOR, MS5611_BARO_DEVICE_PATH_INT, (ESpi_device_id)ESPI_DEVICE_TYPE_BARO);
	}
	if (g_dev[external_bus] == NULL)
		goto fail;
	
	
	if (OK != g_dev[external_bus]->init())
		goto fail;
	
	/* set the poll rate to default, starts automatic data collection */
	if(external_bus)
		fd = open(MS5611_BARO_DEVICE_PATH_EXT, O_RDONLY);
	else
		fd = open(MS5611_BARO_DEVICE_PATH_INT, O_RDONLY);
	
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
	struct baro_report report;
	size_t sz;
	int ret;
	int fd;
	

	/* get the driver */
	if(external_bus)
	{
		fd = open(MS5611_BARO_DEVICE_PATH_EXT, O_RDONLY);
		if (fd < 0)
			Print_Err("%s open failed\n", MS5611_BARO_DEVICE_PATH_EXT);
			return;
	}
	else
	{
		fd = open(MS5611_BARO_DEVICE_PATH_INT, O_RDONLY);
		if (fd < 0)
			Print_Err("%s open failed\n", MS5611_BARO_DEVICE_PATH_INT);
			return;
	}
	
	
	/* do a simple demand read */
	sz = read(fd, (char*)&report, sizeof(report));

	if (sz != sizeof(report)) {
		Print_Err("immediate read failed\n");
		return ;
	}

	Print_Info("single read\n");
	Print_Info("pressure:    %10.4f\n", (double)report.pressure);
	Print_Info("altitude:    %11.4f\n", (double)report.altitude);
	Print_Info("temperature: %8.4f\n", (double)report.temperature);
	Print_Info("time:        %lld\n", report.timestamp);

	/* set the queue depth to 10 */
	if (OK != ioctl(fd, SENSORIOCSQUEUEDEPTH,10)) {
		Print_Err("failed to set queue depth\n");
		return ;
	}

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		Print_Err("failed to set 2Hz poll rate\n");
		return ;
	}

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			Print_Err("timed out waiting for sensor data\n");
			return;
		}

		/* now go get it */
		sz = read(fd, (char*)&report, sizeof(report));

		if (sz != sizeof(report)) {
			Print_Err("periodic read failed\n");
			return ;
		}

		Print_Info("periodic read %u\n", i);
		Print_Info("pressure:    %10.4f\n", (double)report.pressure);
		Print_Info("altitude:    %11.4f\n", (double)report.altitude);
		Print_Info("temperature: %8.4f\n", (double)report.temperature);
		Print_Info("time:        %lld\n", report.timestamp);
	}

	close(fd);

	return;
}

/**
 * Reset the driver.
 */
void
reset(bool external_bus)
{
	int fd;
	
	if(external_bus)
		fd = open(MS5611_BARO_DEVICE_PATH_EXT, O_RDONLY);
	else
		fd = open(MS5611_BARO_DEVICE_PATH_INT, O_RDONLY);
	
	if (fd < 0) {
		err(1, "failed ");
		return;
	}
	
	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
		return;
	}
	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
		return;
	}

	return;
}

/**
 * Print a little info about the driver.
 */
void
info(bool external_bus)
{
	if (g_dev[external_bus] == NULL)
	{
		errx(1, "driver not running\n");
		return;
	}
	printf("state @ %p\n", g_dev[external_bus]);
	g_dev[external_bus]->print_info();

	return;
}

/**
 * Calculate actual MSL pressure given current altitude
 */
void
calibrate(unsigned altitude, bool external_bus)
{
	struct baro_report report;
	float	pressure;
	float	p1;
	
	int fd;

	if(external_bus)
		fd = open(MS5611_BARO_DEVICE_PATH_EXT, O_RDONLY);
	else
		fd = open(MS5611_BARO_DEVICE_PATH_INT, O_RDONLY);
	
	if (fd < 0) {
		err(1, "open failed (try 'ms5611 start' if the driver is not running)");
			return;
	}

	/* start the sensor polling at max */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MAX)) {
		errx(1, "failed to set poll rate");
			return;
	}

	/* average a few measurements */
	pressure = 0.0f;

	for (unsigned i = 0; i < 20; i++) {
		struct pollfd fds;
		int ret;
		ssize_t sz;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 1000);

		if (ret != 1) {
			errx(1, "timed out waiting for sensor data");
				return;
		}

		/* now go get it */
		sz = read(fd, (char*)&report, sizeof(report));

		if (sz != sizeof(report)) {
			err(1, "sensor read failed");
				return;
		}

		pressure += report.pressure;
	}

	pressure /= 20;		/* average */
	pressure /= 10;		/* scale from millibar to kPa */

	/* tropospheric properties (0-11km) for standard atmosphere */
	const float T1 = 15.0 + 273.15;	/* temperature at base height in Kelvin */
	const float a  = -6.5 / 1000;	/* temperature gradient in degrees per metre */
	const float g  = 9.80665f;	/* gravity constant in m/s/s */
	const float R  = 287.05f;	/* ideal gas constant in J/kg/K */

	warnx("averaged pressure %10.4fkPa at %um", (double)pressure, altitude);

	p1 = pressure * (powf(((T1 + (a * (float)altitude)) / T1), (g / (a * R))));

	warnx("calculated MSL pressure %10.4fkPa", (double)p1);

	/* save as integer Pa */
	p1 *= 1000.0f;

	if (ioctl(fd, BAROIOCSMSLPRESSURE, (unsigned long)p1) != OK) {
		err(1, "BAROIOCSMSLPRESSURE");
			return;
	}

	close(fd);
	return ;
}

void
usage()
{
	warnx("missing command: try 'start', 'info', 'test', 'test2', 'reset', 'calibrate'");
	warnx("options:");
	warnx("    -X    (external I2C bus)");
	warnx("    -I    (intternal I2C bus)");
	warnx("    -S    (external SPI bus)");
	warnx("    -s    (internal SPI bus)");
}

} // namespace

using namespace ms5611;

int ms5611_main(int argc, char *argv[])
{
	bool external_bus = false;
	int ch;

	/* jump over start/off/etc and look at options first */
	while ((ch = drv_getopt(argc, argv, "X:")) != EOF) {
		switch (ch) {
			case 'X':
				external_bus = true;
				break;

		default:
			ms5611::usage();
			return 0;
		}
	}

	const char *verb = argv[drv_optind];

	/*
	 * Start/load the driver.
	 */
	Print_Info("verb=%s external_bus = %d\r\n",verb, external_bus);
	if (!strcmp(verb, "start")) {
		ms5611::start(external_bus);
			return 0;
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		ms5611::test(external_bus);
			return 0;
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		ms5611::reset(external_bus);
			return 0;
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		ms5611::info(external_bus);
			return 0;
	}

	/*
	 * Perform MSL pressure calibration given an altitude in metres
	 */
	if (!strcmp(verb, "calibrate")) {
		if (argc < 2) {
			errx(1, "missing altitude");
				return 0;
			
		}

		long altitude = strtol(argv[drv_optind + 1], NULL, 10);

		ms5611::calibrate(altitude, external_bus);
			return 0;
	}

	errx(1, "unrecognised command, try 'start', 'test', 'reset' or 'info'");
}

