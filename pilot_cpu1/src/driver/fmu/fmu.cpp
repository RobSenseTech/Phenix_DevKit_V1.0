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
 * @file fmu.cpp
 *
 * Driver/configurator for the PX4 FMU multi-purpose port on v1 and v2 boards.
 */
#include "device/cdev.h"
#include "FreeRTOS_Print.h"
#include "math.h"
#include "driver_define.h"
#include "board_config.h"
#include <uORB/uORB.h>
#include "timers.h"
#include "sleep.h"
#include <assert.h>
#include "stdio.h"
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <unistd.h>

#include "driver.h"
#include "drv_mixer.h"
#include "drv_rc_input.h"
#include "drv_gpio.h"
#include "drv_pwm_output.h"
#include "Phx_define.h"

#include "rc/sbus.h"
#include <systemlib/mixer/mixer.h>
#include <systemlib/pwm_limit/pwm_limit.h>

#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_controls_0.h>
#include <uORB/topics/actuator_controls_1.h>
#include <uORB/topics/actuator_controls_2.h>
#include <uORB/topics/actuator_controls_3.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>


#ifdef HRT_PPM_CHANNEL
# include <systemlib/ppm_decode.h>
#endif

//using namespace pilot::driver;

/*
 * This is the analog to FMU_INPUT_DROP_LIMIT_US on the IO side
 */

#define CONTROL_INPUT_DROP_LIMIT_MS		2
#define NAN_VALUE	(0.0f/0.0f)

class PX4FMU : public device::CDev
{
public:
	typedef enum Mode {
		MODE_NONE = 0,
		MODE_2PWM,
		MODE_4PWM,
		MODE_6PWM,
		MODE_8PWM,
	}px4fmu_mode_t;
	
	PX4FMU();
	virtual ~PX4FMU();

	virtual int	ioctl(file *filp, int cmd, unsigned long arg);
	virtual ssize_t	write(file *filp, const char *buffer, size_t len);

	virtual int	init();

	int		set_mode(Mode mode);

	int		set_pwm_alt_rate(unsigned rate);
	int		set_pwm_alt_channels(unsigned channels);

	int		set_i2c_bus_clock(unsigned bus, unsigned clock_hz);
	int     get_full_actuators_number();

private:
	static const unsigned _max_actuators = PILOT_MAX_ACTUATOR;

	Mode		_mode;
	unsigned	_pwm_default_rate;
	unsigned	_pwm_alt_rate;
	uint32_t	_pwm_alt_rate_channels;
	unsigned	_current_update_rate;
	TimerHandle_t _work;
	int		_armed_sub;
	int		_param_sub;
	struct rc_input_values	_rc_in;
	orb_advert_t	_to_input_rc;
	orb_advert_t	_outputs_pub;
	unsigned	_num_outputs;
	int		_class_instance;
	int		_sbus_fd;
	int		_dsm_fd;

	volatile bool	_initialized;
	bool		_servo_armed;
	bool		_pwm_on;

	MixerGroup	*_mixers;

	uint32_t	_groups_required;
	uint32_t	_groups_subscribed;
	int		_control_subs[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	actuator_controls_s _controls[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	orb_id_t	_control_topics[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	pollfd	_poll_fds[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	unsigned	_poll_fds_num;

	static pwm_limit_t	_pwm_limit;
	static actuator_armed_s	_armed;
	uint16_t	_failsafe_pwm[_max_actuators];
	uint16_t	_disarmed_pwm[_max_actuators];
	uint16_t	_min_pwm[_max_actuators];
	uint16_t	_max_pwm[_max_actuators];
	uint16_t	_reverse_pwm_mask;
	unsigned	_num_failsafe_set;
	unsigned	_num_disarmed_set;

	static bool	arm_nothrottle() { return (_armed.prearmed && !_armed.armed); }

	static void	cycle_trampoline(void* xTimer,void *arg);

	void		cycle();
	void		work_start();
	void		work_stop();

	static int	control_callback(uintptr_t handle,
					 uint8_t control_group,
					 uint8_t control_index,
					 float &input);
	void		subscribe();
	int		set_pwm_rate(unsigned rate_map, unsigned default_rate, unsigned alt_rate);
	int		pwm_ioctl(file *filp, int cmd, unsigned long arg);
	void		update_pwm_rev_mask();
	void	publish_pwm_outputs(uint16_t *values, size_t numvalues);

	struct GPIOConfig {
		uint32_t	input;
		uint32_t	output;
		uint32_t	alt;
	};

	void		gpio_reset(void);
	void		sensor_reset(int ms);
	void		peripheral_reset(int ms);
	void		gpio_set_function(uint32_t gpios, int function);
	void		gpio_write(uint32_t gpios, int function);
	uint32_t	gpio_read(void);
	int		gpio_ioctl(file *filp, int cmd, unsigned long arg);

	/* do not allow to copy due to ptr data members */
	PX4FMU(const PX4FMU &);
	PX4FMU operator=(const PX4FMU &);
};

pwm_limit_t		PX4FMU::_pwm_limit;
actuator_armed_s	PX4FMU::_armed = {};

PX4FMU	*g_fmu;

PX4FMU::PX4FMU() :
	CDev("fmu", PX4FMU_DEVICE_PATH),
	_mode(MODE_NONE),
	_pwm_default_rate(50),
	_pwm_alt_rate(50),
	_pwm_alt_rate_channels(0),
	_current_update_rate(0),
	_work(NULL),
	_armed_sub(-1),
	_param_sub(-1),
	_rc_in{},
	_to_input_rc(NULL),
	_outputs_pub(NULL),
	_num_outputs(0),
	_class_instance(0),
	_sbus_fd(-1),
	_dsm_fd(-1),
	_initialized(false),
	_servo_armed(false),
	_pwm_on(false),
	_mixers(NULL),
	_groups_required(0),
	_groups_subscribed(0),
	_control_subs{ -1},
	_poll_fds_num(0),
	_failsafe_pwm{0},
	_disarmed_pwm{0},
	_reverse_pwm_mask(0),
	_num_failsafe_set(0),
	_num_disarmed_set(0)
{
	unsigned i;
	for (i = 0; i < _max_actuators; i++) {
		_min_pwm[i] = PWM_DEFAULT_MIN;
		_max_pwm[i] = PWM_DEFAULT_MAX;
	}

	_control_topics[0] = ORB_ID(actuator_controls_0);
	_control_topics[1] = ORB_ID(actuator_controls_1);
	_control_topics[2] = ORB_ID(actuator_controls_2);
	_control_topics[3] = ORB_ID(actuator_controls_3);

	memset(_controls, 0, sizeof(_controls));
	memset(_poll_fds, 0, sizeof(_poll_fds));

#ifdef HRT_PPM_CHANNEL
	// rc input, published to ORB
	memset(&_rc_in, 0, sizeof(_rc_in));
	_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_PPM;
#endif
    Print_Info("timer=%d\n", USEC2TICK(CONTROL_INPUT_DROP_LIMIT_MS * 1000));
	_work = xTimerCreate("FMU_Timer",
						USEC2TICK(CONTROL_INPUT_DROP_LIMIT_MS * 1000),
						pdTRUE,
						(void*)51,
						PX4FMU::cycle_trampoline,
						this);
}

PX4FMU::~PX4FMU()
{
	if (_initialized) {
		/* tell the task we want it to go away */
		work_stop();

		int i = 10;

		do {
			/* wait 50ms - it should wake every 100ms or so worst-case */
			usleep(50000);
			i--;

		} while (_initialized && i > 0);
	}

	/* clean up the alternate device node */
	unregister_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH, _class_instance);

	g_fmu = NULL;
}

int
PX4FMU::init()
{
	int ret;

	assert(!_initialized);

	/* do regular cdev init */
	ret = CDev::init();

	if (ret != OK) {
		return ret;
	}

	/* try to claim the generic PWM output device node as well - it's OK if we fail at this */
	_class_instance = register_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH);

	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		/* lets not be too verbose */
	} else if (_class_instance < 0) {
		warnx("FAILED registering class device\n");
	}

	work_start();

	return OK;
}

int PX4FMU::get_full_actuators_number() {

	if(_max_actuators < 2) {
		return MODE_NONE;
	} else if (_max_actuators == 2) {
		return MODE_2PWM;
	} else if (_max_actuators < 4) {
		return MODE_NONE;
	} else if (_max_actuators == 4) {
		return MODE_4PWM;
	} else if (_max_actuators < 6) {
		return MODE_NONE;
	} else if (_max_actuators == 6) {
		return MODE_6PWM;
	} else if (_max_actuators < 8) {
		return MODE_NONE;
	} else if (_max_actuators == 8) {
		return MODE_8PWM;
	} else {
		return MODE_NONE;
	}
}

int
PX4FMU::set_mode(Mode mode)
{
	/*
	 * Configure for PWM output.
	 *
	 * Note that regardless of the configured mode, the task is always
	 * listening and mixing; the mode just selects which of the channels
	 * are presented on the output pins.
	 */
	switch (mode) {
	case MODE_2PWM:	// v1 multi-port with flow control lines as PWM
		DEVICE_DEBUG("MODE_2PWM\n");

		/* default output rates */
		_pwm_default_rate = 50;
		_pwm_alt_rate = 50;
		_pwm_alt_rate_channels = 0;

		/* XXX magic numbers */
		up_pwm_servo_init(0x3);
		set_pwm_rate(_pwm_alt_rate_channels, _pwm_default_rate, _pwm_alt_rate);

		break;

	case MODE_4PWM: // v1 or v2 multi-port as 4 PWM outs
		DEVICE_DEBUG("MODE_4PWM\n");

		/* default output rates */
		_pwm_default_rate = 50;
		_pwm_alt_rate = 50;
		_pwm_alt_rate_channels = 0;

		/* XXX magic numbers */
		up_pwm_servo_init(0xf);
		set_pwm_rate(_pwm_alt_rate_channels, _pwm_default_rate, _pwm_alt_rate);

		break;

	case MODE_6PWM: // v2 PWMs as 6 PWM outs
		DEVICE_DEBUG("MODE_6PWM\n");

		/* default output rates */
		_pwm_default_rate = 50;
		_pwm_alt_rate = 50;
		_pwm_alt_rate_channels = 0;

		/* XXX magic numbers */
		up_pwm_servo_init(0x3f);
		set_pwm_rate(_pwm_alt_rate_channels, _pwm_default_rate, _pwm_alt_rate);

		break;


	case MODE_8PWM: // AeroCore PWMs as 8 PWM outs
		DEVICE_DEBUG("MODE_8PWM\n");
		/* default output rates */
		_pwm_default_rate = 50;
		_pwm_alt_rate = 50;
		_pwm_alt_rate_channels = 0;

		/* XXX magic numbers */
		up_pwm_servo_init(0xff);
		set_pwm_rate(_pwm_alt_rate_channels, _pwm_default_rate, _pwm_alt_rate);
		break;

	case MODE_NONE:
		DEVICE_DEBUG("MODE_NONE\n");

		_pwm_default_rate = 10;	/* artificially reduced output rate */
		_pwm_alt_rate = 10;
		_pwm_alt_rate_channels = 0;

		/* disable servo outputs - no need to set rates */
		up_pwm_servo_deinit();

		break;

	default:
		return -EINVAL;
	}

	_mode = mode;
	return OK;
}

int
PX4FMU::set_pwm_rate(unsigned rate_map, unsigned default_rate, unsigned alt_rate)
{
	Print_Info("set_pwm_rate %x %u %u\n", rate_map, default_rate, alt_rate);

	for (unsigned pass = 0; pass < 2; pass++) {
		for (unsigned group = 0; group < _max_actuators; group++) {

			// get the channel mask for this rate group
			uint32_t mask = up_pwm_servo_get_rate_group(group);
			if (mask == 0) {
				continue;
			}

			// all channels in the group must be either default or alt-rate
			uint32_t alt = rate_map & mask;

			if (pass == 0) {
				// preflight
				if ((alt != 0) && (alt != mask)) {
					warn("rate group %u mask %x bad overlap %x\n", group, mask, alt);
					// not a legal map, bail
					return -EINVAL;
				}

			} else {
				// set it - errors here are unexpected
				if (alt != 0) {
					if (up_pwm_servo_set_rate_group_update(group, _pwm_alt_rate) != OK) {
						warn("rate group set alt failed\n");
						return -EINVAL;
					}

				} else {
					if (up_pwm_servo_set_rate_group_update(group, _pwm_default_rate) != OK) {
						warn("rate group set default failed\n");
						return -EINVAL;
					}
				}
			}
		}
	}

	_pwm_alt_rate_channels = rate_map;
	_pwm_default_rate = default_rate;
	_pwm_alt_rate = alt_rate;

	return OK;
}

int
PX4FMU::set_pwm_alt_rate(unsigned rate)
{
	printf("all pwm set alt rate:%d\n", rate);
	return set_pwm_rate(_pwm_alt_rate_channels, _pwm_default_rate, rate);
}

int
PX4FMU::set_pwm_alt_channels(unsigned channels)
{
	printf("pwm %d set alt rate:%d\n", channels, _pwm_alt_rate);
	return set_pwm_rate(channels, _pwm_default_rate, _pwm_alt_rate);
}

int
PX4FMU::set_i2c_bus_clock(unsigned bus, unsigned clock_hz)
{
	return DEV_FAILURE;
}

void
PX4FMU::subscribe()
{
	/* subscribe/unsubscribe to required actuator control groups */
	uint32_t sub_groups = _groups_required & ~_groups_subscribed;
	uint32_t unsub_groups = _groups_subscribed & ~_groups_required;
	_poll_fds_num = 0;

	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (sub_groups & (1 << i)) {
			DEVICE_DEBUG("subscribe to actuator_controls_%d\n", i);
			_control_subs[i] = orb_subscribe(_control_topics[i]);
		}

		if (unsub_groups & (1 << i)) {
			DEVICE_DEBUG("unsubscribe from actuator_controls_%d\n", i);
			::close(_control_subs[i]);
			_control_subs[i] = -1;
		}

		if (_control_subs[i] > 0) {
			_poll_fds[_poll_fds_num].fd = _control_subs[i];
			_poll_fds[_poll_fds_num].events = POLLIN;
			_poll_fds_num++;
		}
	}
}

void
PX4FMU::update_pwm_rev_mask()
{
	_reverse_pwm_mask = REVERSE_PWM_MASK;
}

void
PX4FMU::publish_pwm_outputs(uint16_t *values, size_t numvalues)
{
	actuator_outputs_s outputs;
	outputs.noutputs = numvalues;
	outputs.timestamp = hrt_absolute_time();

	for (size_t i = 0; i < _max_actuators; ++i) {
		outputs.output[i] = i < numvalues ? (float)values[i] : 0;
	}

	if (_outputs_pub == NULL) {
		int instance = -1;
		_outputs_pub = orb_advertise_multi(ORB_ID(actuator_outputs), &outputs, &instance, ORB_PRIO_DEFAULT);

	} else {
		orb_publish(ORB_ID(actuator_outputs), _outputs_pub, &outputs);
	}
}


void
PX4FMU::work_start()
{
	xTimerStart(_work, portMAX_DELAY);
}

void
PX4FMU::cycle_trampoline(void* xTimer, void *arg)
{
	PX4FMU *dev = reinterpret_cast<PX4FMU *>(arg);

	dev->cycle();
}

void
PX4FMU::cycle()
{
	if (!_initialized) {

		/* force a reset of the update rate */
		_current_update_rate = 0;

		_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
		//_param_sub = orb_subscribe(ORB_ID(parameter_update));

		/* initialize PWM limit lib */
		pwm_limit_init(&_pwm_limit);

		update_pwm_rev_mask();

#ifdef RC_SERIAL_PORT
		_sbus_fd = sbus_init(RC_SERIAL_PORT, true);
#endif
		_initialized = true;
	}

	if (_groups_subscribed != _groups_required) {
		subscribe();
		_groups_subscribed = _groups_required;
		/* force setting update rate */
		_current_update_rate = 0;
	}

	/*
	 * Adjust actuator topic update rate to keep up with
	 * the highest servo update rate configured.
	 *
	 * We always mix at max rate; some channels may update slower.
	 */
	unsigned max_rate = (_pwm_default_rate > _pwm_alt_rate) ? _pwm_default_rate : _pwm_alt_rate;

	if (_current_update_rate != max_rate) {
		_current_update_rate = max_rate;
		int update_rate_in_ms = int(1000 / _current_update_rate);

		/* reject faster than 500 Hz updates */
		if (update_rate_in_ms < 2) {
			update_rate_in_ms = 2;
		}

		/* reject slower than 10 Hz updates */
		if (update_rate_in_ms > 100) {
			update_rate_in_ms = 100;
		}

		DEVICE_DEBUG("adjusted actuator update interval to %ums\n", update_rate_in_ms);

		for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
			if (_control_subs[i] > 0) {
				orb_set_interval(_control_subs[i], update_rate_in_ms);
			}
		}

		// set to current max rate, even if we are actually checking slower/faster
		_current_update_rate = max_rate;
	}

	/* check if anything updated */
    //从mkblctrl-blctrl电子模块驱动拿数据，貌似没用到，而且poll里面也在等待定时器导致定时器卡死
	int ret = 0;//::poll(_poll_fds, _poll_fds_num, 0);

	/* this would be bad... */
	if (ret < 0) {
		DEVICE_LOG("poll error %d\n", ret);

	} else if (ret == 0) {
		/* timeout: no control data, switch to failsafe values */
//			warnx("no PWM: failsafe\n");

	} else {

		/* get controls for required topics */
		unsigned poll_id = 0;

		for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
			if (_control_subs[i] > 0) {
				if (_poll_fds[poll_id].revents & POLLIN) {
					orb_copy(_control_topics[i], _control_subs[i], &_controls[i]);
				}

				poll_id++;
			}
		}

		/* can we mix? */
		if (_mixers != NULL) {

			size_t num_outputs;

			switch (_mode) {
			case MODE_2PWM:
				num_outputs = 2;
				break;

			case MODE_4PWM:
				num_outputs = 4;
				break;

			case MODE_6PWM:
				num_outputs = 6;
				break;

			case MODE_8PWM:
				num_outputs = 8;
				break;

			default:
				num_outputs = 0;
				break;
			}

			/* do mixing */
			float outputs[_max_actuators];
			num_outputs = _mixers->mix(outputs, num_outputs, NULL);

			/* disable unused ports by setting their output to NaN */
			for (size_t i = 0; i < sizeof(outputs) / sizeof(outputs[0]); i++) {
				if (i >= num_outputs) {
					outputs[i] = NAN_VALUE;
				}
			}

			uint16_t pwm_limited[_max_actuators];

			/* the PWM limit call takes care of out of band errors, NaN and constrains */
			pwm_limit_calc(_servo_armed, arm_nothrottle(), num_outputs, _reverse_pwm_mask, _disarmed_pwm, _min_pwm, _max_pwm,
				       outputs, pwm_limited, &_pwm_limit);

			/* output to the servos */
			for (size_t i = 0; i < num_outputs; i++) {
				up_pwm_servo_set(i, pwm_limited[i]);
			}

			publish_pwm_outputs(pwm_limited, num_outputs);
		}
	}

	/* check arming state */
	bool updated = false;
	orb_check(_armed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);

		/* update the armed status and check that we're not locked down */
		bool set_armed = (_armed.armed || _armed.prearmed) && !_armed.lockdown;

		if (_servo_armed != set_armed) {
			_servo_armed = set_armed;
		}

		/* update PWM status if armed or if disarmed PWM values are set */
		bool pwm_on = (set_armed || _num_disarmed_set > 0);

		if (_pwm_on != pwm_on) {
			_pwm_on = pwm_on;
			up_pwm_servo_arm(pwm_on);
		}
	}
/* TODO:F
	orb_check(_param_sub, &updated);

	if (updated) {
		parameter_update_s pupdate;
		orb_copy(ORB_ID(parameter_update), _param_sub, &pupdate);

		update_pwm_rev_mask();
	}
*/
	bool rc_updated = false;

#ifdef RC_SERIAL_PORT
	bool sbus_failsafe, sbus_frame_drop;
	uint16_t raw_rc_values[input_rc_s::RC_INPUT_MAX_CHANNELS];
	uint16_t raw_rc_count;
	bool sbus_updated = sbus_input(_sbus_fd, &raw_rc_values[0], &raw_rc_count, &sbus_failsafe, &sbus_frame_drop,
				       input_rc_s::RC_INPUT_MAX_CHANNELS);

	if (sbus_updated) {
		// we have a new PPM frame. Publish it.
		_rc_in.channel_count = raw_rc_count;

		if (_rc_in.channel_count > input_rc_s::RC_INPUT_MAX_CHANNELS) {
			_rc_in.channel_count = input_rc_s::RC_INPUT_MAX_CHANNELS;
		}

		for (uint8_t i = 0; i < _rc_in.channel_count; i++) {
			_rc_in.values[i] = raw_rc_values[i];
       //     Print_Info("value[%d]=%d\n", i, _rc_in.values[i]);
		}

		_rc_in.timestamp_publication = hrt_absolute_time();
		_rc_in.timestamp_last_signal = _rc_in.timestamp_publication;

		_rc_in.rc_ppm_frame_length = 0;
		_rc_in.rssi = (!sbus_frame_drop) ? RC_INPUT_RSSI_MAX : 0;
		_rc_in.rc_failsafe = false;
		_rc_in.rc_lost = false;
		_rc_in.rc_lost_frame_count = 0;
		_rc_in.rc_total_frame_count = 0;

		rc_updated = true;
	}
#endif

#ifdef HRT_PPM_CHANNEL

	// see if we have new PPM input data
	if ((ppm_last_valid_decode != _rc_in.timestamp_last_signal) &&
		ppm_decoded_channels > 3) {
		// we have a new PPM frame. Publish it.
		_rc_in.channel_count = ppm_decoded_channels;

		if (_rc_in.channel_count > input_rc_s::RC_INPUT_MAX_CHANNELS) {
			_rc_in.channel_count = input_rc_s::RC_INPUT_MAX_CHANNELS;
		}

		for (uint8_t i = 0; i < _rc_in.channel_count; i++) {
			_rc_in.values[i] = ppm_buffer[i];
		}

		_rc_in.timestamp_publication = ppm_last_valid_decode;
		_rc_in.timestamp_last_signal = ppm_last_valid_decode;

		_rc_in.rc_ppm_frame_length = ppm_frame_length;
		_rc_in.rssi = RC_INPUT_RSSI_MAX;
		_rc_in.rc_failsafe = false;
		_rc_in.rc_lost = false;
		_rc_in.rc_lost_frame_count = 0;
		_rc_in.rc_total_frame_count = 0;
	
		rc_updated = true;
	}

#endif

	if (rc_updated) {
		/* lazily advertise on first publication */
		if (_to_input_rc == NULL) {
			_to_input_rc = orb_advertise(ORB_ID(input_rc), &_rc_in);

		} else {
			orb_publish(ORB_ID(input_rc), _to_input_rc, &_rc_in);
		}
	}

//	xTimerStart(_work, (2/portTICK_PERIOD_MS));
}

void PX4FMU::work_stop()
{
	xTimerStop(_work, portMAX_DELAY);

	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (_control_subs[i] > 0) {
			::close(_control_subs[i]);
			_control_subs[i] = -1;
		}
	}

	::close(_armed_sub);
	//::close(_param_sub);

	/* make sure servos are off */
	up_pwm_servo_deinit();

	DEVICE_LOG("stopping\n");

	/* note - someone else is responsible for restoring the GPIO config */

	/* tell the dtor that we are exiting */
	_initialized = false;
}

int
PX4FMU::control_callback(uintptr_t handle,
			 uint8_t control_group,
			 uint8_t control_index,
			 float &input)
{
	const actuator_controls_s *controls = (actuator_controls_s *)handle;

	input = controls[control_group].control[control_index];

	/* limit control input */
	if (input > 1.0f) {
		input = 1.0f;

	} else if (input < -1.0f) {
		input = -1.0f;
	}

	/* motor spinup phase - lock throttle to zero */
	if (_pwm_limit.state == PWM_LIMIT_STATE_RAMP) {
		if (control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE &&
		    control_index == actuator_controls_s::INDEX_THROTTLE) {
			/* limit the throttle output to zero during motor spinup,
			 * as the motors cannot follow any demand yet
			 */
			input = 0.0f;
		}
	}

	/* throttle not arming - mark throttle input as invalid */
	if (arm_nothrottle()) {
		if (control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE &&
		    control_index == actuator_controls_s::INDEX_THROTTLE) {
			/* set the throttle to an invalid value */
			input = NAN_VALUE;
		}
	}

	return 0;
}

int
PX4FMU::ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret;

	/* try it as a GPIO ioctl first */
	ret = gpio_ioctl(filp, cmd, arg);

	if (ret == -ENOTTY) {
		return ret;
	}

	/* if we are in valid PWM mode, try it as a PWM ioctl as well */
	switch (_mode) {
	case MODE_2PWM:
	case MODE_4PWM:
	case MODE_6PWM:
	case MODE_8PWM:

		ret = pwm_ioctl(filp, cmd, arg);
		break;

	default:
		DEVICE_DEBUG("not in a PWM mode\n");
		break;
	}

	/* if nobody wants it, let CDev have it */
	if (ret == -ENOTTY) {
		ret = CDev::ioctl(filp, cmd, arg);
	}

	return ret;
}

int
PX4FMU::pwm_ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret = OK;

	lock();

	switch (cmd) {
	case PWM_SERVO_ARM:
		printf("HEBIN fmu.cpp PWM_SERVO_ARM\n");
		up_pwm_servo_arm(true);
		break;

	case PWM_SERVO_SET_ARM_OK:
	case PWM_SERVO_CLEAR_ARM_OK:
	case PWM_SERVO_SET_FORCE_SAFETY_OFF:
	case PWM_SERVO_SET_FORCE_SAFETY_ON:
		// these are no-ops, as no safety switch
		printf("HEBIN empty\n");
		break;

	case PWM_SERVO_DISARM:
		printf("HEBIN fmu.cpp PWM_SERVO_DISARM\n");
		up_pwm_servo_arm(false);
		break;

	case PWM_SERVO_GET_DEFAULT_UPDATE_RATE:
		*(uint32_t *)arg = _pwm_default_rate;
		break;

	case PWM_SERVO_SET_UPDATE_RATE:
		printf("all pwm update rate: _pwm_default_rate=%d _pwm_alt_rate=%d\n", _pwm_default_rate, arg);
		ret = set_pwm_rate(_pwm_alt_rate_channels, _pwm_default_rate, arg);
		break;

	case PWM_SERVO_GET_UPDATE_RATE:
		*(uint32_t *)arg = _pwm_alt_rate;
		break;

	case PWM_SERVO_SET_SELECT_UPDATE_RATE:
		printf("pwm %d update rate: _pwm_default_rate=%d _pwm_alt_rate=%d\n", arg, _pwm_default_rate, _pwm_alt_rate);
		ret = set_pwm_rate(arg, _pwm_default_rate, _pwm_alt_rate);
		break;

	case PWM_SERVO_GET_SELECT_UPDATE_RATE:
		*(uint32_t *)arg = _pwm_alt_rate_channels;
		break;

	case PWM_SERVO_SET_FAILSAFE_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > _max_actuators) {
				ret = -EINVAL;
				break;
			}

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (pwm->values[i] == 0) {
					/* ignore 0 */
				} else if (pwm->values[i] > PWM_HIGHEST_MAX) {
					_failsafe_pwm[i] = PWM_HIGHEST_MAX;

				} else if (pwm->values[i] < PWM_LOWEST_MIN) {
					_failsafe_pwm[i] = PWM_LOWEST_MIN;

				} else {
					_failsafe_pwm[i] = pwm->values[i];
				}
			}

			/*
			 * update the counter
			 * this is needed to decide if disarmed PWM output should be turned on or not
			 */
			_num_failsafe_set = 0;

			for (unsigned i = 0; i < _max_actuators; i++) {
				if (_failsafe_pwm[i] > 0) {
					_num_failsafe_set++;
				}
			}

			break;
		}

	case PWM_SERVO_GET_FAILSAFE_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _max_actuators; i++) {
				pwm->values[i] = _failsafe_pwm[i];
			}

			pwm->channel_count = _max_actuators;
			break;
		}

	case PWM_SERVO_SET_DISARMED_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > _max_actuators) {
				ret = -EINVAL;
				break;
			}

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (pwm->values[i] == 0) {
					/* ignore 0 */
				} else if (pwm->values[i] > PWM_HIGHEST_MAX) {
					_disarmed_pwm[i] = PWM_HIGHEST_MAX;

				} else if (pwm->values[i] < PWM_LOWEST_MIN) {
					_disarmed_pwm[i] = PWM_LOWEST_MIN;

				} else {
					_disarmed_pwm[i] = pwm->values[i];
				}
			}

			/*
			 * update the counter
			 * this is needed to decide if disarmed PWM output should be turned on or not
			 */
			_num_disarmed_set = 0;

			for (unsigned i = 0; i < _max_actuators; i++) {
				if (_disarmed_pwm[i] > 0) {
					_num_disarmed_set++;
				}
			}

			break;
		}

	case PWM_SERVO_GET_DISARMED_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _max_actuators; i++) {
				pwm->values[i] = _disarmed_pwm[i];
			}

			pwm->channel_count = _max_actuators;
			break;
		}

	case PWM_SERVO_SET_MIN_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > _max_actuators) {
				ret = -EINVAL;
				break;
			}

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (pwm->values[i] == 0) {
					/* ignore 0 */
				} else if (pwm->values[i] > PWM_HIGHEST_MIN) {
					_min_pwm[i] = PWM_HIGHEST_MIN;

				} else if (pwm->values[i] < PWM_LOWEST_MIN) {
					_min_pwm[i] = PWM_LOWEST_MIN;

				} else {
					_min_pwm[i] = pwm->values[i];
				}
			}

			break;
		}

	case PWM_SERVO_GET_MIN_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _max_actuators; i++) {
				pwm->values[i] = _min_pwm[i];
			}

			pwm->channel_count = _max_actuators;
			arg = (unsigned long)&pwm;
			break;
		}

	case PWM_SERVO_SET_MAX_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > _max_actuators) {
				ret = -EINVAL;
				break;
			}

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (pwm->values[i] == 0) {
					/* ignore 0 */
				} else if (pwm->values[i] < PWM_LOWEST_MAX) {
					_max_pwm[i] = PWM_LOWEST_MAX;

				} else if (pwm->values[i] > PWM_HIGHEST_MAX) {
					_max_pwm[i] = PWM_HIGHEST_MAX;

				} else {
					_max_pwm[i] = pwm->values[i];
				}
			}

			break;
		}

	case PWM_SERVO_GET_MAX_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _max_actuators; i++) {
				pwm->values[i] = _max_pwm[i];
			}

			pwm->channel_count = _max_actuators;
			arg = (unsigned long)&pwm;
			break;
		}

	case PWM_SERVO_SET(7):
	case PWM_SERVO_SET(6):
		if (_mode < MODE_8PWM) {
			ret = -EINVAL;
			break;
		}

	case PWM_SERVO_SET(5):
	case PWM_SERVO_SET(4):
		if (_mode < MODE_6PWM) {
			ret = -EINVAL;
			break;
		}

	/* FALLTHROUGH */
	case PWM_SERVO_SET(3):
	case PWM_SERVO_SET(2):
		if (_mode < MODE_4PWM) {
			ret = -EINVAL;
			break;
		}

	/* FALLTHROUGH */
	case PWM_SERVO_SET(1):
	case PWM_SERVO_SET(0):
		if (arg <= 2100) {
			up_pwm_servo_set(cmd - PWM_SERVO_SET(0), arg);

		} else {
			ret = -EINVAL;
		}

		break;

	case PWM_SERVO_GET(7):
	case PWM_SERVO_GET(6):
		if (_mode < MODE_8PWM) {
			ret = -EINVAL;
			break;
		}

	case PWM_SERVO_GET(5):
	case PWM_SERVO_GET(4):
		if (_mode < MODE_6PWM) {
			ret = -EINVAL;
			break;
		}

	/* FALLTHROUGH */
	case PWM_SERVO_GET(3):
	case PWM_SERVO_GET(2):
		if (_mode < MODE_4PWM) {
			ret = -EINVAL;
			break;
		}

	/* FALLTHROUGH */
	case PWM_SERVO_GET(1):
	case PWM_SERVO_GET(0):
		*(servo_position_t *)arg = up_pwm_servo_get(cmd - PWM_SERVO_GET(0));
		break;

	case PWM_SERVO_GET_RATEGROUP(0):
	case PWM_SERVO_GET_RATEGROUP(1):
	case PWM_SERVO_GET_RATEGROUP(2):
	case PWM_SERVO_GET_RATEGROUP(3):
	case PWM_SERVO_GET_RATEGROUP(4):
	case PWM_SERVO_GET_RATEGROUP(5):
	case PWM_SERVO_GET_RATEGROUP(6):
	case PWM_SERVO_GET_RATEGROUP(7):
		*(uint32_t *)arg = up_pwm_servo_get_rate_group(cmd - PWM_SERVO_GET_RATEGROUP(0));
		break;

	case PWM_SERVO_GET_COUNT:
	case MIXERIOCGETOUTPUTCOUNT:
		switch (_mode) {
		case MODE_8PWM:
			*(unsigned *)arg = 8;
			break;

		case MODE_6PWM:
			*(unsigned *)arg = 6;
			break;

		case MODE_4PWM:
			*(unsigned *)arg = 4;
			break;

		case MODE_2PWM:
			*(unsigned *)arg = 2;
			break;

		default:
			ret = -EINVAL;
			break;
		}

		break;

	case PWM_SERVO_SET_COUNT: {
			/* change the number of outputs that are enabled for
			 * PWM. This is used to change the split between GPIO
			 * and PWM under control of the flight config
			 * parameters. Note that this does not allow for
			 * changing a set of pins to be used for serial on
			 * FMUv1
			 */
			switch (arg) {
			case 0:
				set_mode(MODE_NONE);
				break;

			case 2:
				set_mode(MODE_2PWM);
				break;

			case 4:
				set_mode(MODE_4PWM);
				break;

			case 6:
				set_mode(MODE_6PWM);
				break;

			case 8:
				set_mode(MODE_8PWM);
				break;

			default:
				ret = -EINVAL;
				break;
			}

			break;
		}

	case MIXERIOCRESET:
		if (_mixers != NULL) {
			delete _mixers;
			_mixers = NULL;
			_groups_required = 0;
		}

		break;

	case MIXERIOCADDSIMPLE: {
			mixer_simple_s *mixinfo = (mixer_simple_s *)arg;

			SimpleMixer *mixer = new SimpleMixer(control_callback,
							     (uintptr_t)_controls, mixinfo);

			if (mixer->check()) {
				delete mixer;
				_groups_required = 0;
				ret = -EINVAL;

			} else {
				if (_mixers == NULL)
					_mixers = new MixerGroup(control_callback,
								 (uintptr_t)_controls);

				_mixers->add_mixer(mixer);
				_mixers->groups_required(_groups_required);
			}

			break;
		}

	case MIXERIOCLOADBUF: {
			const char *buf = (const char *)arg;
			unsigned buflen = strnlen(buf, 1024);

			if (_mixers == NULL) {
				_mixers = new MixerGroup(control_callback, (uintptr_t)_controls);
			}

			if (_mixers == NULL) {
				_groups_required = 0;
				ret = -ENOMEM;

			} else {

				ret = _mixers->load_from_buf(buf, buflen);

				if (ret != 0) {
					DEVICE_DEBUG("mixer load failed with %d\n", ret);
					delete _mixers;
					_mixers = NULL;
					_groups_required = 0;
					ret = -EINVAL;

				} else {

					_mixers->groups_required(_groups_required);
				}
			}

			break;
		}

	default:
		ret = -ENOTTY;
		break;
	}

	unlock();

	return ret;
}

/*
  this implements PWM output via a write() method, for compatibility
  with px4io
 */
ssize_t
PX4FMU::write(file *filp, const char *buffer, size_t len)
{
	unsigned count = len / 2;
	uint16_t values[6];

	if (count > 8) {
		// we have at most 8 outputs
		count = 8;
	}

	// allow for misaligned values
	memcpy(values, buffer, count * 2);

	for (uint8_t i = 0; i < count; i++) {
		if (values[i] != PWM_IGNORE_THIS_CHANNEL) {
			up_pwm_servo_set(i, values[i]);
		}
	}

	return count * 2;
}

void
PX4FMU::sensor_reset(int ms)
{
	warn("not need to support\n");
}

void
PX4FMU::peripheral_reset(int ms)
{
	warn("not need to support\n");
}

void
PX4FMU::gpio_reset(void)
{
    warn("gpio doesn't need to support\n");
}

void
PX4FMU::gpio_set_function(uint32_t gpios, int function)
{
    warn("gpio doesn't need to support\n");
}

void
PX4FMU::gpio_write(uint32_t gpios, int function)
{
	warn("gpio doesn't need to support\n");
}

uint32_t
PX4FMU::gpio_read(void)
{
	warn("gpio doesn't need to support\n");
	return 0;
}

int
PX4FMU::gpio_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int	ret = OK;
	return ret;
}

namespace
{

enum PortMode {
	PORT_MODE_UNSET = 0,
	PORT_FULL_GPIO,
	PORT_FULL_SERIAL,
	PORT_FULL_PWM,
	PORT_GPIO_AND_SERIAL,
	PORT_PWM_AND_SERIAL,
	PORT_PWM_AND_GPIO,
	PORT_PWM2,
	PORT_PWM4,
	PORT_PWM6,
	PORT_PWM8,
};

PortMode g_port_mode;

int
fmu_new_mode(PortMode new_mode)
{
	PX4FMU::Mode servo_mode = PX4FMU::MODE_NONE;

	switch (new_mode) {
	case PORT_FULL_GPIO:
	case PORT_MODE_UNSET:
		/* nothing more to do here */
		break;

	case PORT_FULL_PWM:
		servo_mode = (PX4FMU::Mode)g_fmu->get_full_actuators_number();
		break;

	case PORT_PWM2:
		/* select 4-pin PWM mode */
		servo_mode = PX4FMU::MODE_2PWM;
		break;

	case PORT_PWM4:
		/* select 4-pin PWM mode */
		servo_mode = PX4FMU::MODE_4PWM;
		break;

	case PORT_PWM6:
		/* select 6-pin PWM mode */
		servo_mode = PX4FMU::MODE_6PWM;
		break;

	case PORT_PWM8:
		/* select 8-pin PWM mode */
		servo_mode = PX4FMU::MODE_8PWM;
		break;

	case PORT_FULL_SERIAL:
	/* nothing more to do here */
		break;

	case PORT_GPIO_AND_SERIAL:
	/* nothing more to do here */
		break;

	case PORT_PWM_AND_SERIAL:
	/* nothing more to do here */
		break;

	case PORT_PWM_AND_GPIO:
	/* nothing more to do here */
		break;

	default:
		return -1;
	}

	/* (re)set the PWM output mode */
	g_fmu->set_mode(servo_mode);

	return OK;
}

int fmu_new_i2c_speed(unsigned bus, unsigned clock_hz)
{
	return OK;
}

int
fmu_start(void)
{
	int ret = OK;

	if (g_fmu == NULL) {

		g_fmu = new PX4FMU;

		if (g_fmu == NULL) {
			ret = -ENOMEM;

		} else {
			ret = g_fmu->init();

			if (ret != OK) {
				delete g_fmu;
				g_fmu = NULL;
			}
		}
	}

	return ret;
}

int
fmu_stop(void)
{
	int ret = OK;

	if (g_fmu != NULL) {

		delete g_fmu;
		g_fmu = NULL;
	}

	return ret;
}

void
sensor_reset(int ms)
{
	(void)ms;
}

void
peripheral_reset(int ms)
{
	(void)ms;
}

void
test(void)
{
	int	 fd;
	unsigned servo_count = 0;
	unsigned pwm_value = 1000;
	int	 direction = 1;
	int	 ret;

	fd = open(PX4FMU_DEVICE_PATH, O_RDWR);

	if (fd < 0) {
		errx(1, "open fail\n");
		return;
	}

	if (ioctl(fd, PWM_SERVO_ARM, 0) < 0) { err(1, "servo arm failed\n"); return; }
	if (ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count) != 0) {
		err(1, "Unable to get servo count\n");
		return;
	}

	warnx("Testing %u servos\n", (unsigned)servo_count);

	for (;;) {
		/* sweep all servos between 1000..2000 */
		servo_position_t servos[servo_count];

		for (unsigned i = 0; i < servo_count; i++) {
			servos[i] = pwm_value;
		}

		if (direction == 1) {
			// use ioctl interface for one direction
			for (unsigned i = 0; i < servo_count;	i++) {
				if (ioctl(fd, PWM_SERVO_SET(i), servos[i]) < 0) {
					err(1, "servo %u set failed\n", i);
					return;
				}
			}

		} else {
			// and use write interface for the other direction
			ret = write(fd, servos, sizeof(servos));

			if (ret != (int)sizeof(servos)) {
				err(1, "error writing PWM servo data, wrote %u got %d\n", sizeof(servos), ret);
				return;
			}
		}

		if (direction > 0) {
			if (pwm_value < 2000) {
				pwm_value++;

			} else {
				direction = -1;
			}

		} else {
			if (pwm_value > 1000) {
				pwm_value--;

			} else {
				direction = 1;
			}
		}

		/* readback servo values */
		for (unsigned i = 0; i < servo_count; i++) {
			servo_position_t value;

			if (ioctl(fd, PWM_SERVO_GET(i), (unsigned long)&value)) {
				err(1, "error reading PWM servo %d\n", i);
				return;
			}
			warn("%d\n", value);
			if (value != servos[i]) {
				errx(1, "servo %d readback error, got %u expected %u\n", i, value, servos[i]);
				return;
			}
		}

	}

	close(fd);

	return;
}

void
fake(int argc, char *argv[])
{
	if (argc < 5) {
		errx(1, "fmu fake <roll> <pitch> <yaw> <thrust> (values -100 .. 100)\n");
		return;
	}

	actuator_controls_s ac;

	ac.control[0] = strtol(argv[1], 0, 0) / 100.0f;

	ac.control[1] = strtol(argv[2], 0, 0) / 100.0f;

	ac.control[2] = strtol(argv[3], 0, 0) / 100.0f;

	ac.control[3] = strtol(argv[4], 0, 0) / 100.0f;

	orb_advert_t handle = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &ac);

	if (handle == NULL) {
		errx(1, "advertise failed\n");
		return;
	}

	actuator_armed_s aa;

	aa.armed = true;
	aa.lockdown = false;

	handle = orb_advertise(ORB_ID(actuator_armed), &aa);

	if (handle == NULL) {
		errx(1, "advertise failed 2\n");
		return;
	}

	return;
}

} // namespace

extern "C" int fmu_main(int argc, char *argv[]);

int
fmu_main(int argc, char *argv[])
{
	PortMode new_mode = PORT_MODE_UNSET;
	const char *verb = argv[1];

	if (!strcmp(verb, "stop")) {
		fmu_stop();
		errx(0, "FMU driver stopped\n");
		return 0;
	}

	if (!strcmp(verb, "id")) {
		errx(0,"unsupport get board serial number\n");
		return 0;
	}


	if (fmu_start() != OK) {
		errx(1, "failed to start the FMU driver\n");
		return 0;
	}

	/*
	 * Mode switches.
	 */
	if (!strcmp(verb, "mode_gpio")) {
		errx(0,"unsupport mode_gpio\n");
		return 0;
	} else if (!strcmp(verb, "mode_pwm")) {
		new_mode = PORT_FULL_PWM;
	} else if (!strcmp(verb, "mode_pwm2")) {
		new_mode = PORT_PWM2;
	} else if (!strcmp(verb, "mode_pwm4")) {
		new_mode = PORT_PWM4;
	} else if (!strcmp(verb, "mode_pwm6")) {
		new_mode = PORT_PWM6;
	} else if (!strcmp(verb, "mode_pwm8")) {
		new_mode = PORT_PWM8;
	} else if (!strcmp(verb, "mode_serial")) {
		errx(0,"unsupport mode_serial\n");
		return 0;
	} else if (!strcmp(verb, "mode_gpio_serial")) {
		errx(0,"unsupport mode_gpio_serial\n");
		return 0;
	} else if (!strcmp(verb, "mode_pwm_serial")) {
		errx(0,"unsupport mode_pwm_serial\n");
		return 0;
	} else if (!strcmp(verb, "mode_pwm_gpio")) {
		errx(0,"unsupport mode_pwm_gpio\n");
		return 0;
	}

	/* was a new mode set? */
	if (new_mode != PORT_MODE_UNSET) {

		/* yes but it's the same mode */
		if (new_mode == g_port_mode) {
			return OK;
		}

		/* switch modes */
		fmu_new_mode(new_mode);
	}

	if (!strcmp(verb, "test")) {
		test();
		return 0;
	}

	if (!strcmp(verb, "fake")) {
		fake(argc - 1, argv + 1);
		return 0;
	}


	if (!strcmp(verb, "sensor_reset")) {
		warnx("unsupport sensor_reset\n");
		return 0;
	}

	if (!strcmp(verb, "peripheral_reset")) {
		warnx("unsupport peripheral_reset\n");
		return 0;
	}

	if (!strcmp(verb, "i2c")) {
		warnx("unsupport fmu I2C\n");
		return 0;
	}
	fprintf(stderr, "FMU: unrecognised command %s, try:\n", verb);
	return 0;
}
