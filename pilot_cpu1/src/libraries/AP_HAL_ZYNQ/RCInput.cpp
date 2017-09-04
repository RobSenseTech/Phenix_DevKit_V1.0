#include <AP_HAL/AP_HAL.h>

#include "RCInput.h"
//#include <driver/drv_pwm_output.h>       //该头文件已经移植到driver目录下
#include "hrt/drv_hrt.h"			 //
#include "uORB/uORB.h"
#include "device/cdev.h"


using namespace PX4;

extern const AP_HAL::HAL& hal;

void PX4RCInput::init()
{
	_perf_rcin = perf_alloc(PC_ELAPSED, "APM_rcin");
	_rc_sub = orb_subscribe(ORB_ID(input_rc));
	if (_rc_sub == -1) {
		AP_HAL::panic("Unable to subscribe to input_rc");
	}
	clear_overrides();
	rcin_mutex = xSemaphoreCreateMutex();			//创建信号量
}

bool PX4RCInput::new_input() 
{
	xSemaphoreTake(rcin_mutex, portMAX_DELAY);
    bool valid = _rcin.timestamp_last_signal != _last_read || _override_valid;
    _last_read = _rcin.timestamp_last_signal;
    _override_valid = false;
    xSemaphoreGive(rcin_mutex);
    return valid;
}

uint8_t PX4RCInput::num_channels() 
{
	xSemaphoreTake(rcin_mutex, portMAX_DELAY);
    uint8_t n = _rcin.channel_count;
    xSemaphoreGive(rcin_mutex);
    return n;
}

uint16_t PX4RCInput::read(uint8_t ch) 
{
	if (ch >= RC_INPUT_MAX_CHANNELS) {
		return 0;
	}
		xSemaphoreTake(rcin_mutex, portMAX_DELAY);		//用xSemaphoreTake替换掉pthread_mutex_lock
	if (_override[ch]) {
            uint16_t v = _override[ch];
            xSemaphoreGive(rcin_mutex);
            return v;
	}
	if (ch >= _rcin.channel_count) {
			xSemaphoreGive(rcin_mutex);		//用xSemaphoreTake替换掉pthread_mutex_lock
            return 0;
	}
	uint16_t v = _rcin.values[ch];
		xSemaphoreGive(rcin_mutex);
        return v;
}

uint8_t PX4RCInput::read(uint16_t* periods, uint8_t len) 
{
	if (len > RC_INPUT_MAX_CHANNELS) {
		len = RC_INPUT_MAX_CHANNELS;
	}
	for (uint8_t i = 0; i < len; i++){
		periods[i] = read(i);
	}
	return len;
}

bool PX4RCInput::set_overrides(int16_t *overrides, uint8_t len) 
{
	bool res = false;
	for (uint8_t i = 0; i < len; i++) {
		res |= set_override(i, overrides[i]);
	}
	return res;
}

bool PX4RCInput::set_override(uint8_t channel, int16_t override) {
	if (override < 0) {
		return false; /* -1: no change. */
	}
	if (channel >= RC_INPUT_MAX_CHANNELS) {
		return false;
	}
	_override[channel] = override;
	if (override != 0) {
		_override_valid = true;
		return true;
	}
	return false;
}

void PX4RCInput::clear_overrides()
{
	for (uint8_t i = 0; i < RC_INPUT_MAX_CHANNELS; i++) {
		set_override(i, 0);
	}
}

void PX4RCInput::_timer_tick(void)
{
	perf_begin(_perf_rcin);
	bool rc_updated = false;
	if (orb_check(_rc_sub, &rc_updated) == 0 && rc_updated) {
			xSemaphoreTake(rcin_mutex,portMAX_DELAY);		//用xSemaphoreTake替换掉pthread_mutex_lock
			orb_copy(ORB_ID(input_rc), _rc_sub, &_rcin);
   //         pilot_info("values[0]=%d\n", _rcin.values[0]);
			xSemaphoreGive(rcin_mutex);                               //用xSemaphoreGive替换掉pthread_mutex_unlock
	}
//    pilot_info("rc_updated=%d\n", rc_updated);
        // note, we rely on the vehicle code checking new_input() 
        // and a timeout for the last valid input to handle failsafe
	perf_end(_perf_rcin);
}

//改函数是打开px4io板，这里由于没有px4io板，所以注释掉
bool PX4RCInput::rc_bind(int dsmMode)
{
    
    return true;
}
