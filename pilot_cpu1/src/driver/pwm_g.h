/**
*******************************************************************************
* @file                  pwm_g.h
* @author                $Author: Frank Wang <wang990099@163.com> $
*
* Base class for devices connected via pwm globle define.
* Copyright 2016 RobSense. All rights reserved.
*******************************************************************************/
#ifndef _DEVICE_PWM_GLOBAL_H
#define _DEVICE_PWM_GLOBAL_H

#include "pwm/drv_pwm_servo.h"
#include "xil_io.h"
#include "xparameters.h"

/*
* The configuration table for devices
*/

#define CHANNEL_RATE_OFFSET       0x0U
#define CHANNEL_PWM_OFFSET        0x4U
#define PL_APB_CLKIN              100000000U
#define SECOND_TO_MICROSECOND     1000000U

#define CHANNEL_1_ADDRESS         XPAR_MYIP_PWM_0_S00_AXI_BASEADDR
#define CHANNEL_2_ADDRESS         XPAR_MYIP_PWM_1_S00_AXI_BASEADDR
#define CHANNEL_3_ADDRESS         XPAR_MYIP_PWM_2_S00_AXI_BASEADDR
#define CHANNEL_4_ADDRESS         XPAR_MYIP_PWM_3_S00_AXI_BASEADDR
#define CHANNEL_5_ADDRESS         XPAR_MYIP_PWM_4_S00_AXI_BASEADDR
#define CHANNEL_6_ADDRESS         XPAR_MYIP_PWM_5_S00_AXI_BASEADDR
#define CHANNEL_7_ADDRESS         XPAR_MYIP_PWM_6_S00_AXI_BASEADDR
#define CHANNEL_8_ADDRESS         XPAR_MYIP_PWM_7_S00_AXI_BASEADDR


const struct pwm_servo_timer pwm_timers[PWM_SERVO_MAX_TIMERS] =
{
	{
		.base = CHANNEL_1_ADDRESS,
		.clock_freq = PL_APB_CLKIN,
	},
	{
		.base = CHANNEL_2_ADDRESS,
		.clock_freq = PL_APB_CLKIN,
	},
	{
		.base = CHANNEL_3_ADDRESS,
		.clock_freq = PL_APB_CLKIN,
	},
	{
		.base = CHANNEL_4_ADDRESS,
		.clock_freq = PL_APB_CLKIN,
	},
	{
		.base = CHANNEL_5_ADDRESS,
		.clock_freq = PL_APB_CLKIN,
	},
	{
		.base = CHANNEL_6_ADDRESS,
		.clock_freq = PL_APB_CLKIN,
	},
	{
		.base = CHANNEL_7_ADDRESS,
		.clock_freq = PL_APB_CLKIN,
	},
	{
		.base = CHANNEL_8_ADDRESS,
		.clock_freq = PL_APB_CLKIN,
	}
};

const struct pwm_servo_channel pwm_channels[PWM_SERVO_MAX_CHANNELS] = {
	{
		.timer_index = 0,
		.channel_id = 1,
		.default_value = 0,
	},
	{
		.timer_index = 1,
		.channel_id = 2,
		.default_value = 0,
	},
	{
		.timer_index = 2,
		.channel_id = 3,
		.default_value = 0,
	},
	{
		.timer_index = 3,
		.channel_id = 4,
		.default_value = 0,
	},
	{
		.timer_index = 4,
		.channel_id = 5,
		.default_value = 0,
	},
	{
		.timer_index = 5,
		.channel_id = 6,
		.default_value = 0,
	},
	{
		.timer_index = 6,
		.channel_id = 7,
		.default_value = 0,
	},
	{
		.timer_index = 7,
		.channel_id = 8,
		.default_value = 0,
	}
};


#define PL_PWM_RATE_SET(Instance, value) \
	Xil_Out32(((u32)(Instance.base) + CHANNEL_RATE_OFFSET), \
		((Instance.clock_freq)/value))

#define PL_PWM_RATE_GET(Instance) \
	Xil_In32((u32)(Instance.base) + CHANNEL_RATE_OFFSET)

#define PL_PWM_SET(Instance, value) \
	Xil_Out32(((u32)(Instance.base) + CHANNEL_PWM_OFFSET), \
		(((Instance.clock_freq)/(SECOND_TO_MICROSECOND))*value))

u32 pl_pwm_get(struct pwm_servo_timer * timersptr) {
	u32 value = 0;
	value = Xil_In32((u32)(timersptr->base) + CHANNEL_PWM_OFFSET);
	value = value / ((timersptr->clock_freq) / SECOND_TO_MICROSECOND);
	return value;
}

#endif /* _DEVICE_PWM_GLOBAL_H */
