/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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

/*
 * @file drv_pwm_servo.c
 *
 * Servo driver supporting PWM servos connected to STM32 timer blocks.
 *
 * Works with any of the 'generic' or 'advanced' STM32 timers that
 * have output pins, does not require an interrupt.
 */

#include <sys/types.h>
#include <stdbool.h>

#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <drv_pwm_output.h>
#include "drv_pwm_servo.h"
#include "driver_define.h"
#include "pwm_g.h"
#include "FreeRTOS_Print.h"

static void		pwm_timer_init(unsigned timer);
static void		pwm_timer_set_rate(unsigned timer, unsigned rate);
static void		pwm_channel_init(unsigned channel);

static void
pwm_timer_init(unsigned timer)
{
	/* default to updating at 50Hz */
	pwm_timer_set_rate(timer, 50);
}

static void
pwm_timer_set_rate(unsigned timer, unsigned rate)
{
	PL_PWM_RATE_SET(pwm_timers[timer], rate);
}

static void
pwm_channel_init(unsigned channel)
{
	unsigned timer = pwm_channels[channel].timer_index;

	/* configure the channel */
	PL_PWM_SET(pwm_timers[timer], pwm_channels[channel].default_value);
}

int
up_pwm_servo_set(unsigned channel, servo_position_t value)
{
	if (channel >= PWM_SERVO_MAX_CHANNELS) {
		return -1;
	}

	unsigned timer = pwm_channels[channel].timer_index;

	/* test timer for validity */
	if (pwm_timers[timer].base == 0) {
		return -1;
	}
	PL_PWM_SET(pwm_timers[timer], value);

	return 0;
}

servo_position_t
up_pwm_servo_get(unsigned channel)
{
	if (channel >= PWM_SERVO_MAX_CHANNELS) {
		return 0;
	}
	unsigned timer = pwm_channels[channel].timer_index;
	servo_position_t value = 0;

	/* test timer for validity */
	if ((pwm_timers[timer].base == 0) ||
	    (pwm_channels[channel].channel_id == 0)) {
		return 0;
	}

	/* configure the channel */
	value = pl_pwm_get(&(pwm_timers[timer]));

	return value;
}

int
up_pwm_servo_init(uint32_t channel_mask)
{
	/* there are something wrong here, it should init PWM PIN firstly before set frequency */
	{
		Xil_Out32((CHANNEL_1_ADDRESS + CHANNEL_PWM_OFFSET), 10);
		Xil_Out32((CHANNEL_2_ADDRESS + CHANNEL_PWM_OFFSET), 10);
		Xil_Out32((CHANNEL_3_ADDRESS + CHANNEL_PWM_OFFSET), 10);
		Xil_Out32((CHANNEL_4_ADDRESS + CHANNEL_PWM_OFFSET), 10);
		Xil_Out32((CHANNEL_5_ADDRESS + CHANNEL_PWM_OFFSET), 10);
		Xil_Out32((CHANNEL_6_ADDRESS + CHANNEL_PWM_OFFSET), 10);
		Xil_Out32((CHANNEL_7_ADDRESS + CHANNEL_PWM_OFFSET), 10);
		Xil_Out32((CHANNEL_8_ADDRESS + CHANNEL_PWM_OFFSET), 10);
	}

	/* do basic timer initialisation first */
	unsigned i;
	for (i = 0; i < PWM_SERVO_MAX_TIMERS; i++) {
		if (pwm_timers[i].base != 0) {
			pwm_timer_init(i);
		}
	}

	/* now init channels */
	for (i = 0; i < PWM_SERVO_MAX_CHANNELS; i++) {
		/* don't do init for disabled channels; this leaves the pin configs alone */
		if (((1 << i) & channel_mask) && (pwm_channels[i].channel_id != 0)) {
			pwm_channel_init(i);
		}
	}

	return OK;
}

void
up_pwm_servo_deinit(void)
{
	/* disable the timers */
	up_pwm_servo_arm(false);
}

int
up_pwm_servo_set_rate_group_update(unsigned group, unsigned rate)
{
	/* limit update rate to 1..10000Hz; somewhat arbitrary but safe */
	if (rate < 1) {
		return -ERANGE;
	}

	if (rate > 10000) {
		return -ERANGE;
	}

	if ((group >= PWM_SERVO_MAX_TIMERS) || (pwm_timers[group].base == 0)) {
		return ERROR;
	}

	pwm_timer_set_rate(group, rate);

	return OK;
}

int
up_pwm_servo_set_rate(unsigned rate)
{
	unsigned i;
	for (i = 0; i < PWM_SERVO_MAX_TIMERS; i++) {
		up_pwm_servo_set_rate_group_update(i, rate);
	}

	return 0;
}

uint32_t
up_pwm_servo_get_rate_group(unsigned group)
{
	unsigned channels = 0;
	unsigned i;
	for (i = 0; i < PWM_SERVO_MAX_CHANNELS; i++) {
		if (pwm_channels[i].timer_index == group) {
			channels |= (1 << i);
		}
	}

	return channels;
}

void
up_pwm_servo_arm(bool armed)
{
	unsigned i;
	/* iterate timers and arm/disarm appropriately */
	for (i = 0; i < PWM_SERVO_MAX_TIMERS; i++) {
		if (pwm_timers[i].base != 0) {
			Print_Info("up_pwm_servo_arm timer[%d] arm=%d\n", i, armed);
			if (armed) {
				// nothing to do;
			} else {
				PL_PWM_SET(pwm_timers[i], 1);
			}
		}
	}
}
