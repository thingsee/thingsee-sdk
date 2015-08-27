/****************************************************************************
 * Copyright (C) 2014-2015 Haltian Ltd. All rights reserved.
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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
 * Author: Petri Salonen <petri.salonen@haltian.com>
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <apps/thingsee/ts_core.h>

#include <nuttx/sensors/max44009.h>
#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nxfonts.h>
#include "../../../apps/system/display/oled_display.h"
#include "../../../apps/system/display/oled_text_block.h"

static int als_demo_timeout(const int, void * const);
static bool als_demo_deepsleep_hook(void * const priv);
static float convert_rawals2float(uint16_t);
static float read_als_data(void);
void als_demo_stop(void);

static int als_demo_sleep_timer=-1;

int als_demo_start(int sleep) {
	// Disable deep sleep
	ts_core_deepsleep_hook_add(als_demo_deepsleep_hook, NULL);

	oled_start_module();

	struct oled_tblock_s loc_1 = {.x=0, .y=0, .width=128, .height=22};
	oled_printf(&loc_1, FONTID_SANS17X22, "ALS display");

	// Setup timer
	als_demo_sleep_timer=ts_core_timer_setup(TS_TIMER_TYPE_INTERVAL, sleep, als_demo_timeout, NULL);

	return OK;
}

void als_demo_stop() {
	// Remove timer
	ts_core_timer_stop(als_demo_sleep_timer);
	// Remove deep sleep hook
	ts_core_deepsleep_hook_remove(als_demo_deepsleep_hook);
}

static bool als_demo_deepsleep_hook(void * const priv) {
	return false;
}

static int als_demo_timeout(const int timer_id, void * const priv) {
	struct oled_tblock_s loc_2 = {.x=0, .y=24, .width=128, .height=10};
	float als=read_als_data();
	oled_printf(&loc_2, FONTID_MONO5X8, "ALS = %.1f lux", als);

	return OK;
}

static float read_als_data() {
	float als = 0;
	max44009_data_t data;

	int fd = open("/dev/als0", O_RDWR);
	if (fd>=0) {
		int ret = ioctl(fd, MAX44009_IOC_READ_RAW_DATA, (unsigned int)&data);
		if (ret < 0)
			perror("Cannot read raw data");
		else
			als = convert_rawals2float(data.raw_value);
		close(fd);
	}
	return als;
}

static float convert_rawals2float(uint16_t raw_data) {
	float lux_result = 0;
	const uint16_t mask_raw_data = 0x0FFF;
	const uint16_t mask_first_bit = 0x0001;
	uint16_t exponent_part = 0;
	uint16_t mantissa_part = 0;
	uint16_t i = 0;

	raw_data &= mask_raw_data;
	for (i = 0; i < 12; i++) {
		if (i < 8) {
			mantissa_part += (1 << i) * (mask_first_bit & (raw_data >> i));
		} else {
			exponent_part += ((1 << i) >> 8) * (mask_first_bit & (raw_data >> i));
		}
	}

	lux_result = (1 << exponent_part) * mantissa_part * 0.045f;

	return lux_result;
}

