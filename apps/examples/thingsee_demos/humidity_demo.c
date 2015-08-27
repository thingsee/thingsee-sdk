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
#include <stdbool.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nxfonts.h>
#include <arch/board/board-adc.h>
#include <apps/thingsee/ts_core.h>
#include <nuttx/sensors/hts221.h>
#include "../../../apps/system/display/oled_display.h"
#include "../../../apps/system/display/oled_text_block.h"

int humidity_init_module(void);
int humidity_read_data(float*, float*);
int humidity_read_data_raw(hts221_conv_data_t *);
void humidity_close_module(void);
static int humidity_demo_timeout(const int, void * const);
static bool humidity_demo_deepsleep_hook(void * const priv);
void humidity_demo_stop(void);

int hts221fd;
int humidity_demo_sleep_timer=-1;

void humidity_demo_start(int sleep) {
	if (humidity_init_module()==OK) {
		humidity_demo_sleep_timer=-1;

		// Disable deep sleep
		ts_core_deepsleep_hook_add(humidity_demo_deepsleep_hook, NULL);

		oled_start_module();

		struct oled_tblock_s loc_1 = {.x=0, .y=0, .width=128, .height=22};

		oled_printf(&loc_1, FONTID_SANS17X22, "Humidity display");
		// Setup timer
		humidity_demo_sleep_timer=ts_core_timer_setup(TS_TIMER_TYPE_INTERVAL, sleep, humidity_demo_timeout, NULL);
	}
}

void humidity_demo_stop() {
	// Remove timer
	ts_core_timer_stop(humidity_demo_sleep_timer);
	// Remove deep sleep hook
	ts_core_deepsleep_hook_remove(humidity_demo_deepsleep_hook);

	humidity_close_module();
}

static bool humidity_demo_deepsleep_hook(void * const priv) {
	return false;
}

static int humidity_demo_timeout(const int timer_id, void * const priv) {
	float temp, humidity;
	struct oled_tblock_s loc_2 = {.x=0, .y=24, .width=128, .height=10};
	struct oled_tblock_s loc_3 = {.x=0, .y=34, .width=128, .height=10};

	if (humidity_read_data(&temp, &humidity) == OK) {
		oled_printf(&loc_2, FONTID_MONO5X8, "Humidity = %.2f%%", humidity);
		oled_printf(&loc_3, FONTID_MONO5X8, "Temp = %.2f C", temp);
	}

	return OK;
}


int humidity_init_module() {
	int status = OK;
	hts221_settings_t settings = {
		.temp_resol = HTS221_AVGT4,
		.humid_resol = HTS221_AVGH8,
		.odr = HTS221_ODR_7HZ,
		.is_bdu = true,
		.is_data_rdy = true,
		.is_high_edge = true,
		.is_open_drain = false,
		.is_boot = false
		};
	struct hts221_status_t hwstatus;
	struct hts221_raw_data_t raw_data;


	 /* Open humidity sensor */
	hts221fd = open("/dev/hts221", O_RDONLY);
	if (hts221fd < 0) {
		dbg("Failed to open /dev/hts221 (%d).\n", get_errno());
		status = ERROR;
	}

	if (status==OK) {
		/* Configure */
		status = ioctl(hts221fd, HTS221_IOC_CFGR, (unsigned int)&settings);
		if (status < 0) {
			dbg("Failed to read humidity & temperature.\n");
		}
	}
	if (status==OK) {
		/* Clear data from device */
		status = ioctl(hts221fd, HTS221_IOC_CHECK_STATUS_REG, (unsigned int)&hwstatus);
		if (status < 0) {
			dbg("Failed to read humidity & temperature.\n");
		}
	}

	if (status==OK) {
		if (hwstatus.is_humid_ready && hwstatus.is_temp_ready) {
		status = ioctl(hts221fd, HTS221_IOC_READ_RAW_DATA, (unsigned int)&raw_data);
		if (status < 0) {
			dbg("Failed to read humidity & temperature.\n");
			}
		}
	}

	if (status==OK) {
		/* Start conversion */
		status = ioctl(hts221fd, HTS221_IOC_START_CONVERSION, 0);
		if (status < 0) {
			dbg("Failed to read humidity & temperature.\n");
			}
	}

	return status;
}

int humidity_read_data(float *temp, float *humidity) {
	struct hts221_conv_data_t values;

	int retval = humidity_read_data_raw(&values);
	if (retval == OK) {
		*temp = ((float)values.temperature)/100.0f;
		*humidity = ((float)values.humidity)/10.0f;
	}

	return retval;
}

int humidity_read_data_raw(hts221_conv_data_t *values) {
	struct pollfd pfd = {
		.fd = hts221fd,
		.events = POLLIN
	};
	struct hts221_status_t hwstatus;

	int ret = -1;
	int loop = 0;
	while (ret<=0 && loop++ < 20)
		ret = poll(&pfd, 1, 100);
	if (loop==20)
		return -2;

//	poll(&pfd, 1, -1);

	int status = ioctl(hts221fd, HTS221_IOC_CHECK_STATUS_REG, (unsigned int)&hwstatus);
	if (status < 0) {
		dbg("Failed to read humidity & temperature.\n");
		return -1;
	}
	if (!hwstatus.is_humid_ready && !hwstatus.is_temp_ready) {
		/* Not ready yet. */
		return -2;
	}

	status = ioctl(hts221fd, HTS221_IOC_READ_CONVERT_DATA, (unsigned int)values);
	if (status < 0) {
		dbg("Failed to read humidity & temperature.\n");
		return -1;
	}

	return OK;
}

void humidity_close_module() {
	close(hts221fd);
	hts221fd = -1;
	}
