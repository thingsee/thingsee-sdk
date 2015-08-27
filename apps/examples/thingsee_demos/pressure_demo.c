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
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>

#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nxfonts.h>
#include <arch/board/board-adc.h>
#include <apps/thingsee/ts_core.h>
#include <nuttx/sensors/lps25h.h>
#include "../../../apps/system/display/oled_display.h"
#include "../../../apps/system/display/oled_text_block.h"

int pressure_demo_start(int);
int pressure_init_module(void);
int pressure_read_data(float*, float*);
static int pressure_demo_timeout(const int, void * const);
static bool pressure_demo_deepsleep_hook(void * const priv);
void pressure_demo_stop(void);

#define LPS25H_DEVPATH "/dev/pres0"
int lps25hfd=-1;
int pressure_demo_sleep_timer=-1;

int pressure_demo_start(int sleep) {
	if (pressure_init_module()==OK) {
		pressure_demo_sleep_timer=-1;

		// Disable deep sleep
		ts_core_deepsleep_hook_add(pressure_demo_deepsleep_hook, NULL);

		oled_start_module();

		struct oled_tblock_s loc_1 = {.x=0, .y=0, .width=128, .height=22};
		oled_printf(&loc_1, FONTID_SANS17X22, "Pressure display");

		// Setup timer
		pressure_demo_sleep_timer=ts_core_timer_setup(TS_TIMER_TYPE_INTERVAL, sleep, pressure_demo_timeout, NULL);
		return OK;
	}
	return ERROR;
}

void pressure_demo_stop() {
	// Remove timer
	ts_core_timer_stop(pressure_demo_sleep_timer);
	// Remove deep sleep hook
	ts_core_deepsleep_hook_remove(pressure_demo_deepsleep_hook);
	if (lps25hfd>=0)
		close(lps25hfd);
}

static bool pressure_demo_deepsleep_hook(void * const priv) {
	return false;
}

static int pressure_demo_timeout(const int timer_id, void * const priv) {
	float temp, pressure;
	static float aver=0;
	struct oled_tblock_s loc_2 = {.x=0, .y=24, .width=128, .height=10};
	struct oled_tblock_s loc_3 = {.x=0, .y=34, .width=128, .height=10};
	struct oled_tblock_s loc_4 = {.x=0, .y=44, .width=128, .height=10};

	pressure_read_data(&temp, &pressure);
	oled_printf(&loc_2, FONTID_MONO5X8, "Press = %.2f hPa", pressure);
	if (aver==0)
		aver=pressure;
	else
		aver=((aver*49)+pressure)/50;
	oled_printf(&loc_3, FONTID_MONO5X8, "Aver  = %.3f hPa", aver);
	oled_printf(&loc_4, FONTID_MONO5X8, "Temp = %.2f C", temp);

	return OK;
}


int pressure_init_module() {
	int status=OK;
	lps25hfd = open(LPS25H_DEVPATH, O_RDONLY);
	if (lps25hfd < 0) {
		dbg("Failed to open %s (%d).\n", LPS25H_DEVPATH, get_errno());
		status = ERROR;
	}

	if (status==OK) {
		status = ioctl(lps25hfd, LPS25H_PRES_CONFIG_ON, 0);
		if (status < 0) {
			dbg("Failed to read pressure & temperature.\n");
		}
	}

	if (status==OK) {
//		status = ioctl(lps25hfd, LPS25H_SENSOR_INT_TST, 0);
		if (status < 0) {
			dbg("Failed to read pressure & temperature.\n");
		}
	}

return status;
}

int pressure_read_data(float *temp, float *pressure) {
	int status=OK;
    lps25h_temper_data_t t;
    lps25h_pressure_data_t p;

    status = ioctl(lps25hfd, LPS25H_PRESSURE_OUT, (long)&p);
    if (status < 0) {
        dbg("Failed to read pressure & temperature.\n");
      }

    if (status==OK) {
    status = ioctl(lps25hfd, LPS25H_TEMPERATURE_OUT, (long)&t);
	if (status < 0) {
		dbg("Failed to read pressure & temperature.\n");
		}
    }

    if (status==OK) {
		*temp = ((float)t.int_temper)/1000.0f;
		*pressure = ((float)p.pressure_Pa)/100000.0f;
    }
    return status;
}

