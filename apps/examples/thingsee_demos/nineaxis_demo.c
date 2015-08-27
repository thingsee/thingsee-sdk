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

#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nxfonts.h>
#include <arch/board/board-adc.h>
#include <apps/thingsee/ts_core.h>
#include "../../../apps/thingsee/nineaxls/lsm9ds1_module.h"
#include "../../../apps/system/display/oled_display.h"
#include "../../../apps/system/display/oled_text_block.h"

int nineaxis_demo_start(int);
void startAndSetBias(void);
static int nineaxis_demo_timeout(const int, void * const);
static bool nineaxis_demo_deepsleep_hook(void * const priv);
void nineaxis_demo_stop(void);

int16_t bias[9] = { 0 };
float fbias[6];
float gyro_reso, xl_reso, mag_reso;
int nineaxis_demo_sleep_timer=-1;

int nineaxis_demo_start(int sleep) {
	// Disable deep sleep
	ts_core_deepsleep_hook_add(nineaxis_demo_deepsleep_hook, NULL);

	oled_start_module();

	struct oled_tblock_s loc_1 = {.x=0, .y=0, .width=128, .height=22};
	struct oled_tblock_s loc_2 = {.x=0, .y=24, .width=128, .height=10};
	oled_printf(&loc_1, FONTID_SANS17X22, "9-axis display");
	oled_printf(&loc_2, FONTID_MONO5X8, "Starting...");

	startAndSetBias();

	// Setup timer
	nineaxis_demo_sleep_timer=ts_core_timer_setup(TS_TIMER_TYPE_INTERVAL, sleep, nineaxis_demo_timeout, NULL);

	return OK;
}

void nineaxis_demo_stop() {
	// Remove timer
	ts_core_timer_stop(nineaxis_demo_sleep_timer);
	// Remove deep sleep hook
	ts_core_deepsleep_hook_remove(nineaxis_demo_deepsleep_hook);
	nineax_lsm9ds1_stop();
}

static bool nineaxis_demo_deepsleep_hook(void * const priv) {
	return false;
}

static int nineaxis_demo_timeout(const int timer_id, void * const priv) {
	int16_t data[9] = { 0 };
	struct oled_tblock_s loc_2 = {.x=0, .y=24, .width=128, .height=10};
	struct oled_tblock_s loc_3 = {.x=0, .y=34, .width=128, .height=10};
	struct oled_tblock_s loc_4 = {.x=0, .y=44, .width=128, .height=10};

	nineax_lsm9ds1_wait_for_sensor();
	nineax_lsm9ds1_read_all(data);

	//		oled_printf(&loc_1, FONTID_SANS17X22, "Gyro: %.2f, %.2f, %.2f   ", ((float)data[0])/1000, ((float)data[1])/1000, ((float)data[2])/1000);
	//		oled_printf(&loc_2, FONTID_SANS17X22, "Accl: %.2f, %.2f, %.2f   ", ((float)data[3])/1000, ((float)data[4])/1000, ((float)data[5])/1000);
	//		oled_printf(&loc_3, FONTID_SANS17X22, "Magn: %.2f, %.2f, %.2f   ", ((float)data[6])/1000, ((float)data[7])/1000, ((float)data[8])/1000);

	float gx, gy, gz, ax, ay, az, mx, my, mz;
	/* Calculate the gyro value into actual degrees per second, that depend on the sensor scale in use. */
	gx = data[0] * gyro_reso - fbias[0];
	gy = data[1] * gyro_reso - fbias[1];
	gz = data[2] * gyro_reso - fbias[2];

	/* Calculate the acceleration value into actual g's, that depend on the sensor scale in use. */
	ax = data[3] * xl_reso - fbias[3];
	ay = data[4] * xl_reso - fbias[4];
	az = data[5] * xl_reso - fbias[5];

	/* Actual magnetometer value in milli-Gauss. Bias is ignored because it has been set into offset registers in device, which compensates for it automatically. */
	mx = data[6] * mag_reso;
	my = data[7] * mag_reso;
	mz = data[8] * mag_reso;

	oled_printf(&loc_2, FONTID_MONO5X8, "G: %.2f, %.2f, %.2f", gx, gy, gz);
	oled_printf(&loc_3, FONTID_MONO5X8, "A: %.2f, %.2f, %.2f", ax, ay, az);
	oled_printf(&loc_4, FONTID_MONO5X8, "M: %.2f, %.2f, %.2f", mx, my, mz);

	return OK;
}

void startAndSetBias() {
	nineax_lsm9ds1_start();

	/* Get biases from EEPROM, assuming prior calibration has been done. */
	nineax_lsm9ds1_load_bias_from_eeprom(bias);

	nineax_lsm9ds1_config_gyro();
	nineax_lsm9ds1_config_mag();

	nineax_lsm9ds1_read_resolutions(&gyro_reso, &xl_reso, &mag_reso);

	/* Convert biases to floating-point here, not during every sensor update. */
	fbias[0] = gyro_reso * bias[0];
	fbias[1] = gyro_reso * bias[1];
	fbias[2] = gyro_reso * bias[2];
	fbias[3] = xl_reso * bias[3];
	fbias[4] = xl_reso * bias[4];
	fbias[5] = xl_reso * bias[5];
}
