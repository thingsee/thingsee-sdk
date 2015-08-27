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

#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nxfonts.h>
#include <arch/board/board-adc.h>
#include <apps/thingsee/ts_core.h>
#include <apps/thingsee/modules/ts_accel.h>
#include "../../../apps/system/display/oled_display.h"
#include "../../../apps/system/display/oled_text_block.h"

void accel_demo_callback(void const * const, void * const);
static bool accel_demo_deepsleep_hook(void * const);
int accel_demo_start(int);
void accel_demo_stop(void);

int accel_demo_start(int sleep) {
	// Disable deep sleep
	ts_core_deepsleep_hook_add(accel_demo_deepsleep_hook, NULL);

	oled_start_module();

	struct oled_tblock_s loc_1 = {.x=0, .y=00, .width=128, .height=22};
	struct oled_tblock_s loc_2 = {.x=0, .y=24, .width=128, .height=10};
	struct oled_tblock_s loc_3 = {.x=0, .y=34, .width=128, .height=10};
	oled_printf(&loc_1, FONTID_SANS17X22, "Acceleration display");
	oled_printf(&loc_2, FONTID_MONO5X8, "Tilt = -");
	oled_printf(&loc_3, FONTID_MONO5X8, "Impact = -");

	ts_accel_callback_register(ACCEL_EVENT_IMPACT | ACCEL_EVENT_TILT, accel_demo_callback, NULL);

	return OK;
}

void accel_demo_stop() {
	ts_accel_callback_unregister(accel_demo_callback);
	// Remove deep sleep hook
	ts_core_deepsleep_hook_remove(accel_demo_deepsleep_hook);
}

static bool accel_demo_deepsleep_hook(void * const priv) {
	return false;
}

void accel_demo_callback(void const * const e, void * const priv) {
	struct accel_event_s const * const event = e;
	struct oled_tblock_s loc_2 = {.x=0, .y=24, .width=128, .height=10};
	struct oled_tblock_s loc_3 = {.x=0, .y=34, .width=128, .height=10};

	switch (event->id) {
		default:
			return;

		case ACCEL_EVENT_TILT: {
			struct accel_event_tilt const * const tilt_e = e;
			oled_printf(&loc_2, FONTID_MONO5X8, "Tilt = %d", tilt_e->tilt_value);
			}
			break;

		case ACCEL_EVENT_IMPACT: {
			struct accel_event_impact const * const imp_e = e;
			oled_printf(&loc_3, FONTID_MONO5X8, "Impact = %.2f g", ((float)imp_e->impact_value)/1000.0f);
			}
			break;
	}
}
