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
#include <apps/thingsee/ts_core.h>

#include <nuttx/input/cypress_mbr3108.h>
#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nxfonts.h>
#include "../../../apps/system/display/oled_display.h"
#include "../../../apps/system/display/oled_text_block.h"

void capsense_demo_start(void);
void capsense_demo_stop(void);
static int capsense_demo_callback(const struct pollfd *const, void *const);

int capsense_demo_fd=-1;

void capsense_demo_start() {
	oled_start_module();

	struct oled_tblock_s loc_1 = {.x=0, .y=0, .width=128, .height=22};
	struct oled_tblock_s loc_2 = {.x=0, .y=24, .width=128, .height=10};

	oled_printf(&loc_1, FONTID_SANS17X22, "Capsense display");

	capsense_demo_fd = open("/dev/capsense0", O_RDWR);
	if (capsense_demo_fd >= 0) {
		ts_core_fd_register(capsense_demo_fd, POLLIN, capsense_demo_callback, NULL);
		oled_printf(&loc_2, FONTID_MONO5X8, "Buttons: - -");
	}
}

void capsense_demo_stop() {
	ts_core_fd_unregister(capsense_demo_fd);
	close(capsense_demo_fd);
}

static int capsense_demo_callback(const struct pollfd *const pfd, void *const priv) {
	struct mbr3108_sensor_status_s data;
	struct oled_tblock_s loc_2 = {.x=0, .y=24, .width=128, .height=10};

	read(pfd->fd, &data, sizeof(data));

	switch (data.proximity) {
	default:
		oled_printf(&loc_2, FONTID_MONO5X8, "Buttons: - -");
		break;

	case 1:
		oled_printf(&loc_2, FONTID_MONO5X8, "Buttons: 1 -", data.proximity);
		break;

	case 2:
		oled_printf(&loc_2, FONTID_MONO5X8, "Buttons: - 2", data.proximity);
		break;

	case 3:
		oled_printf(&loc_2, FONTID_MONO5X8, "Buttons: 1 2", data.proximity);
	}
	return OK;
}
