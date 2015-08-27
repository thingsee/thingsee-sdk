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

#include <nuttx/input/cypress_mbr3108.h>

#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nxfonts.h>

void demo_menu(void);
static int demo_menu_callback(const struct pollfd *const, void *const);
int start_demo_app(int);
void stop_demo_app(int);

int als_demo_start(int); void als_demo_stop(void);
int pressure_demo_start(int); void pressure_demo_stop(void);
int humidity_demo_start(int); void humidity_demo_stop(void);
int accel_demo_start(int); void accel_demo_stop(void);
int nineaxis_demo_start(int); void nineaxis_demo_stop(void);

int (*f_start_table[])(int)={ als_demo_start, pressure_demo_start, humidity_demo_start, accel_demo_start, nineaxis_demo_start};
void (*f_stop_table[])(void)={als_demo_stop,  pressure_demo_stop,  humidity_demo_stop,  accel_demo_stop,  nineaxis_demo_stop};

void demo_menu(void) {
	int demo_menu_fd;

	demo_menu_fd = open("/dev/capsense0", O_RDWR);
	if (demo_menu_fd < 0) {
		dbg("Cannot open capasitive buttons for ThingSee demo.\n");
		// Fall back to main loop
		return;
	}

	ts_core_fd_register(demo_menu_fd, POLLIN, demo_menu_callback, NULL);
	// Start first app
	start_demo_app(0);
}

int start_demo_app(int index) {
	static int refresh_time=500;
	return (*f_start_table[index]) (refresh_time);
}

void stop_demo_app(int index) {
	(*f_stop_table[index])();
}

static int demo_menu_callback(const struct pollfd *const pfd, void *const priv) {
	static bool bChanging=false;
	static int demo_index=0;
	static int prev_button=0;  // Store for button
	int demo_table_entries=sizeof(f_start_table) / sizeof(f_start_table[0]);

	if (bChanging)  // Busy starting/stopping previous app -> ignore
		return ERROR;

	bChanging=true;
	struct mbr3108_sensor_status_s data;
	read(pfd->fd, &data, sizeof(data));

	if (data.button==0) {
		if (prev_button==1) { // Back
			stop_demo_app(demo_index);
			if (--demo_index<0)
				demo_index=demo_table_entries-1;
			// Start new app
			start_demo_app(demo_index);
		}
		else if (prev_button==2) { // Forward
			// Stop old app
			stop_demo_app(demo_index);
			if (++demo_index>=demo_table_entries)
				demo_index=0;
			// Start new app
			start_demo_app(demo_index);
		}
	}
	prev_button=data.button;
	bChanging=false;
	return OK;
}

