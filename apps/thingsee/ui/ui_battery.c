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

#include <arch/board/board-battery.h>
#include <nuttx/sensors/lps25h.h>

#include <ui_battery.h>

static int UI_get_temperature(void);
static int UI_get_battery_voltage(float *voltage);
static int UI_pressure_init_module(int *fd);
static int UI_read_temperature_data(int fd, int *temp);

/* Pressure sensor device */
#define LPS25H_DEVPATH "/dev/pres0"

uint32_t UI_get_battery_percentage(void) {
    int temperature, ret;
    float voltage = 0.0f;

    temperature = UI_get_temperature();
    ret = UI_get_battery_voltage(&voltage);
    if (ret < 0)
      dbg("Opening of the VBAT ADC failed in UI.\n");

    return board_get_battery_level(voltage, temperature);
}

static int UI_get_battery_voltage(float *voltage)
{
  uint32_t val;
  int ret;

  ret = board_get_battery_average_voltage(&val);
  if (ret < 0)
    {
      return ret;
    }

  *voltage = (float) val / 1000.0;

  return OK;
}

static int UI_get_temperature(void) {
    int temperature = 0; /* Set default to 0 degrees. */
    int lps25hfd = -1;

    if (UI_pressure_init_module(&lps25hfd) == OK) {
        UI_read_temperature_data(lps25hfd, &temperature);
        if (lps25hfd >= 0)
            close(lps25hfd);
        else
            dbg("Opening of the temperature sensor failed in UI.\n");
    }
    return temperature;
}

static int UI_pressure_init_module(int *fd) {
    int status = OK;
    *fd = open(LPS25H_DEVPATH, O_RDONLY);
    if (*fd < 0) {
        dbg("Failed to open %s (%d).\n", LPS25H_DEVPATH, get_errno());
        status = ERROR;
    }

    if (status == OK) {
        status = ioctl(*fd, LPS25H_PRES_CONFIG_ON, 0);
        if (status < 0) {
            dbg("Failed to read pressure & temperature.\n");
        }
    }

    return status;
}

static int UI_read_temperature_data(int fd, int *temp) {
    int status = OK;
    lps25h_temper_data_t t;
    lps25h_pressure_data_t pt;

    status = ioctl(fd, LPS25H_PRESSURE_OUT, (long)&pt);
    if (status < 0) {
        dbg("Failed to read pressure. Temperature reading might be bogus.\n");
    }

    status = ioctl(fd, LPS25H_TEMPERATURE_OUT, (long)&t);
    if (status < 0) {
        dbg("Failed to read temperature.\n");
    }

    if (status == OK) {
        *temp = t.int_temper / LPS25H_TEMPER_DIVIDER;
    }
    return status;
}

