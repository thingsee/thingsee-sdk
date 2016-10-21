/****************************************************************************
 * apps/ts_engine/engine/sense_lps25h.c
 *
 * Copyright (C) 2015-2016 Haltian Ltd. All rights reserved.
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
 * Authors:
 *   Pekka Ervasti <pekka.ervasti@haltian.com>
 *
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

#include <stdbool.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <nuttx/sensors/lps25h.h>

#include "eng_dbg.h"
#include "parse.h"
#include "execute.h"

#define LPS25H_DEVPATH "/dev/pres0"

int sense_lps25h_active_read_pressure(struct ts_cause *cause)
{
  lps25h_pressure_data_t p;
  int fd;
  int ret;

  fd = open(LPS25H_DEVPATH, O_RDONLY);
  if (fd < 0)
    {
      eng_dbg("Failed to open %s (%d)\n", LPS25H_DEVPATH, get_errno());
      return ERROR;
    }

  ret = ioctl(fd, LPS25H_PRES_CONFIG_ON, 0);
  if (ret < 0)
    {
      eng_dbg("LPS25H_PRES_CONFIG_ON failed\n");
      goto errout_close;
    }

  ret = ioctl(fd, LPS25H_PRESSURE_OUT, (long) &p);
  if (ret < 0)
    {
      eng_dbg("LPS25H_PRESSURE_OUT failed\n");
      goto errout_close;
    }

  cause->dyn.sense_value.value.valuedouble = (double) p.pressure_Pa
      / (double) 100000;

  handle_cause(cause);

  close(fd);
  return OK;

  errout_close:

  close(fd);
  return ERROR;
}

int sense_lps25h_active_read_temperature(struct ts_cause *cause)
{
  lps25h_temper_data_t t;
  lps25h_pressure_data_t pt;
  int fd;
  int ret;

  fd = open(LPS25H_DEVPATH, O_RDONLY);
  if (fd < 0)
    {
      eng_dbg("Failed to open %s (%d)\n", LPS25H_DEVPATH, get_errno());
      return ERROR;
    }

  ret = ioctl(fd, LPS25H_PRES_CONFIG_ON, 0);
  if (ret < 0)
    {
      eng_dbg("LPS25H_PRES_CONFIG_ON failed\n");
      goto errout_close;
    }

  ret = ioctl(fd, LPS25H_PRESSURE_OUT, (long)&pt);
  if (ret < 0)
    {
      dbg("Failed to read pressure. Temperature reading might be bogus.\n");
    }

  ret = ioctl(fd, LPS25H_TEMPERATURE_OUT, (long) &t);
  if (ret < 0)
    {
      eng_dbg("LPS25H_TEMPERATURE_OUT failed\n");
      goto errout_close;
    }

  cause->dyn.sense_value.value.valuedouble = (double) t.int_temper
      / LPS25H_TEMPER_DIVIDER;

  handle_cause(cause);

  close(fd);
  return OK;

  errout_close:

  close(fd);
  return ERROR;
}
