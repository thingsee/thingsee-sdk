/****************************************************************************
 * apps/ts_engine/engine/sense_group_acceleration.c
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

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <debug.h>
#include <string.h>
#include <math.h>

#include <apps/thingsee/ts_core.h>

#include <nuttx/sensors/max44009.h>

#include "eng_dbg.h"
#include "parse.h"
#include "util.h"
#include "execute.h"

#define ALS_DEV "/dev/als0"

#define THRESHOLD_MAX ((0xf0 << 0xe) * 0.045f)
#define THRESHOLD_MIN 0

struct pow
{
  uint32_t mantissa;
  uint8_t exponent;
};

struct als
{
  int fd;
  double bias;
  int refcount;
};

static struct als g_als = { .fd = -1, .bias = 0.0, .refcount = 0 };

static int open_dev(struct als *als)
{
  if (als->refcount == 0)
    {
      DEBUGASSERT(als->fd >= -1);

      als->fd = open(ALS_DEV, O_RDWR);
      if (als->fd < 0)
        {
          eng_dbg("open %s failed\n", ALS_DEV);
          return als->fd;
        }
    }

  als->refcount++;
  return als->fd;
}

static int close_dev(struct als *als)
{
  int ret = OK;

  if (als->fd < 0)
    {
      /* open failed. */

      return -1;
    }

  DEBUGASSERT(als->refcount > 0);

  als->refcount--;
  if (als->refcount == 0)
    {
      ret = close(als->fd);
      if (ret < 0)
        {
          eng_dbg("close %d failed\n", als->fd);
        }
      als->fd = -1;
    }

  return ret;
}

static void force_close_dev(struct als *als)
{
  if (als->fd < 0)
    {
      /* open failed. */

      return;
    }

  DEBUGASSERT(als->refcount > 0);

  /* See comment in caller. */
  /* close(als->fd); */
  als->fd = -1;
  als->refcount = 0;
}

static double als_convert_raw2double(uint16_t raw_data)
{
  struct pow value;

  value.mantissa = raw_data & 0x00ff;
  value.exponent = (raw_data & 0x0f00) >> 8;

  return (double) (value.mantissa << value.exponent) * 0.045;
}

static void als_convert_double2pow(double value, struct pow *pow)
{
  pow->mantissa = (uint32_t) ceil((value / 0.045f));
  pow->exponent = 0;

  while (pow->mantissa > 0xff)
    {
      pow->mantissa >>= 1;
      pow->exponent++;
    }

  eng_dbg("mantissa: %u exponent: %d\n", pow->mantissa, (int)pow->exponent);
}

static int als_read_raw_data(int fd, uint16_t *raw_data)
{
  int ret;
  max44009_data_t data;

  ret = ioctl(fd, MAX44009_IOC_READ_RAW_DATA, (unsigned int) &data);
  if (ret < 0)
    {
      eng_dbg("MAX44009_IOC_READ_RAW_DATA failed\n");
      return ERROR;
    }

  *raw_data = data.raw_value;

  return OK;
}

static int als_read_data(int fd, double *data)
{
  int ret;
  uint16_t raw_data;

  ret = als_read_raw_data(fd, &raw_data);
  if (ret < 0)
    return ret;

  *data = als_convert_raw2double(raw_data);

  /* Those factors are valid for device with mechanics */

  if (*data < 50)
    {
	  *data *= 22;
    }
  else if (*data < 200)
    {
	  *data *= 15;
    }
  else
    {
	  *data *= 10;
    }

  return ret;
}

static int als_set_thresholds(int fd, double low, double high, bool relative)
{
  int ret;
  struct pow lowpow;
  struct pow highpow;
  max44009_init_ops_t init_ops;

  if (relative)
    {
      ret = als_read_data(fd, &g_als.bias);
      if (ret < 0)
        {
          eng_dbg("als_read_data failed\n");
          return ERROR;
        }

      if (low != THRESHOLD_MAX)
        {
          low += g_als.bias;

        }

      if (high != THRESHOLD_MIN)
        {
          high += g_als.bias;
        }
    }

  if (low < THRESHOLD_MIN)
    {
      low = THRESHOLD_MIN;
    }

  if (high > THRESHOLD_MAX)
    {
      high = THRESHOLD_MAX;
    }

  als_convert_double2pow(high, &highpow);
  als_convert_double2pow(low, &lowpow);

  init_ops.lower_threshold = (uint8_t) ((lowpow.exponent << 4)
      | (lowpow.mantissa >> 4));

  init_ops.upper_threshold = (uint8_t) ((highpow.exponent << 4)
      | (highpow.mantissa >> 4));

  eng_dbg("upper: 0x%04x\n", init_ops.upper_threshold);
  eng_dbg("lower: 0x%04x\n", init_ops.lower_threshold);

  ret = ioctl(fd, MAX44009_IOC_DO_SELFCALIB, (unsigned int) &init_ops);
  if (ret < 0)
    {
      eng_dbg("MAX44009_IOC_DO_SELFCALIB failed\n");
      return ERROR;
    }

  return OK;
}

static int als_read_int_sts(int fd)
{
  int ret = OK;
  max44009_data_t data;

  ret = ioctl(fd, MAX44009_IOC_READ_INTERRUPT_STATUS, (unsigned int) &data);
  if (ret < 0)
    {
      eng_dbg("MAX44009_IOC_READ_INTERRUPT_STATUS failed\n");
      return ERROR;
    }

  return OK;
}

static int als_threshold_callback(const struct pollfd * const pfd,
    void * const priv)
{
  struct ts_cause *cause = priv;
  int ret;

  ret = als_read_data(cause->dyn.fd, &cause->dyn.sense_value.value.valuedouble);
  if (ret < 0)
    {
      eng_dbg("als_read_data failed\n");
      return ERROR;
    }

  ret = als_read_int_sts(cause->dyn.fd);
  if (ret < 0)
    {
      eng_dbg("als_read_int_sts failed\n");
      return ERROR;
    }

  if (cause->conf.threshold.relative)
    {
      cause->dyn.sense_value.value.valuedouble -= g_als.bias;
    }

  handle_cause(cause);

  return OK;
}

static int sense_ambient_light_init(int fd)
{
  int ret;
  const max44009_init_ops_t init_ops =
    {
    .is_cont = false, .is_manual = false, .is_cdr = false, .integr_time =
        MAX44009_INTEGR_TIME_100, .threshold_timer = 0
    };

  ret = ioctl(fd, MAX44009_IOC_INIT, (unsigned int) &init_ops);
  if (ret < 0)
    {
      eng_dbg("MAX44009_IOC_INIT failed\n");
      return ERROR;
    }

  ret = als_read_int_sts(fd);
  if (ret < 0)
    {
      eng_dbg("als_read_int_sts failed\n");
      return ERROR;
    }

  return OK;
}

int sense_ambient_light_irq_init(struct ts_cause *cause)
{
  int ret;
  double thr_isGt = THRESHOLD_MAX;
  double thr_isLt = THRESHOLD_MIN;
  bool found_gt, found_lt;

  found_gt = __ts_engine_find_threshold(cause, isGt, &thr_isGt, NULL);
  found_lt = __ts_engine_find_threshold(cause, isLt, &thr_isLt, NULL);

  if (!found_gt && !found_lt)
    {
      bool found_any;
      bool thr_any = false;

      found_any = __ts_engine_find_threshold(cause, isAny, NULL, &thr_any);
      if (found_any && thr_any)
        {
          /* Select full range for isAny, note that it is still possible
           * to negate this if profile is stupid enough to do so.
           */

          thr_isLt = THRESHOLD_MAX;
          thr_isGt = THRESHOLD_MIN;
        }
      else
        {
          eng_dbg("no thresholds set, aborting setup\n");
          return ERROR;
        }
    }

  if (cause->conf.threshold.negate)
    {
      double temp;

      temp = thr_isLt;
      thr_isLt = thr_isGt;
      thr_isGt = temp;
    }

  cause->dyn.fd = open_dev(&g_als);
  if (cause->dyn.fd < 0)
    {
      eng_dbg("open_dev failed\n");
      return ERROR;
    }

  ret = als_set_thresholds(cause->dyn.fd, thr_isLt, thr_isGt,
      cause->conf.threshold.relative);
  if (ret < 0)
    {
      eng_dbg("als_set_thresholds failed\n");
      goto errout;
    }

  ret = sense_ambient_light_init(cause->dyn.fd);
  if (ret < 0)
    {
      eng_dbg("sense_ambient_light_init failed\n");
      goto errout;
    }

  ret = ts_core_fd_register(cause->dyn.fd, POLLIN, als_threshold_callback,
                            cause);
  DEBUGASSERT(ret == OK);

  return OK;

errout:
  close_dev(&g_als);
  return ERROR;
}

int sense_ambient_light_irq_uninit(struct ts_cause *cause)
{
  /* uninit_cause() has already closed the device fd,
     call this to reset the refcount in our layer of code.

     TODO: Fix by improving engine architecture instead.
   */

  force_close_dev(&g_als);
  return OK;
}

int sense_ambient_light_active_init(struct ts_cause *cause)
{
  int ret;

  g_als.fd = open_dev(&g_als);
  if (g_als.fd < 0)
    {
      eng_dbg("open_dev failed\n");
      return ERROR;
    }

  ret = sense_ambient_light_init(g_als.fd);
  if (ret < 0)
    {
      eng_dbg("sense_ambient_light_init failed\n");
      close_dev(&g_als);
      return ERROR;
    }

  return OK;
}

int sense_ambient_light_active_uninit(struct ts_cause *cause)
{
  close_dev(&g_als);
  return OK;
}

int sense_ambient_light_active_read(struct ts_cause *cause)
{
  int ret;

  if (g_als.fd < 0)
    {
      /* Init had failed, attempt to reopen. */

      ret = sense_ambient_light_active_init(cause);
      if (ret < 0)
        {
          return ERROR;
        }
    }

  ret = als_read_data(g_als.fd, &cause->dyn.sense_value.value.valuedouble);
  if (ret < 0)
    {
      eng_dbg("als_read_data failed\n");
      return ERROR;
    }

  handle_cause_event(cause, NULL);

  return OK;
}
