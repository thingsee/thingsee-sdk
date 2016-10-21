/****************************************************************************
 * modules/accelerometer.c
 *
 * Copyright (C) 2015-2016 Haltian Ltd.
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
 *   Jussi Kivilinna <jussi.kivilinna@haltian.com>
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <errno.h>
#include <math.h>
#include <time.h>
#include <fixedmath.h>
#include <stdint.h>
#include <debug.h>

#include <nuttx/sensors/lis2dh.h>
#include <apps/thingsee/ts_core.h>

#include "example_app_dbg.h"
#include "example_app_accelerometer.h"

#define LIS2DH_DEVICE      "/dev/acc0"
#define RESULT_READ_COUNT  1

#define LIS2DH_INT_SRC_ZH  0x20
#define LIS2DH_INT_SRC_ZL  0x10
#define LIS2DH_INT_SRC_YH  0x08
#define LIS2DH_INT_SRC_YL  0x04
#define LIS2DH_INT_SRC_XH  0x02
#define LIS2DH_INT_SRC_XL  0x01

#define MIN(x, y) ((x) < (y) ? (x) : (y))
#define MAX(x, y) ((x) > (y) ? (x) : (y))

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

/* How long to keep accelerometer powered off between impacts?
 *
 * Tuning this allows power-save optimization when using accelerometer to
 * wake-up system when movement is detected but avoid excess wake-ups when
 * in constant movement (pocket, car, etc).
 */

#define CONFIG_EXAMPLE_APP_DEFAULT_ACCEL_RESTART_TIME     0 /* seconds */

#define CONFIG_EXAMPLE_APP_DEFAULT_ACCEL_IMPACT_THRESHOLD 0.4 /* g */

struct accel_s
{
  int fd;
  int restart_timerid;
  struct lis2dh_setup *setup;
  const struct accelerometer_config *config;
  struct
  {
    accelerometer_callback_t callback;
    void *priv;
  } client;
};

/* Default configuration for accelerometer sensor */

static const struct lis2dh_setup g_default_accel_setup =
{
  .data_rate = LIS2DH_ODR_10HZ,
  .low_power_mode_enable = 0,
  .temp_enable = false,
  .bdu = ST_LIS2DH_CR4_BDU_UPD_ON_READ,
  .fullscale = ST_LIS2DH_CR4_FULL_SCALE_4G,
  .int1_aoi_enable = ST_LIS2DH_CR3_I1_AOI1_ENABLED,
  .int1_latch = ST_LIS2DH_CR5_LIR_INT1,
  .hpis1 = ST_LIS2DH_CR2_HPENABLED_INT1,
  .hpis2 = ST_LIS2DH_CR2_HPENABLED_INT2,
  .fds = ST_LIS2DH_CR2_FDS,
  .int2_aoi_enable = ST_LIS2DH_CR3_I1_AOI2_ENABLED,
  .int2_latch = ST_LIS2DH_CR5_LIR_INT2,
  .fifo_enable = 0x00,
  .fifo_mode = 0x00,
  .hpmode = ST_LIS2DH_CR2_HPFILT_M_NORM,
  .hpcf = 0x00,
  .xen = ST_LIS2DH_CR1_XEN,
  .yen = ST_LIS2DH_CR1_YEN,
  .zen = ST_LIS2DH_CR1_ZEN,
  .int1_int_x_high_enable = ST_LIS2DH_INT_CFG_XHIE,
  .int1_int_y_high_enable = ST_LIS2DH_INT_CFG_YHIE,
  .int1_int_z_high_enable = ST_LIS2DH_INT_CFG_ZHIE,
  .int1_int_threshold = 6, /* 6 * 0.03125 = 0.1875g */
  .int1_int_duration = 0x00,
};

static struct accelerometer_config g_default_config =
{
  .restart_time = CONFIG_EXAMPLE_APP_DEFAULT_ACCEL_RESTART_TIME,
  .impact_threshold = CONFIG_EXAMPLE_APP_DEFAULT_ACCEL_IMPACT_THRESHOLD,
};

/* Current configuration for accelerometer, based on 'g_default_accel_setup',
 * modified by exapp_accelerometer_init(). */

static struct lis2dh_setup g_accel_setup;

static struct accel_s g_accel =
{
  .fd = -1,
  .restart_timerid = -1,
  .setup = &g_accel_setup,
};


#ifdef CONFIG_SDK_EXAMPLE_APP_DEBUG_VERBOSE
static void dbg_event (struct lis2dh_result *result)
{
  exapp_dbg("x: %.3f\n", ((double)result->measurements[0].x) / 1000);

  exapp_dbg("y: %.3f\n", ((double)result->measurements[0].y) / 1000);

  exapp_dbg("z: %.3f\n", ((double)result->measurements[0].z) / 1000);

  exapp_dbg("int1_occurred: 0x%08x\n", result->header.int1_occurred);

  exapp_dbg("int2_occurred: 0x%08x\n", result->header.int2_occurred);

  exapp_dbg("int1_source: 0x%08x\n", result->header.int1_source);

  exapp_dbg("int2_source: 0x%08x\n", result->header.int2_source);

  if (result->header.int1_source & LIS2DH_INT_SRC_XH)
    {
      exapp_dbg("1: x high\n");
    }

  if (result->header.int1_source & LIS2DH_INT_SRC_XL)
    {
      exapp_dbg("1: x low\n");
    }
  if (result->header.int1_source & LIS2DH_INT_SRC_YH)
    {
      exapp_dbg("1: y high\n");
    }

  if (result->header.int1_source & LIS2DH_INT_SRC_YL)
    {
      exapp_dbg("1: y low\n");
    }
  if (result->header.int1_source & LIS2DH_INT_SRC_ZH)
    {
      exapp_dbg("1: z high\n");
    }

  if (result->header.int1_source & LIS2DH_INT_SRC_ZL)
    {
      exapp_dbg("1: z low\n");
    }

  if (result->header.int2_source & LIS2DH_INT_SRC_XH)
    {
      exapp_dbg("2: x high\n");
    }

  if (result->header.int2_source & LIS2DH_INT_SRC_XL)
    {
      exapp_dbg("2: x low\n");
    }
  if (result->header.int2_source & LIS2DH_INT_SRC_YH)
    {
      exapp_dbg("2: y high\n");
    }

  if (result->header.int2_source & LIS2DH_INT_SRC_YL)
    {
      exapp_dbg("2: y low\n");
    }
  if (result->header.int2_source & LIS2DH_INT_SRC_ZH)
    {
      exapp_dbg("2: z high\n");
    }

  if (result->header.int2_source & LIS2DH_INT_SRC_ZL)
    {
      exapp_dbg("2: z low\n");
    }
}
#endif

static int accel_restart_timer_cb(const int timer_id,
                                  const struct timespec *date,
                                  void * const priv)
{
  struct accel_s *accel = priv;
  int ret;

  exapp_dbg("\n");

  accel->restart_timerid = -1;

  /* Restart accelerometer. */

  ret = exapp_accelerometer_init(accel->config, accel->client.callback,
                                 accel->client.priv);
  DEBUGASSERT(ret >= 0);

  return OK;
}

static void start_accel_restart_timer(struct accel_s * const accel)
{
  struct timespec ts;

  DEBUGASSERT(accel->restart_timerid < 0);

  if (accel->config->restart_time > 0)
    {
      const struct accelerometer_config *config = accel->config;

      /* Shutdown accelerometer. */

      exapp_accelerometer_uninit();

      /* Start restart timer. */

      clock_gettime(CLOCK_MONOTONIC, &ts);
      ts.tv_sec += config->restart_time;
      accel->restart_timerid =
          ts_core_timer_setup_date(&ts, accel_restart_timer_cb, accel);
      DEBUGASSERT(accel->restart_timerid >= 0);
    }
}

static void stop_accel_restart_timer(struct accel_s * const accel)
{
  if (accel->restart_timerid >= 0)
    {
      ts_core_timer_stop(accel->restart_timerid);
      accel->restart_timerid = -1;
    }
}

static int handle_accel_data(struct lis2dh_result *result,
                             struct accel_s * const accel)
{
  DEBUGASSERT(result && accel && result->header.meas_count <= RESULT_READ_COUNT);

  if (result->header.meas_count != 0)
    {
#ifdef CONFIG_SDK_EXAMPLE_APP_DEBUG_VERBOSE
      dbg_event(result);
#endif
      accel->client.callback(result, accel->client.priv);

      if (accel->fd >= 0)
        {
          start_accel_restart_timer(accel);
        }
    }
  else
    {
      exapp_dbg("Zero length data from driver!\n");
    }

  free(result);
  return OK;
}

static int accel_dev_cb(const struct pollfd * const pfd, void * const priv)
{
  struct accel_s * const accel = (struct accel_s *) priv;
  int bytes;
  struct lis2dh_result *result;

  DEBUGASSERT(pfd && priv);

  result = (struct lis2dh_result *) zalloc(
      sizeof(struct lis2dh_result)
          + RESULT_READ_COUNT * sizeof(struct lis2dh_vector_s));
  if (result == NULL)
    {
      exapp_dbg("Failed to allocate memory for result\n");
      return ERROR;
    }

  bytes = read(
      accel->fd,
      result,
      sizeof(struct lis2dh_result)
          + RESULT_READ_COUNT * sizeof(struct lis2dh_vector_s));

  if (bytes
      >= sizeof(struct lis2dh_result) + 1 * sizeof(struct lis2dh_vector_s))
    {
      return handle_accel_data(result, priv);
    }

  exapp_dbg("Read failed: %d\n", bytes);

  free(result);
  return ERROR;
}

static int accel_calc_threshold(double threshold, struct accel_s *accel)
{
  int shift;
  int reg_thres;

  shift = (accel->setup->fullscale >> 4) + 1;
  reg_thres = ((int) (threshold * 127)) >> shift;

  if (reg_thres > 127)
    reg_thres = 127;

  return reg_thres;
}

int exapp_accelerometer_init(const struct accelerometer_config *config,
                             accelerometer_callback_t callback, void *priv)
{
  struct accel_s * accel = &g_accel;
  int ret;
  int retry = 5;

  if (accel->fd >= 0)
    {
      /* Already initialized. */

      return OK;
    }

  memcpy(&g_accel_setup, &g_default_accel_setup, sizeof(g_accel_setup));
  accel->setup = &g_accel_setup;

  /* Configure impact wake-up threshold. For more complete dynamic
   * configuration, see 'apps/ts_engine/engine/sense_group_acceleration.c'. */
  accel->setup->int1_int_threshold =
      accel_calc_threshold(config->impact_threshold, accel);

  exapp_dbg("open lis2dh device\n");
  DEBUGASSERT(accel->fd < 0);

  accel->fd = open(LIS2DH_DEVICE, O_RDWR);
  if (accel->fd < 0)
    {
      exapp_dbg("Failed to open accelerometer device\n");
      return ERROR;
    }

  while (retry--)
    {
      ret = ioctl(accel->fd, SNIOC_WRITESETUP, (unsigned long)accel->setup);
      if (ret == OK)
        break;
    }

  if (retry == 0 && ret != 0)
    {
      exapp_dbg("Failed to setup LIS2DH\n");
      goto errout_close;
    }

  ret = ts_core_fd_register(accel->fd, POLLIN, accel_dev_cb, accel);
  if (ret < 0)
    {
      exapp_dbg("ts_core_fd_register failed\n");
      goto errout_close;
    }

  accel->config = config ? config : &g_default_config;
  accel->client.callback = callback;
  accel->client.priv = priv;

  return OK;

errout_close:
  close(g_accel.fd);
  g_accel.fd = -1;

  return ERROR;
}

void exapp_accelerometer_uninit(void)
{
  int ret;

  stop_accel_restart_timer(&g_accel);

  if (g_accel.fd < 0)
    {
      /* Already unregistered. */

      return;
    }

  exapp_dbg("unregister fd\n");
  DEBUGASSERT(g_accel.fd >= 0);

  ret = ts_core_fd_unregister(g_accel.fd);
  DEBUGASSERT (ret == OK);

  exapp_dbg("close lis2dh device\n");

  close(g_accel.fd);
  g_accel.fd = -1;
}

