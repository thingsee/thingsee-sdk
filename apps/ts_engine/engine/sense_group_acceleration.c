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
#include <nuttx/irq.h>

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
#include <apps/thingsee/ts_devinfo.h>

#include "eng_dbg.h"
#include "parse.h"
#include "sense.h"
#include "execute.h"
#include "util.h"

#define LIS2DH_DEVICE			"/dev/acc0"
#define RESULT_READ_COUNT		1

#define LIS2DH_INT_SRC_ZH		0x20
#define LIS2DH_INT_SRC_ZL		0x10
#define LIS2DH_INT_SRC_YH 		0x08
#define LIS2DH_INT_SRC_YL		0x04
#define LIS2DH_INT_SRC_XH		0x02
#define LIS2DH_INT_SRC_XL		0x01

#define ABS(x)				((x) < 0 ? -1 * (x) : (x))

#define MIN(x, y)			((x) < (y) ? (x) : (y))
#define MAX(x, y)			((x) > (y) ? (x) : (y))

#define ACCEL_MAX			16

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

typedef bool
(*accel_callback_t) (struct lis2dh_result *result, void * const priv);

enum accel_threshold_type
{
  THR_NONE, THR_LOW, THR_HIGH
};

enum accel_senses
{
  X, Y, Z, IMPACT, NUMBER_OF_ACCEL_SENSES
};

struct accel_threshold
{
  double value;
  enum accel_threshold_type type;
};

struct sense_id_map
{
  uint32_t sId;
  uint32_t sId_mapped;
  bool negate;
};

struct accel_s
{
  int fd;
  sq_queue_t callbacks;
  struct lis2dh_setup setup;
  struct accel_threshold thr[NUMBER_OF_ACCEL_SENSES];
  int refcount;
  const struct sense_id_map *map;
  bool impact_mode:1;
};

struct accel_callback_entry_s
{
  sq_entry_t entry;
  accel_callback_t callback;
  void * priv;
};

struct axis_map
{
  uint8_t axis;
  bool negate;
};

static struct accel_s g_accel =
  { 0 };

static const struct sense_id_map g_sense_id_map_default[] = {
    [X] = { SENSE_ID_LATERAL, SENSE_ID_LATERAL, false },
    [Y] = { SENSE_ID_LONGITUDINAL, SENSE_ID_LONGITUDINAL, false },
    [Z] = { SENSE_ID_VERTICAL, SENSE_ID_VERTICAL, false },
    [IMPACT] = { SENSE_ID_IMPACT, SENSE_ID_IMPACT, false },
  };

static const struct sense_id_map g_sense_id_map_b17[] = {
    [X] = { SENSE_ID_LATERAL, SENSE_ID_LATERAL, false },
    [Y] = { SENSE_ID_LONGITUDINAL, SENSE_ID_LONGITUDINAL, false },
    [Z] = { SENSE_ID_VERTICAL, SENSE_ID_VERTICAL, false },
    [IMPACT] = { SENSE_ID_IMPACT, SENSE_ID_IMPACT, false },
  };

static const struct sense_id_map *
get_sense_id_map(void)
{
  const struct sense_id_map *map = g_sense_id_map_default;

  /* Bug fix for HW bug SENSESW-428: B1.7 has wrong sensor orientation. */

  if (ts_device_is_prototype_before_B17() == 0)
    {
      map = g_sense_id_map_b17;
    }

  return map;
}

static uint32_t
map_sId(struct accel_s *accel, uint32_t sId)
{
  int i;
  int n = ARRAY_SIZE(g_sense_id_map_default);

  for (i = 0; i < n; i++)
    {
      if (accel->map[i].sId == sId)
        break;
    }

  DEBUGASSERT(i < n);

  return (i < n) ? accel->map[i].sId_mapped : SENSE_ID_INVALID;
}

static bool
map_negate(struct accel_s *accel, uint32_t sId)
{
  int i;
  int n = ARRAY_SIZE(g_sense_id_map_default);

  for (i = 0; i < n; i++)
    {
      if (accel->map[i].sId == sId)
        break;
    }

  DEBUGASSERT(i < n);

  return (i < n) ? accel->map[i].negate : false;
}

static void
accel_publish_event (struct accel_s * const accel, struct lis2dh_result *result)
{
  struct accel_callback_entry_s const * cb;

  DEBUGASSERT(accel != NULL);

  cb = (struct accel_callback_entry_s *) sq_peek(&accel->callbacks);
  while (cb)
    {
      if (cb->callback)
        {
          if (cb->callback (result, cb->priv))
            {
              break;
            }
        }

      cb = (struct accel_callback_entry_s *) sq_next(&cb->entry);
    }
}

static int
handle_accel_data (struct lis2dh_result *result, struct accel_s * const accel)
{
  DEBUGASSERT(result && accel && result->header.meas_count <= RESULT_READ_COUNT);

  if (result->header.meas_count != 0)
    {
      accel_publish_event (accel, result);
    }
  else
    {
      eng_dbg ("Zero length data from driver!\n");
    }

  free (result);
  return OK;
}

static int
accel_read_from_dev (int fd, struct accel_s *priv)
{
  int bytes;
  struct lis2dh_result *result;

  DEBUGASSERT(priv);

  result = (struct lis2dh_result *) zalloc (
      sizeof(struct lis2dh_result)
	  + RESULT_READ_COUNT * sizeof(struct lis2dh_vector_s));
  if (result == NULL)
    {
      eng_dbg ("Failed to allocate memory for result\n");
      return ERROR;
    }

  bytes = read(fd, result,
               sizeof(struct lis2dh_result) + RESULT_READ_COUNT * sizeof(struct lis2dh_vector_s));

  if (bytes
      >= sizeof(struct lis2dh_result) + 1 * sizeof(struct lis2dh_vector_s))
    {
      return handle_accel_data (result, priv);
    }

  eng_dbg ("Read failed: %d\n", bytes);

  free (result);
  return ERROR;
}

static int
accel_receiver (const struct pollfd * const pfd, void * const priv)
{
  struct accel_s * const accel = (struct accel_s *) priv;

  eng_dbg("+\n");

  DEBUGASSERT(pfd && priv);

  return accel_read_from_dev (accel->fd, accel);

}

static int
accel_calc_threshold (double threshold, struct accel_s *accel)
{
  int shift;

  shift = (accel->setup.fullscale >> 4) + 1;

  return ((int) (threshold * 127)) >> shift;
}

static double
accel_min (double min, struct accel_threshold *thr)
{
  if (thr->type == THR_HIGH && thr->value < min)
    {
      return thr->value;
    }

  return min;
}

static double
accel_max (double max, struct accel_threshold *thr)
{
  if (thr->type == THR_LOW && thr->value > max)
    {
      return thr->value;
    }

  return max;
}

static int
accel_callback_register (accel_callback_t callback, void * const priv)
{
  struct accel_s * accel = &g_accel;
  struct accel_callback_entry_s * cb;

  if (!callback)
    return ERROR;

  cb = (struct accel_callback_entry_s *) sq_peek(&accel->callbacks);
  while (cb)
    {
      if (cb->callback == callback)
        {
          cb->priv = priv;

          return OK;
        }

      cb = (struct accel_callback_entry_s *) sq_next(&cb->entry);
    }

  cb = malloc (sizeof(struct accel_callback_entry_s));
  if (!cb)
    {
      return ERROR;
    }
  cb->callback = callback;
  cb->priv = priv;

  sq_addlast (&cb->entry, &accel->callbacks);

  return OK;
}

static int
accel_callback_unregister (accel_callback_t callback)
{
  struct accel_s * accel = &g_accel;
  struct accel_callback_entry_s * cb;

  if (!callback)
    return ERROR;

  cb = (struct accel_callback_entry_s *) sq_peek(&accel->callbacks);
  while (cb)
    {
      if (cb->callback == callback)
        {
          sq_rem(&cb->entry, &accel->callbacks);
          free(cb);
          return OK;
        }

      cb = (struct accel_callback_entry_s *) sq_next(&cb->entry);
    }

  return ERROR;
}

#ifdef CONFIG_THINGSEE_ENGINE_DBG
static void
dbg_event (struct lis2dh_result *result)
{
  eng_dbg("x: %.3f\n", ((double)result->measurements[0].x) / 1000);

  eng_dbg("y: %.3f\n", ((double)result->measurements[0].y) / 1000);

  eng_dbg("z: %.3f\n", ((double)result->measurements[0].z) / 1000);

  eng_dbg("int1_occurred: 0x%08x\n", result->header.int1_occurred);

  eng_dbg("int2_occurred: 0x%08x\n", result->header.int2_occurred);

  eng_dbg("int1_source: 0x%08x\n", result->header.int1_source);

  eng_dbg("int2_source: 0x%08x\n", result->header.int2_source);

  if (result->header.int1_source & LIS2DH_INT_SRC_XH)
    {
      eng_dbg("1: x high\n");
    }

  if (result->header.int1_source & LIS2DH_INT_SRC_XL)
    {
      eng_dbg("1: x low\n");
    }
  if (result->header.int1_source & LIS2DH_INT_SRC_YH)
    {
      eng_dbg("1: y high\n");
    }

  if (result->header.int1_source & LIS2DH_INT_SRC_YL)
    {
      eng_dbg("1: y low\n");
    }
  if (result->header.int1_source & LIS2DH_INT_SRC_ZH)
    {
      eng_dbg("1: z high\n");
    }

  if (result->header.int1_source & LIS2DH_INT_SRC_ZL)
    {
      eng_dbg("1: z low\n");
    }

  if (result->header.int2_source & LIS2DH_INT_SRC_XH)
    {
      eng_dbg("2: x high\n");
    }

  if (result->header.int2_source & LIS2DH_INT_SRC_XL)
    {
      eng_dbg("2: x low\n");
    }
  if (result->header.int2_source & LIS2DH_INT_SRC_YH)
    {
      eng_dbg("2: y high\n");
    }

  if (result->header.int2_source & LIS2DH_INT_SRC_YL)
    {
      eng_dbg("2: y low\n");
    }
  if (result->header.int2_source & LIS2DH_INT_SRC_ZH)
    {
      eng_dbg("2: z high\n");
    }

  if (result->header.int2_source & LIS2DH_INT_SRC_ZL)
    {
      eng_dbg("2: z low\n");
    }
}
#endif

static bool
accel_cb_x (struct lis2dh_result *result, void * priv)
{
  struct ts_cause *cause = priv;

#ifdef CONFIG_THINGSEE_ENGINE_DBG
  eng_dbg("+\n");
  dbg_event (result);
#endif

  if ((result->header.int1_source & LIS2DH_INT_SRC_XH)
      || (result->header.int2_source & LIS2DH_INT_SRC_XL))
    {
      cause->dyn.sense_value.value.valuedouble = (double)result->measurements[0].x / 1000;
      if (map_negate(cause->dyn.priv, cause->dyn.sense_value.sId))
        {
          cause->dyn.sense_value.value.valuedouble *= -1;
        }
      return handle_cause (cause);
    }

  return false;
}

static bool
accel_cb_y (struct lis2dh_result *result, void * priv)
{
  struct ts_cause *cause = priv;

#ifdef CONFIG_THINGSEE_ENGINE_DBG
  eng_dbg("+\n");
  dbg_event (result);
#endif

  if ((result->header.int1_source & LIS2DH_INT_SRC_YH)
      || (result->header.int2_source & LIS2DH_INT_SRC_YL))
    {
      cause->dyn.sense_value.value.valuedouble = (double)result->measurements[0].y / 1000;
      if (map_negate(cause->dyn.priv, cause->dyn.sense_value.sId))
        {
          cause->dyn.sense_value.value.valuedouble *= -1;
        }
      return handle_cause (cause);
    }

  return false;
}

static bool
accel_cb_z (struct lis2dh_result *result, void * priv)
{
  struct ts_cause *cause = priv;

#ifdef CONFIG_THINGSEE_ENGINE_DBG
  eng_dbg("+\n");
  dbg_event (result);
#endif

  if ((result->header.int1_source & LIS2DH_INT_SRC_ZH)
      || (result->header.int2_source & LIS2DH_INT_SRC_ZL))
    {
      cause->dyn.sense_value.value.valuedouble = (double)result->measurements[0].z / 1000;
      if (map_negate(cause->dyn.priv, cause->dyn.sense_value.sId))
        {
          cause->dyn.sense_value.value.valuedouble *= -1;
        }
      return handle_cause (cause);
    }

  return false;
}

static bool
accel_cb_impact (struct lis2dh_result *result, void * priv)
{
  struct ts_cause *cause;

  cause = (struct ts_cause *) priv;

#ifdef CONFIG_THINGSEE_ENGINE_DBG
  eng_dbg("+\n");
  dbg_event (result);
#endif

  if ((result->header.int1_source & 0x2a)
      || (result->header.int2_source & 0x15))
    {
      int16_t x, y, z;

      x = result->measurements[0].x;
      y = result->measurements[0].y;
      z = result->measurements[0].z;

      cause->dyn.sense_value.value.valuedouble = (double) ub32sqrt(x * x + y * y + z * z) / 1000;
      return handle_cause (cause);
    }

  return false;
}

static int
accel_setup (struct ts_cause *cause, bool on)
{
  struct accel_s * accel = &g_accel;
  int ret;
  int retry = 5;
  double thr_greater, thr_lesser, thr_high = ACCEL_MAX, thr_low = 0;
  bool high = false, low = false;
  int i;
  accel_callback_t callback;
  bool thr_any;

  eng_dbg("0x%08x %s\n", cause->dyn.sense_value.sId, (on ? "on" : "off"));

  if (on && accel->refcount == 0)
    {
      accel->map = get_sense_id_map();

      eng_dbg("open lis2dh device\n");

      accel->fd = open (LIS2DH_DEVICE, O_RDWR);
      if (accel->fd < 0)
        {
          eng_dbg ("Failed to open accelerometer device: %d\n", errno);
          return ERROR;
        }

      accel->setup.data_rate = LIS2DH_ODR_10HZ;
      accel->setup.low_power_mode_enable = 0;

      accel->setup.temp_enable = false;
      accel->setup.bdu = ST_LIS2DH_CR4_BDU_UPD_ON_READ;
      accel->setup.fullscale = ST_LIS2DH_CR4_FULL_SCALE_4G;

      /* int1 for high events */

      accel->setup.int1_aoi_enable = ST_LIS2DH_CR3_I1_AOI1_ENABLED;
      accel->setup.int1_latch = ST_LIS2DH_CR5_LIR_INT1;

      accel->impact_mode = (cause->dyn.sense_value.sId == SENSE_ID_IMPACT);
      if (accel->impact_mode)
        {
          accel->setup.hpis1 = ST_LIS2DH_CR2_HPENABLED_INT1;
          accel->setup.hpis2 = ST_LIS2DH_CR2_HPENABLED_INT2;
          accel->setup.fds = ST_LIS2DH_CR2_FDS;
          accel->setup.hpmode = ST_LIS2DH_CR2_HPFILT_M_NORM;
        }
      else
        {
          accel->setup.hpis1 = 0;
          accel->setup.hpis2 = 0;
          accel->setup.fds = 0;
          accel->setup.hpmode = ST_LIS2DH_CR2_HPFILT_M_NORM2;
        }

      /* int2 for low events */

      accel->setup.int2_aoi_enable = ST_LIS2DH_CR3_I1_AOI2_ENABLED;
      accel->setup.int2_latch = ST_LIS2DH_CR5_LIR_INT2;

      accel->setup.fifo_enable = 0x00;
      accel->setup.fifo_mode = 0x00;
      accel->setup.hpcf = 0x00;

      for (i = 0; i < NUMBER_OF_ACCEL_SENSES; i++)
        {
          accel->thr[i].type = THR_NONE;
        }

      sq_init(&accel->callbacks);
    }

  switch (map_sId(accel, cause->dyn.sense_value.sId))
    {
    case SENSE_ID_LONGITUDINAL:
      callback = accel_cb_y;
      break;
    case SENSE_ID_LATERAL:
      callback = accel_cb_x;
      break;
    case SENSE_ID_VERTICAL:
      callback = accel_cb_z;
      break;
    case SENSE_ID_IMPACT:
      callback = accel_cb_impact;
      break;
    default:
      eng_dbg("invalid sId\n");
      goto errout_close;
    }

  if (on)
    {
      cause->dyn.priv = accel;

      eng_dbg("accel_callback_register: %p\n", callback);
      ret = accel_callback_register (callback, cause);
      if (ret != OK)
        {
          eng_dbg("accel_callback_register failed\n");
          goto errout_close;
        }
    }
  else
    {
      ret = accel_callback_unregister (callback);
      if (ret != OK)
        {
          eng_dbg("accel_callback_unregister failed\n");
          goto errout_close;
        }
    }

  bool found;

  if (on)
    {
      found = __ts_engine_find_threshold (cause, isGt, &thr_greater, NULL);

      if (found)
        {
          if (thr_greater >= 0)
            {
              high = true;
              thr_high = thr_greater;
            }
          else
            {
              low = true;
              thr_low = ABS(thr_greater);
            }
        }

      found = __ts_engine_find_threshold (cause, isLt, &thr_lesser, NULL);

      if (found)
        {
          if (thr_lesser >= 0)
            {
              low = true;
              thr_low = MAX(thr_low, thr_lesser);
            }
          else
            {
              high = true;
              thr_high = MIN(thr_high, ABS(thr_lesser));
            }
        }

      if (!high && !low)
        {
          found = __ts_engine_find_threshold (cause, isAny, NULL, &thr_any);
          if (found && thr_any)
            {
              low = true;
              thr_low = ACCEL_MAX;
            }
          else
            {
              eng_dbg ("no thresholds set, aborting setup\n");
              goto errout_close;
            }
        }
    }

  switch (map_sId(accel, cause->dyn.sense_info->sId))
    {
    case SENSE_ID_VERTICAL:

      if (on)
        {
          accel->setup.zen = ST_LIS2DH_CR1_ZEN;

          if (high)
            {
              accel->setup.int1_int_z_high_enable = ST_LIS2DH_INT_CFG_ZHIE;

              accel->thr[Z].type = THR_HIGH;
              accel->thr[Z].value = MIN(
                  thr_high,
                  (accel->thr[Z].value ? accel->thr[Z].value : ACCEL_MAX));
            }

          if (low)
            {
              accel->setup.int2_int_z_low_enable = ST_LIS2DH_INT_CFG_ZLIE;

              accel->thr[Z].type = THR_LOW;
              accel->thr[Z].value = MAX(thr_low, accel->thr[Z].value);
            }
        }
      else
        {
          accel->setup.zen = 0;

          accel->setup.int1_int_z_high_enable = 0;
          accel->setup.int2_int_z_low_enable = 0;

          accel->thr[Z].type = THR_NONE;
        }

      break;

    case SENSE_ID_LONGITUDINAL:

      if (on)
        {
          accel->setup.yen = ST_LIS2DH_CR1_YEN;

          if (high)
            {
              accel->setup.int1_int_y_high_enable = ST_LIS2DH_INT_CFG_YHIE;

              accel->thr[Y].type = THR_HIGH;
              accel->thr[Y].value = MIN(
                  thr_high,
                  (accel->thr[Y].value ? accel->thr[Y].value : ACCEL_MAX));
            }

          if (low)
            {
              accel->setup.int2_int_y_low_enable = ST_LIS2DH_INT_CFG_YLIE;

              accel->thr[Y].type = THR_LOW;
              accel->thr[Y].value = MAX(thr_low, accel->thr[Y].value);
            }
        }
      else
        {
          accel->setup.yen = 0;

          accel->setup.int1_int_y_high_enable = 0;
          accel->setup.int2_int_y_low_enable = 0;

          accel->thr[Y].type = THR_NONE;
        }

      break;

    case SENSE_ID_LATERAL:

      if (on)
        {
          accel->setup.xen = ST_LIS2DH_CR1_XEN;

          if (high)
            {
              accel->setup.int1_int_x_high_enable = ST_LIS2DH_INT_CFG_XHIE;

              accel->thr[X].type = THR_HIGH;
              accel->thr[X].value = MIN(
                  thr_high,
                  (accel->thr[X].value ? accel->thr[X].value : ACCEL_MAX));
            }

          if (low)
            {
              accel->setup.int2_int_x_low_enable = ST_LIS2DH_INT_CFG_XLIE;

              accel->thr[X].type = THR_LOW;
              accel->thr[X].value = MAX(thr_low, accel->thr[X].value);
            }
        }
      else
        {
          accel->setup.xen = 0;

          accel->setup.int1_int_x_high_enable = 0;
          accel->setup.int2_int_x_low_enable = 0;

          accel->thr[X].type = THR_NONE;
        }

      break;

    case SENSE_ID_IMPACT:

      if (on)
        {
          /* TODO: check in the purpose parse phase */

          if (accel->refcount > 0 && !accel->impact_mode)
            {
              eng_dbg("can not do x/y/z accel and impact at the same time\n");
              goto errout_close;
            }

          accel->setup.xen = ST_LIS2DH_CR1_XEN;
          accel->setup.yen = ST_LIS2DH_CR1_YEN;
          accel->setup.zen = ST_LIS2DH_CR1_ZEN;

          if (high)
            {
              accel->setup.int1_int_x_high_enable = ST_LIS2DH_INT_CFG_XHIE;
              accel->setup.int1_int_y_high_enable = ST_LIS2DH_INT_CFG_YHIE;
              accel->setup.int1_int_z_high_enable = ST_LIS2DH_INT_CFG_ZHIE;

              accel->thr[IMPACT].type = THR_HIGH;
              accel->thr[IMPACT].value =
                  MIN(thr_high,
                      (accel->thr[IMPACT].value ? accel->thr[IMPACT].value : ACCEL_MAX));
            }

          if (low)
            {
              accel->setup.int2_int_x_low_enable = ST_LIS2DH_INT_CFG_XLIE;
              accel->setup.int2_int_y_low_enable = ST_LIS2DH_INT_CFG_YLIE;
              accel->setup.int2_int_z_low_enable = ST_LIS2DH_INT_CFG_ZLIE;

              accel->thr[IMPACT].type = THR_LOW;
              accel->thr[IMPACT].value = MAX(thr_low, accel->thr[IMPACT].value);
            }
        }
      else
        {
          accel->setup.xen = 0;
          accel->setup.yen = 0;
          accel->setup.zen = 0;

          accel->setup.int1_int_x_high_enable = 0;
          accel->setup.int2_int_x_low_enable = 0;

          accel->setup.int1_int_y_high_enable = 0;
          accel->setup.int2_int_y_low_enable = 0;

          accel->setup.int1_int_z_high_enable = 0;
          accel->setup.int2_int_z_low_enable = 0;

          accel->thr[IMPACT].type = THR_NONE;
        }

      break;
    }

  double temp;

  if (high)
    {
      temp = ACCEL_MAX;

      for (i = 0; i < NUMBER_OF_ACCEL_SENSES; i++)
        {
          temp = accel_min (temp, &accel->thr[i]);
        }

      accel->setup.int1_int_threshold = accel_calc_threshold (temp, accel);
      accel->setup.int1_int_duration = 0x00;

      eng_dbg("int1_threshold: %d\n", accel->setup.int1_int_threshold);
    }

  if (low)
    {
      temp = 0;

      for (i = 0; i < NUMBER_OF_ACCEL_SENSES; i++)
        {
          temp = accel_max (temp, &accel->thr[i]);
        }

      accel->setup.int2_int_threshold = accel_calc_threshold (temp, accel);
      accel->setup.int2_int_duration = 0x00;

      eng_dbg("int2_threshold: %d\n", accel->setup.int2_int_threshold);
    }

  while (retry--)
    {
      ret = ioctl (accel->fd, SNIOC_WRITESETUP, (unsigned long) &accel->setup);
      if (ret == OK)
        break;
    }

  if (retry == 0 && ret != 0)
    {
      eng_dbg ("Failed to setup LIS2DH\n");
      goto errout_close;
    }

  if (on && accel->refcount == 0)
    {
      eng_dbg("register fd\n");

      ret = ts_core_fd_register (accel->fd, POLLIN, accel_receiver, accel);
      if (ret < 0)
        {
          eng_dbg ("ts_core_fd_register failed: %d\n", errno);
          goto errout_close;
        }
    }

  if (on)
    {
      accel->refcount++;
    }
  else
    {
      accel->refcount--;
    }

  if (accel->refcount == 0)
    {
      eng_dbg("unregister fd\n");

      ret = ts_core_fd_unregister (accel->fd);
      DEBUGASSERT (ret == OK);

      eng_dbg ("close lis2dh device\n");

      close (accel->fd);
      accel->fd = -1;
    }

  return OK;

  errout_close:

  if (accel->refcount > 0)
    {
      return ERROR;
    }

  eng_dbg("close fd\n");

  close (accel->fd);
  accel->fd = -1;

  return ERROR;
}

int
sense_acceleration_init (struct ts_cause *cause)
{
  int ret;

  ret = accel_setup (cause, true);
  if (ret < 0)
    {
      eng_dbg("accel_setup failed\n");
      return ERROR;
    }

  return OK;
}

int
sense_acceleration_uninit (struct ts_cause *cause)
{
  int ret;

  ret = accel_setup (cause, false);
  if (ret < 0)
    {
      eng_dbg("accel_setup failed\n");
      return ERROR;
    }

  return OK;
}
