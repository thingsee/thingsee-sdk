/****************************************************************************
 * apps/ts_engine/engine/sense_group_orientation.c
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

#include <sys/types.h>
#include <string.h>
#include <math.h>

#include "eng_dbg.h"
#include "parse.h"
#include "sense.h"
#include "execute.h"

#include "sense_group_orientation.h"

#include "../../thingsee/nineaxls/lsm9ds1_module.h" /* TODO */
#include "../../thingsee/nineaxls/ts_nineaxls_fusion.h" /* TODO */

#define X       0x01
#define Y       0x02
#define Z       0x04

#define YAW     0x01
#define PITCH   0x02
#define ROLL    0x04

enum sense
{
  ACCEL = 0,
  MAGN,
  GYRO,
  FUSION,
  NUMBER_OF_SENSES
};

struct nineax_s
{
  float fbias[6];
  float gyro_reso, xl_reso, mag_reso;
  int refcount;
  int fd;
  bool usefusion:1;
  uint8_t valid[NUMBER_OF_SENSES];
};

struct nineax_s g_nineax =
  { .refcount = 0, .fd = -1 };

static void
read_accel(float *ax, float *ay, float *az)
{
  uint16_t data[3];

  nineax_lsm9ds1_read_xl (data);

  /* Calculate the acceleration value into actual g's, that depend on the
   * sensor scale in use.
   */
  *ax = (int16_t) data[0] * g_nineax.xl_reso - g_nineax.fbias[3];
  *ay = (int16_t) data[1] * g_nineax.xl_reso - g_nineax.fbias[4];
  *az = (int16_t) data[2] * g_nineax.xl_reso - g_nineax.fbias[5];
}

static void
read_magnetic_field(float *mx, float *my, float *mz)
{
  uint16_t data[3];

  nineax_lsm9ds1_read_mag (data);

  /* Actual magnetometer value in milli-Gauss. Bias is ignored because it
   * has been set into offset registers in device, which compensates for it
   * automatically.
   */
  *mx = (int16_t) data[0] * g_nineax.mag_reso;
  *my = (int16_t) data[1] * g_nineax.mag_reso;
  *mz = (int16_t) data[2] * g_nineax.mag_reso;
}

static void
read_gyro(float *gx, float *gy, float *gz)
{
  uint16_t data[3];

  nineax_lsm9ds1_read_gyro (data);

  /* Calculate the gyro value into actual degrees per second, that depend
   * on the sensor scale in use.
   */
  *gx = (int16_t) data[0] * g_nineax.gyro_reso - g_nineax.fbias[0];
  *gy = (int16_t) data[1] * g_nineax.gyro_reso - g_nineax.fbias[1];
  *gz = (int16_t) data[2] * g_nineax.gyro_reso - g_nineax.fbias[2];
}

static int
sense_fusion_read(void)
{
  int16_t data[9];
  float gx, gy, gz, ax, ay, az, mx, my, mz;
  int ret;

  ret = nineax_lsm9ds1_read_all(data);
  if (ret < 0)
    {
       eng_dbg("nineax_lsm9ds1_read_all failed\n");
       return ret;
    }
#if 0
  eng_dbg("%hd %hd %hd %hd %hd %hd %hd %hd %hd\n",
          data[0], data[1], data[2], data[3], data[4],
          data[5], data[6], data[7], data[8]);
#endif

  /* Calculate the gyro value into actual degrees per second, that depend
   * on the sensor scale in use.
   */
  gx = data[0] * g_nineax.gyro_reso - g_nineax.fbias[0];
  gy = data[1] * g_nineax.gyro_reso - g_nineax.fbias[1];
  gz = data[2] * g_nineax.gyro_reso - g_nineax.fbias[2];

  /* Calculate the acceleration value into actual g's, that depend on the
   * sensor scale in use.
   */
  ax = data[3] * g_nineax.xl_reso - g_nineax.fbias[3];
  ay = data[4] * g_nineax.xl_reso - g_nineax.fbias[4];
  az = data[5] * g_nineax.xl_reso - g_nineax.fbias[5];

  /* Actual magnetometer value in milli-Gauss. Bias is ignored because it
   * has been set into offset registers in device, which compensates for it
   * automatically.
   */
  mx = data[6] * g_nineax.mag_reso;
  my = data[7] * g_nineax.mag_reso;
  mz = data[8] * g_nineax.mag_reso;

  ts_nineaxls_fusion_update (gx, gy, gz, ax, ay, az, mx, my, mz);

  return ret;
}

static int
sense_fusion_callback(const struct pollfd * const pfd,
    void * const priv)
{
  (void) sense_fusion_read();
  return OK;
}

static int
init_fusion(void)
{
  int ret = OK;

  if (!g_nineax.usefusion)
    {
      ret = ts_core_fd_register(g_nineax.fd, POLLIN, sense_fusion_callback, NULL);
      if (ret < 0)
        {
          eng_dbg("ts_core_fd_register failed\n");
          return ret;
        }
      g_nineax.usefusion = true;
    }

  return ret;
}

static void
read_fusion(float *yaw, float *pitch, float *roll)
{
  init_fusion();

  /* Do orientation conversion for latest fusion state.
     Note that this is quite CPU heavy operation. */

  ts_nineaxls_fusion_get_orientation (yaw, pitch, roll);
}

int
sense_orientation_init (struct ts_cause *cause)
{
  uint8_t gyro_id = 0;
  uint8_t mag_id = 0;
  int16_t bias[9] = { 0 };
  int ret, i;

  if (g_nineax.refcount == 0)
    {
      eng_dbg("starting lsm9ds1\n");

      nineax_lsm9ds1_start ();
      nineax_lsm9ds1_who_am_i (&gyro_id, &mag_id);
      eng_lldbg("Read ids: 0x%02X 0x%02X\n", gyro_id, mag_id);

      g_nineax.fd = nineax_lsm9ds1_get_fd();
      if (g_nineax.fd < 0)
        {
          eng_dbg("nineax_lsm9ds1_get_fd failed\n");
          return ERROR;
        }
      g_nineax.usefusion = false;

      /* Get biases from EEPROM and configure sensor to use those values. */

      ret = nineax_lsm9ds1_load_bias_from_eeprom(bias);
      if (ret != OK)
        {
          eng_dbg("Warning: lsm9ds1 sensor uncalibrated, results may be poor!\n");
        }

      nineax_lsm9ds1_config_gyro ();
      nineax_lsm9ds1_config_mag ();

      nineax_lsm9ds1_read_resolutions (&g_nineax.gyro_reso, &g_nineax.xl_reso,
				       &g_nineax.mag_reso);

      /* Convert biases to floating-point here, not during every sensor update. */

      g_nineax.fbias[0] = g_nineax.gyro_reso * bias[0];
      g_nineax.fbias[1] = g_nineax.gyro_reso * bias[1];
      g_nineax.fbias[2] = g_nineax.gyro_reso * bias[2];
      g_nineax.fbias[3] = g_nineax.xl_reso * bias[3];
      g_nineax.fbias[4] = g_nineax.xl_reso * bias[4];
      g_nineax.fbias[5] = g_nineax.xl_reso * bias[5];
    }

  g_nineax.refcount++;

  for (i = 0; i < NUMBER_OF_SENSES; i++)
    {
      g_nineax.valid[i] = 0;
    }

  return OK;
}

int
sense_orientation_uninit (struct ts_cause *cause)
{
  g_nineax.refcount--;

  if (g_nineax.refcount == 0)
    {
      if (g_nineax.usefusion)
        {
          int ret;

          ret = ts_core_fd_unregister(g_nineax.fd);
          if (ret < 0)
            {
              eng_dbg("ts_core_fd_unregister failed\n");
            }
          g_nineax.usefusion = false;
        }

      eng_dbg("stopping lsm9ds1\n");

      nineax_lsm9ds1_stop ();
    }
  return OK;
}

int
sense_lsm9ds1_acceleration (struct ts_cause *cause)
{
  static float ax, ay, az;

  switch (cause->dyn.sense_value.sId & 0xffffff00)
    {
    case SENSE_ID_LONGITUDINAL:
      if (!(g_nineax.valid[ACCEL] & Y))
        {
          read_accel(&ax, &ay, &az);
          g_nineax.valid[ACCEL] = X | Z;
        }
      cause->dyn.sense_value.value.valuedouble = ay;
      break;

    case SENSE_ID_VERTICAL:
      if (!(g_nineax.valid[ACCEL] & Z))
        {
          read_accel(&ax, &ay, &az);
          g_nineax.valid[ACCEL] = X | Y;
        }
      cause->dyn.sense_value.value.valuedouble = az;
      break;

    case SENSE_ID_LATERAL:
      if (!(g_nineax.valid[ACCEL] & X))
        {
          read_accel(&ax, &ay, &az);
          g_nineax.valid[ACCEL] = Y | Z;
        }
      cause->dyn.sense_value.value.valuedouble = ax;
      break;
    }

  return handle_cause_event (cause, NULL);
}

int
sense_lsm9ds1_magnetic_field (struct ts_cause *cause)
{
  static float mx, my, mz;

  switch (cause->dyn.sense_value.sId & 0xffffff00)
    {
    case SENSE_ID_MAGNETIC_FIELD_LONGITUDINAL:
      if (!(g_nineax.valid[MAGN] & Y))
        {
          read_magnetic_field(&mx, &my, &mz);
          g_nineax.valid[MAGN] = X | Z;
        }
      cause->dyn.sense_value.value.valuedouble = my;
      break;

    case SENSE_ID_MAGNETIC_FIELD_VERTICAL:
      if (!(g_nineax.valid[MAGN] & Z))
        {
          read_magnetic_field(&mx, &my, &mz);
          g_nineax.valid[MAGN] = X | Y;
        }
      cause->dyn.sense_value.value.valuedouble = mz;
      break;

    case SENSE_ID_MAGNETIC_FIELD_LATERAL:
      if (!(g_nineax.valid[MAGN] & X))
        {
          read_magnetic_field(&mx, &my, &mz);
          g_nineax.valid[MAGN] = Y | Z;
        }
      cause->dyn.sense_value.value.valuedouble = mx;
      break;
    }

  return handle_cause_event (cause, NULL);
}

int
sense_lsm9ds1_angle_speed (struct ts_cause *cause)
{
  static float gx, gy, gz;

  switch (cause->dyn.sense_value.sId & 0xffffff00)
    {
    case SENSE_ID_ANGLE_SPEED_YAW:
      if (!(g_nineax.valid[GYRO] & Y))
        {
          read_gyro(&gx, &gy, &gz);
          g_nineax.valid[GYRO] = X | Z;
        }
      cause->dyn.sense_value.value.valuedouble = gy;
      break;

    case SENSE_ID_ANGLE_SPEED_PITCH:
      if (!(g_nineax.valid[GYRO] & Z))
        {
          read_gyro(&gx, &gy, &gz);
          g_nineax.valid[GYRO] = X | Y;
        }
      cause->dyn.sense_value.value.valuedouble = gz;
      break;

    case SENSE_ID_ANGLE_SPEED_ROLL:
      if (!(g_nineax.valid[GYRO] & X))
        {
          read_gyro(&gx, &gy, &gz);
          g_nineax.valid[GYRO] = Y | Z;
        }
      cause->dyn.sense_value.value.valuedouble = gx;
      break;
    }

  return handle_cause_event (cause, NULL);
}

int
sense_orientation (struct ts_cause *cause)
{
  static float yaw, pitch, roll;

  switch (cause->dyn.sense_value.sId)
    {
    case SENSE_ID_HEADING:
      /* Heading is same as yaw, but scaled to [0, 360] range. */

      if (!(g_nineax.valid[FUSION] & YAW))
        {
          read_fusion(&yaw, &pitch, &roll);
          g_nineax.valid[FUSION] = PITCH | ROLL;
        }
      float heading = (yaw < 0.0f) ? yaw + 360.0f : yaw;
      cause->dyn.sense_value.value.valuedouble = heading;
      break;

    case SENSE_ID_YAW:
      if (!(g_nineax.valid[FUSION] & YAW))
        {
          read_fusion(&yaw, &pitch, &roll);
          g_nineax.valid[FUSION] = PITCH | ROLL;
        }
      cause->dyn.sense_value.value.valuedouble = yaw;
      break;

    case SENSE_ID_PITCH:
      if (!(g_nineax.valid[FUSION] & PITCH))
        {
          read_fusion(&yaw, &pitch, &roll);
          g_nineax.valid[FUSION] = YAW | ROLL;
        }
      cause->dyn.sense_value.value.valuedouble = pitch;
      break;

    case SENSE_ID_ROLL:
      if (!(g_nineax.valid[FUSION] & ROLL))
        {
          read_fusion(&yaw, &pitch, &roll);
          g_nineax.valid[FUSION] = YAW | PITCH;
        }
      cause->dyn.sense_value.value.valuedouble = roll;
      break;
    }

  return handle_cause_event (cause, NULL);
}
