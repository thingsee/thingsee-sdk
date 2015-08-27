/****************************************************************************
 * apps/thingsee/nineaxls/ts_nineaxls_fusion.c
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
 *   Author: Juha Niskanen <juha.niskanen@haltian.com>
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
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>
#include <sys/types.h>
#include <math.h>
#include <debug.h>
#include <nuttx/clock.h>

#include "ts_nineaxls_fusion.h"


/* Device's default magnetic declination. Should be updated
 * from external data periodically, and if device moves around
 * the globe.
 * Value in Oulu, Finland is +10° 17' EAST on 25th of June, 2015.
 */
static float ts_device_declination = +10.28333333f;


/* Update sensor fusion internal state from sensor raw data and calibration. */
int ts_nineaxls_fusion_update(float gx, float gy, float gz,
                              float ax, float ay, float az,
                              float mx, float my, float mz)
{
  int ret = OK;
  float deltat = 0.0f;
  static float sum = 0.0f;
  static uint32_t sum_count = 0;
  static uint32_t ltick = 0;
  uint32_t ctick = 0;

  /* Sensor x, y, and z axes of the accelerometer and gyro are aligned. The magnetometer
   * z-axis (+ up) is aligned with the z-axis (+ up) of accelerometer and gyro, but the
   * magnetometer x-axis is aligned with the negative x-axis of the gyro and the magnetometer.
   * Carefully note the changing position of the white "orientation dot" in confusingly drawn
   * diagram in ST DocID 025715 Rev 1, Fig 1, (p.10).
   *
   * We have to adjust for this orientation mismatch when feeding the output to the quaternion filter.
   * For the LSM9DS1, we have chosen a magnetic rotation that keeps the sensor forward along the
   * x-axis just like in the LSM9DS0 sensor. This rotation can be modified to allow any convenient
   * orientation convention. We don't know if it is ok to orient your aircraft in this fashion.
   */
#if defined(CONFIG_LSM9DS1_SENS)
  mx = -mx;
#endif

  /* For integration in filter algo */
  ctick = clock_systimer();
  deltat = (ctick - ltick) / (MSEC_PER_TICK * 1000.0f);
  ltick = ctick;

  /* Sum for averaging filter update rate */
  sum += deltat;
  sum_count++;

  /* The gyro rate needs to be rad/s */
#if defined(CONFIG_THINGSEE_NINEAXELS_FUSION_MADGWICK)
  ret = ts_sensor_update_madgwick(gx * M_PI / 180.0f, gy * M_PI / 180.0f, gz * M_PI / 180.0f,
                                  ax, ay, az,
                                  mx, my, mz,
                                  deltat);
#elif defined(CONFIG_THINGSEE_NINEAXELS_FUSION_MAHONY)
  ret = ts_sensor_update_mahony(gx * M_PI / 180.0f, gy * M_PI / 180.0f, gz * M_PI / 180.0f,
                                ax, ay, az,
                                mx, my, mz,
                                deltat);
#else
  /* No algorithm! */
  ret = -1;
#endif

  if (ret < 0)
     dbg("sensor filter is misbehaving!\n");

  //dbg("sensor rate %2.4g HZ\n", (float)sum_count / sum);

  return ret;
}

/* Get fusioned state as quaternion. */
int ts_nineaxls_fusion_get_quaternion(float o_q[static 4])
{
  int ret = OK;

  o_q[0] = ts_sensor_q[0];
  o_q[1] = ts_sensor_q[1];
  o_q[2] = ts_sensor_q[2];
  o_q[3] = ts_sensor_q[3];

  return ret;
}

/* Set magnetic declination in device's current locale. Use value zero, if want
 * to point your compass to Magnetic North Pole instead of true north.
 *
 * Returns old declination value.
 */
float ts_nineaxls_fusion_set_declination(float decl)
{
  float old = ts_device_declination;

  ts_device_declination = decl;
  return old;
}

/* Get current orientation of your ThingSee device, in format suitable for navigation. */
int ts_nineaxls_fusion_get_orientation(float *o_yaw, float *o_pitch, float *o_roll)
{
  int ret = OK;
  float yaw, pitch, roll;

  /* Convert output variables from updated quaternion. These are Tait-Bryan angles, commonly used
   * for aircraft orientation. In this coordinate system, the positive z-axis is down toward Earth.
   *
   * Yaw is the angle between sensor x-axis and Earth magnetic North (or true North if corrected for
   * local declination, looking down on the sensor positive yaw is counterclockwise.
   *
   * Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive,
   * up toward the sky is negative.
   *
   * Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
   *
   * These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
   * Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct
   * orientation the rotations must be applied in the correct order which for this configuration
   * is yaw, pitch, and then roll.
   */

#define q ts_sensor_q
  yaw   = atan2f(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
  pitch = -asinf(2.0f * (q[1] * q[3] - q[0] * q[2]));
  roll  = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
#undef q

  /* Convert yaw to degrees and compensate for declination. */

  yaw   *= 180.0f / M_PI;
  yaw   -= ts_device_declination;

  /* See SENSESW-857 for minus sign in pitch and roll. */

  pitch *= -180.0f / M_PI;
  roll  *= -180.0f / M_PI;

  *o_yaw = yaw;
  *o_pitch = pitch;
  *o_roll = roll;

  return ret;
}

