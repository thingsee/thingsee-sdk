/****************************************************************************
 * apps/thingsee/nineaxls/ts_nineaxls_quaternion.c
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

#include <math.h>
#include <float.h>
#include <errno.h>

/* Quaternion to hold algorithm output. This is a global variable for speed
 * optimization purposes.
 */
float ts_sensor_q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };

/* Someone needs to actually measure these: */

/* Gyroscope measurement error in rads/s (start at 40 deg/s) */
#define GYRO_MEAS_ERROR (M_PI * (40.0f / 180.0f))

/* Gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s) */
#define GYRO_MEAS_DRIFT (M_PI * (0.0f  / 180.0f))

/* Constant sqrt(3.0f/4.0f) */
#define SQRT_3_DIV_4 0.866025f

/* Madgwick scheme parameters: */

/* Compute beta, 0.041 in the paper [1]. Our value,
 * 0.6045995 is much larger and thus makes the filter converge
 * much faster.
 */
#define MAD_BETA (SQRT_3_DIV_4 * GYRO_MEAS_ERROR)

/* compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value. */
#define MAD_ZETA (SQRT_3_DIV_4 * GYRO_MEAS_DRIFT)

/* Mahony scheme parameters: */

/* MAH_KP is for proportional feedback, MAH_KI for integral error term. */
#define MAH_KP (2.0f * 5.0f)
#define MAH_KI 0.0f

/* 1/sqrt(x) using integer arithmetic. */
float inv_sqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

/* Helper macros, normalize 3- and 4-dimensional vectors in-place.  */
#define norm3d(x1, x2, x3) \
  do { \
    float norm; \
    norm = inv_sqrt((x1) * (x1) + (x2) * (x2) + (x3) * (x3)); \
    (x1) *= norm; \
    (x2) *= norm; \
    (x3) *= norm; \
  } while(0)

#define norm4d(x1, x2, x3, x4) \
  do { \
    float norm; \
    norm = inv_sqrt((x1) * (x1) + (x2) * (x2) + (x3) * (x3) + (x4) * (x4)); \
    (x1) *= norm; \
    (x2) *= norm; \
    (x3) *= norm; \
    (x4) *= norm; \
  } while(0)


#ifdef CONFIG_THINGSEE_NINEAXELS_FUSION_MADGWICK

/*
 * Implementation of Sebastian Madgwick's sensor fusion algorithm [1]
 * which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based
 * estimate of absolute device orientation which can then be converted to yaw, pitch, and roll.
 *
 * The performance of the orientation filter is at least as good as conventional Kalman-based
 * filtering algorithms but is much less computationally intensive; it can be performed on a
 * STM32L1 operating at 32 MHz
 *
 * References:
 *
 * [1] Madgwick, Sebastian O. H. An efficient orientation filter for inertial and inertial/magnetic
 *     sensor arrays (2010). Preliminary report. http://www.x-io.co.uk/res/doc/madgwick_internal_report.pdf
 *
 */

int ts_sensor_update_madgwick(float gx, float gy, float gz,
                              float ax, float ay, float az,
                              float mx, float my, float mz,
                              float deltat)
{
  int ret = 0;
  float q1 = ts_sensor_q[0];
  float q2 = ts_sensor_q[1];
  float q3 = ts_sensor_q[2];
  float q4 = ts_sensor_q[3];
  float hx, hy, _2bx, _2bz;
  float s1, s2, s3, s4;
  float qDot1, qDot2, qDot3, qDot4;

  float _2q1mx;
  float _2q1my;
  float _2q1mz;
  float _2q2mx;
  float _4bx;
  float _4bz;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  /* Normalise accelerometer and magnetometer measurements. */

  norm3d(ax, ay, az);
  norm3d(mx, my, mz);

  /* Reference direction of Earth's magnetic field */

  _2q1mx = 2.0f * q1 * mx;
  _2q1my = 2.0f * q1 * my;
  _2q1mz = 2.0f * q1 * mz;
  _2q2mx = 2.0f * q2 * mx;
  hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
  hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
  _2bx = sqrt(hx * hx + hy * hy);
  _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;

  /* Gradient decent algorithm corrective step */

  s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);


  /* Normalize step magnitude */

  norm4d(s1, s2, s3, s4);

  /* Compute rate of change of quaternion */

  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - MAD_BETA * s1;
  qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - MAD_BETA * s2;
  qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - MAD_BETA * s3;
  qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - MAD_BETA * s4;

  /* Integrate to yield quaternion */

  q1 += qDot1 * deltat;
  q2 += qDot2 * deltat;
  q3 += qDot3 * deltat;
  q4 += qDot4 * deltat;

  /* Normalize the output quaternion */

  norm4d(q1, q2, q3, q4);
  ts_sensor_q[0] = q1;
  ts_sensor_q[1] = q2;
  ts_sensor_q[2] = q3;
  ts_sensor_q[3] = q4;
  return ret;
}

#endif /* CONFIG_THINGSEE_NINEAXELS_FUSION_MADGWICK */

#ifdef CONFIG_THINGSEE_NINEAXELS_FUSION_MAHONY

/* Vector for integral error of Mahony method. */
static float mah_ierr[3] = {0.0f, 0.0f, 0.0f};

/*
 * Similar to Madgwick scheme but uses proportional and integral filtering on the error between
 * estimated reference vectors and measured ones.
 */
int ts_sensor_update_mahony(float gx, float gy, float gz,
                            float ax, float ay, float az,
                            float mx, float my, float mz,
                            float deltat)
{
  int ret = 0;
  float q1 = ts_sensor_q[0];
  float q2 = ts_sensor_q[1];
  float q3 = ts_sensor_q[2];
  float q4 = ts_sensor_q[3];
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float pa, pb, pc;

  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  /* Normalise accelerometer and magnetometer measurements */

  norm3d(ax, ay, az);
  norm3d(mx, my, mz);

  /* Reference direction of Earth's magnetic field */

  hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
  hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
  bx = sqrt((hx * hx) + (hy * hy));
  bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

  /* Estimated direction of gravity and magnetic field */

  vx = 2.0f * (q2q4 - q1q3);
  vy = 2.0f * (q1q2 + q3q4);
  vz = q1q1 - q2q2 - q3q3 + q4q4;
  wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
  wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
  wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

  /* Error is cross product between estimated direction and measured direction of gravity */

  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

  /* Accumulate integral error */

  if (MAH_KI > 0.0f)
    {
      mah_ierr[0] += ex;
      mah_ierr[1] += ey;
      mah_ierr[2] += ez;
    }
  else
    {
      mah_ierr[0] = 0.0f;
      mah_ierr[1] = 0.0f;
      mah_ierr[2] = 0.0f;
    }

  /* Apply feedback terms */

  gx = gx + MAH_KP * ex + MAH_KI * mah_ierr[0];
  gy = gy + MAH_KP * ey + MAH_KI * mah_ierr[1];
  gz = gz + MAH_KP * ez + MAH_KI * mah_ierr[2];

  /* Integrate rate of change of quaternion */

  pa = q2;
  pb = q3;
  pc = q4;
  q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
  q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
  q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
  q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

  /* Normalize the output quaternion */

  norm4d(q1, q2, q3, q4);
  ts_sensor_q[0] = q1;
  ts_sensor_q[1] = q2;
  ts_sensor_q[2] = q3;
  ts_sensor_q[3] = q4;
  return ret;
}

#endif /* CONFIG_THINGSEE_NINEAXELS_FUSION_MAHONY */

