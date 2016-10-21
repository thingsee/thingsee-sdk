/****************************************************************************
 * apps/thingsee/nineaxls/ts_nineaxls_main.c
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
#include <debug.h>

#ifdef CONFIG_GAM9AXEL_SENS
#  include "lsm9ds0_module.h"
#elif defined(CONFIG_LSM9DS1_SENS)
#  include "lsm9ds1_module.h"
#  include "ts_nineaxls_fusion.h"
#else
#  error "What sensor module do you want?"
#endif

#ifdef CONFIG_GAM9AXEL_SENS
static int lsm9ds0_main(void)
{
  int ret = OK;
  uint8_t gyro_id = 0;
  uint8_t acc_mag_id = 0;
  uint16_t acc_xyz[3] = { 0 };
  uint16_t magn_xyz[3] = { 0 };
  uint8_t sts_reg_m = 0;
  uint8_t int_src_reg_m = 0;
  uint8_t sts_reg_a = 0;
  uint8_t int_gen_1_src = 0;
  uint8_t int_gen_2_src = 0;

  nineax_lsm9ds0_start();
  nineax_lsm9ds0_who_am_i(&gyro_id, &acc_mag_id);
  lldbg("Read ids: 0x%02X 0x%02X\n", gyro_id, acc_mag_id);

  nineax_lsm9ds0_config_xm();
  while (1)
    {
      nineax_lsm9ds0_read_status_xm(&sts_reg_m, &int_src_reg_m, &sts_reg_a,
                                    &int_gen_1_src, &int_gen_2_src);
      nineax_lsm9ds0_wait_for_sensor();
      nineax_lsm9ds0_read_xm(magn_xyz, acc_xyz);
      lldbg("Raw data, MAGN: 0x%04X, 0x%04X, 0x%04X. ACC: 0x%04X, 0x%04X, 0x%04X\n",
         magn_xyz[0], magn_xyz[1], magn_xyz[2], acc_xyz[0], acc_xyz[1],
         acc_xyz[2]);
    }
  nineax_lsm9ds0_self_test_xm();
  nineax_lsm9ds0_self_test_gyro();
  nineax_lsm9ds0_stop();

  return ret;
}

#elif defined(CONFIG_LSM9DS1_SENS)

static int lsm9ds1_main(void)
{
  int ret = OK;
  uint8_t gyro_id = 0;
  uint8_t mag_id = 0;
  int16_t data[9] = { 0 };
  int16_t bias[9] = { 0 };
  float fbias[6];
  float gyro_reso, xl_reso, mag_reso;

  nineax_lsm9ds1_start();
  nineax_lsm9ds1_who_am_i(&gyro_id, &mag_id);
  lldbg("Read ids: 0x%02X 0x%02X\n", gyro_id, mag_id);

  /* Get biases from EEPROM and configure sensor to use those values. */
  ret = nineax_lsm9ds1_load_bias_from_eeprom(bias);
  if (ret != OK)
    {
      lldbg("Warning: lsm9ds1 sensor uncalibrated, results may be poor!\n");
    }

  nineax_lsm9ds1_config_gyro();
  nineax_lsm9ds1_config_mag();

  nineax_lsm9ds1_read_resolutions(&gyro_reso, &xl_reso, &mag_reso);

  /* Convert biases to floating-point here, not during every sensor update. */
  fbias[0] = gyro_reso * bias[0];
  fbias[1] = gyro_reso * bias[1];
  fbias[2] = gyro_reso * bias[2];
  fbias[3] = xl_reso * bias[3];
  fbias[4] = xl_reso * bias[4];
  fbias[5] = xl_reso * bias[5];

  while (1)
    {
      float yaw, pitch, roll;
      static uint32_t cnt;
      float gx, gy, gz, ax, ay, az, mx, my, mz;

      nineax_lsm9ds1_wait_for_sensor();
      nineax_lsm9ds1_read_all(data);
      if (cnt % 100 == 0)
        {
          lldbg("Raw GYRO: %hd, %hd, %hd\n", data[0], data[1], data[2]);
          lldbg("Raw XL  : %hd, %hd, %hd\n", data[3], data[4], data[5]);
          lldbg("Raw MAGN: %hd, %hd, %hd\n", data[6], data[7], data[8]);
        }

      /* Calculate the gyro value into actual degrees per second, that depend
       * on the sensor scale in use.
       */
      gx = data[0] * gyro_reso - fbias[0];
      gy = data[1] * gyro_reso - fbias[1];
      gz = data[2] * gyro_reso - fbias[2];

      /* Calculate the acceleration value into actual g's, that depend on the
       * sensor scale in use.
       */
      ax = data[3] * xl_reso - fbias[3];
      ay = data[4] * xl_reso - fbias[4];
      az = data[5] * xl_reso - fbias[5];

      /* Actual magnetometer value in milli-Gauss. Bias is ignored because it
       * has been set into offset registers in device, which compensates for it
       * automatically.
       */
      mx = data[6] * mag_reso;
      my = data[7] * mag_reso;
      mz = data[8] * mag_reso;

      ts_nineaxls_fusion_update(gx, gy, gz, ax, ay, az, mx, my, mz);

      /* Orientation conversion is CPU-heavy, only do it once in a while. */

      if (cnt % 100 == 0)
        {
          ts_nineaxls_fusion_get_orientation(&yaw, &pitch, &roll);
          lldbg("Yaw = %.6g, Pitch = %.6g, Roll = %.6g\n", yaw, pitch, roll);
          lldbg("q = [%.6g %.6g %.6g %.6g]\n", ts_sensor_q[0], ts_sensor_q[1], ts_sensor_q[2], ts_sensor_q[3]);
        }
      cnt++;
    }

  nineax_lsm9ds1_stop();

  return ret;
}
#endif

int ts_nineaxls_main(void)
{
#ifdef CONFIG_GAM9AXEL_SENS
  return lsm9ds0_main();
#elif defined(CONFIG_LSM9DS1_SENS)
  return lsm9ds1_main();
#endif
}

