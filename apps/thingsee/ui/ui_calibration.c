/****************************************************************************
 * apps/thingsee/ui/ui_calibration.c
 *
 *   Copyright (C) 2015-2016 Haltian Ltd. All rights reserved.
 *   Author: Petri Salonen <petri.salonen@haltian.com>
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

#include <debug.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <string.h>
#include <nuttx/irq.h>
#include <nuttx/sensors/lsm9ds1.h>
#include <apps/thingsee/ts_devinfo.h>
#include <apps/ts_engine/watchdog.h>

#include "ui_calibration.h"

#ifdef CONFIG_THINGSEE_NINEAXELS_MODULE

#define ST_MAGN_VALID_ID                0x3D
#define ST_SENS_PATH                    "/dev/lsm9ds1"

/****************************************************************************
 * Name: ui_calib_failure_handler
 *
 * Description:
 *  Helper function. Removes needs for goto's
 *
 * Input Parameters:
 *  file - file handler to be closed
 *
 * Returned Values:
 *  ERROR (-1) always, since this one is FAILURE HANDLER
 *
 ****************************************************************************/

static int ui_calib_failure_handler(int file)
{
  if (file > 0)
    {
      close(file);
    }

  return ERROR;
}

/****************************************************************************
 * Name: ui_calib_lsm9ds1_calibrate
 *
 * Description:
 *  Calibrate ST Lsm9ds1 sensor
 *
 * Input Parameters:
 *  sens_id - sensor's ID
 *
 * Returned Values:
 *  OK on calibration succeeded
 *
 ****************************************************************************/

int ui_calib_lsm9ds1_calibrate(int sens_id)
{
  int file;
  int ret;
  lsm9ds1_who_am_i_t who_am_i;
  lsm9ds1_sensor_bias_t b, ref_bias;
  char buf[TS_DEVICE_PROD_DATA_LSM9DS1_REF_SIZE];

  file = open(ST_SENS_PATH, O_RDWR);
  if (file < 0)
    {
      perror("Cannot open sensor to read/write");
      return ERROR;
    }

  ret = ioctl(file, LSM9DS1_IOC_WHO_AM_I, (unsigned int) &who_am_i);
  if (ret < 0)
    {
      return ui_calib_failure_handler(file);
    }

  if (who_am_i.magn != ST_MAGN_VALID_ID)
    {
      return ui_calib_failure_handler(file);
    }

  /* Timeconsuming operation, kick the watchdog to avoid resets */

  if (board_get_hw_ver() < BOARD_HWVER_B2_0)
    ts_watchdog_kick();

  if (sens_id == UI_CALIB_SENS_GYROSCOPE)
    ret = ioctl(file, LSM9DS1_IOC_CALIBRATE_BIAS_GYRO, (unsigned int) &b);
  else
    ret = ioctl(file, LSM9DS1_IOC_CALIBRATE_BIAS_MAG, (unsigned int) &b);

  if (ret < 0)
    {
      return ui_calib_failure_handler(file);
    }

  close(file);

  /* Read or create prod data */

  ret = ts_device_prod_data_get_entry("lsm9ds1_ref_bias", buf, sizeof(buf));
  if (ret <= 0)
    snprintf(buf, sizeof(buf), "[0,0,0,0,0,0,0,0,0]");

  /* Split prod data as reference */

  ret = sscanf(buf, "[%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd]",
               &ref_bias.gyro_bias[0], &ref_bias.gyro_bias[1], &ref_bias.gyro_bias[2],
               &ref_bias.xl_bias[0], &ref_bias.xl_bias[1], &ref_bias.xl_bias[2],
               &ref_bias.mag_bias[0], &ref_bias.mag_bias[1], &ref_bias.mag_bias[2]);

  /* If not enough entries, clear them all */

  if (ret != 9)
    memset(&ref_bias, 0, sizeof(lsm9ds1_sensor_bias_t));

  /* Copy the other bias data from the reference */

  if (sens_id == UI_CALIB_SENS_GYROSCOPE)
    {
      memcpy(b.mag_bias, ref_bias.mag_bias, sizeof(ref_bias.mag_bias));
    }
  else
    {
      memcpy(b.gyro_bias, ref_bias.gyro_bias, sizeof(ref_bias.gyro_bias));
      memcpy(b.xl_bias, ref_bias.xl_bias, sizeof(ref_bias.xl_bias));
    }

  snprintf(buf, sizeof(buf), "[%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd]",
           b.gyro_bias[0], b.gyro_bias[1], b.gyro_bias[2],
           b.xl_bias[0], b.xl_bias[1], b.xl_bias[2],
           b.mag_bias[0], b.mag_bias[1], b.mag_bias[2]);

  if (sens_id == UI_CALIB_SENS_GYROSCOPE)
    dbg("Got gyro & accelerometer bias. Full bias data: %s\n", buf);
  else
    dbg("Got magnetometer bias. Full bias data: %s\n", buf);

  ret = ts_device_prod_data_set_entry("lsm9ds1_ref_bias", buf);
  if (ret < 0)
    {
      return ui_calib_failure_handler(-1);
    }

  return OK;
}

#endif
