/****************************************************************************
 * apps/thingsee/nineaxls/lsm9ds1_module.c 
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

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <debug.h>
#include <apps/thingsee/ts_devinfo.h>

#include <nuttx/sensors/lsm9ds1.h>

#include "lsm9ds1_module.h"

typedef struct lsm9ds1_dev_t
  {
    int fd;
    lsm9ds1_fifo_mode_t fifo_mode_g;
    uint8_t fifo_samples_nbr_g;
    float gyro_reso;
    float xl_reso;
    float mag_reso;
  } lsm9ds1_dev_t;

static lsm9ds1_dev_t sensor =
  { .fd = -1 };

/****************************************************************************
 * Name: nineax_lsm9ds1_reset_fifo
 *
 * Description:
 *  Resets FIFO for gyro
 *
 * Input Parameters:
 * 	first_time - first time initialization reset
 *
 * Returned Values:
 *   On success returns 0. On Error returns -1
 *
 ****************************************************************************/

int nineax_lsm9ds1_reset_fifo(bool first_time)
{
  int ret = OK;
  lsm9ds1_gyro_config_data_t config_data = {
    .fifo_mode = LSM9DS1_FIFO_BYPASS,
    .fifo_samples_nbr = 0
  };

  ret = ioctl(sensor.fd, LSM9DS1_IOC_RST_FIFO_GYRO, (unsigned int)&config_data);
  if (ret < 0)
    {
      return ERROR;
    }

  if (!first_time)
    {
      config_data.fifo_mode = sensor.fifo_mode_g;
      config_data.fifo_samples_nbr = sensor.fifo_samples_nbr_g;
      ret = ioctl(sensor.fd, LSM9DS1_IOC_RST_FIFO_GYRO, (unsigned int)&config_data);
    }

  return ret;
}

/****************************************************************************
 * Name: nineax_lsm9ds1_who_am_i
 *
 * Description:
 *  Gets sensors' ids
 *
 * Input Parameters:
 * 	gyro_id - gyro-sensor's id. Should return 0xD4
 * 	mag_id - magnetometer's id. Should return 0x49
 *
 * Returned Values:
 *   On success returns 0. On Error returns -1
 *
 ****************************************************************************/

int nineax_lsm9ds1_who_am_i(uint8_t * gyro_id, uint8_t * mag_id)
{
  int ret = OK;
  lsm9ds1_who_am_i_t who_am_i;

  ret = ioctl(sensor.fd, LSM9DS1_IOC_WHO_AM_I, (unsigned int)&who_am_i);
  if (ret < 0)
    {
      return ERROR;
    }

  *gyro_id = who_am_i.acc_gyro;
  *mag_id = who_am_i.magn;

  return ret;
}

/****************************************************************************
 * Name: nineax_lsm9ds1_start
 *
 * Description:
 *  Opens 9-axels sensor
 *
 * Input Parameters:
 *
 * Returned Values:
 *   On success returns 0. On Error returns -1
 *
 ****************************************************************************/

int nineax_lsm9ds1_start(void)
{
  int ret = OK;

  sensor.fd = open("/dev/lsm9ds1", O_RDWR);
  if (sensor.fd < 0)
    {
      perror("Cannot open file");
      return ERROR;
    }

  return ret;
}

/****************************************************************************
 * Name: nineax_lsm9ds1_config_gyro
 *
 * Description:
 *  Setups gyro-sensor
 *
 * Input Parameters:
 *
 * Returned Values:
 *   On success returns 0. On Error returns -1
 *
 ****************************************************************************/

int nineax_lsm9ds1_config_gyro(void)
{
  int ret = OK;
  uint8_t fifo_sts = 0;
  uint8_t int_gyro_sts = 0;
  uint8_t int_xl_sts = 0;
  uint8_t status_reg = 0;
  lsm9ds1_gyro_config_data_t config_data = {
    .odr = LSM9DS1_GYRO_ODR_238HZ,
    .state = LSM9DS1_GYRO_STATE_NORMAL,
    .is_reboot = true,
    .scale = LSM9DS1_GYRO_SCALE_245DPS,
    .xl_scale = LSM9DS1_XL_SCALE_2G,
    .fifo_mode = LSM9DS1_FIFO_STREAM,
    .fifo_samples_nbr = 31,
    .fifo_en = true,
    .hpen = true,
    .bdu = true,
    .is_high = true,
    .threshold_int = true,
    .x_threshold = 0x7F,
    .y_threshold = 0x7F,
    .z_threshold = 0xFF,
    .int_gen_dur_g = 0,
    .int_gen_dur_xl = 0,
  };

  ret = nineax_lsm9ds1_reset_fifo(true);
  if (ret < 0)
    {
      return ERROR;
    }

  ret = nineax_lsm9ds1_read_status_gyro(&fifo_sts, &int_gyro_sts, &int_xl_sts,
                                    &status_reg);
  if (ret < 0)
    {
      return ERROR;
    }

  ret = ioctl(sensor.fd, LSM9DS1_IOC_CONFIG_GYRO, (unsigned int)&config_data);
  if (ret < 0)
    {
      return ERROR;
    }
  sensor.fifo_mode_g = config_data.fifo_mode;
  sensor.fifo_samples_nbr_g = config_data.fifo_samples_nbr;
  sensor.gyro_reso = lsm9ds1_gyro_resolution[config_data.scale];
  sensor.xl_reso = lsm9ds1_xl_resolution[config_data.xl_scale];

  return ret;
}

/****************************************************************************
 * Name: nineax_lsm9ds1_self_test_gyro
 *
 * Description:
 *  Runs gyro's self-test
 *
 * Input Parameters:
 *
 * Returned Values:
 *   On success returns 0. On Error returns -1
 *
 ****************************************************************************/

int nineax_lsm9ds1_self_test_gyro(void)
{
  int ret = OK;
  uint16_t raw_data[3] = { 0 };
  lsm9ds1_gyro_config_data_t config_data = {
    .state = LSM9DS1_GYRO_STATE_NORMAL,
    .fifo_en = false,
    .hpen = false,
    .bdu = false,
    .latch_int = false,
    .is_high = false,
    .threshold_int = false,
    .scale = LSM9DS1_GYRO_SCALE_245DPS,
    .odr = LSM9DS1_GYRO_ODR_238HZ,
  };

  ret = ioctl(sensor.fd, LSM9DS1_IOC_CONFIG_GYRO, (unsigned int)&config_data);
  if (ret < 0)
    {
      return ERROR;
    }

  ret = nineax_lsm9ds1_read_gyro(raw_data);

  lldbg("First self-test gyro data: 0x%04X, 0x%04X, 0x%04X\n", raw_data[0],
        raw_data[1], raw_data[2]);

  ret = ioctl(sensor.fd, LSM9DS1_IOC_CONFIG_GYRO, (unsigned int)&config_data);
  if (ret < 0)
    {
      return ERROR;
    }

  ret = nineax_lsm9ds1_read_gyro(raw_data);

  lldbg("Second self-test gyro data: 0x%04X, 0x%04X, 0x%04X\n", raw_data[0],
        raw_data[1], raw_data[2]);

  return ret;
}

/****************************************************************************
 * Name: nineax_lsm9ds1_config_mag
 *
 * Description:
 *  Setups magnetometer-sensor
 *
 * Input Parameters:
 *
 * Returned Values:
 *   On success returns 0. On Error returns -1
 *
 ****************************************************************************/

int nineax_lsm9ds1_config_mag(void)
{
  int ret = OK;

  /* Configure using driver's internal defaults. */
  ret = ioctl(sensor.fd, LSM9DS1_IOC_CONFIG_MAG, 0);
  if (ret < 0)
    {
      return ERROR;
    }

  /* We know this is the internal, default scale. */
  sensor.mag_reso = lsm9ds1_gyro_resolution[LSM9DS1_MAG_SCALE_4GS];

  return ret;
}

/****************************************************************************
 * Name: nineax_lsm9ds1_self_test_mag
 *
 * Description:
 *  Runs self-test
 *
 * Input Parameters:
 *
 * Returned Values:
 *   On success returns 0. On Error returns -1
 *
 ****************************************************************************/

int nineax_lsm9ds1_self_test_mag(void)
{
  int ret = OK;
  uint16_t magn_data[3] = { 0 };

  /* configure using driver's internal defaults */
  ret = ioctl(sensor.fd, LSM9DS1_IOC_CONFIG_MAG, 0);
  if (ret < 0)
    {
      return ERROR;
    }

  ret = nineax_lsm9ds1_read_mag(magn_data);
  if (ret < 0)
    {
      return ERROR;
    }

  lldbg("magn: 0x%04X, 0x%04X, 0x%04X\n",
        magn_data[0], magn_data[1], magn_data[2]);

  return ret;
}

/****************************************************************************
 * Name: nineax_lsm9ds1_read_mag
 *
 * Description:
 *  Reads xm-sensor
 *
 * Input Parameters:
 * 	raw_data - pointer to raw_data array (size of the array is 3. The very
 * 	first element is always X and the last is Z)
 *
 * Returned Values:
 *   On success returns 0. On Error returns -1
 *
 ****************************************************************************/

int nineax_lsm9ds1_read_mag(uint16_t * raw_data)
{
  int ret = OK;
  lsm9ds1_raw_data_t magn_raw;

  ret = ioctl(sensor.fd, LSM9DS1_IOC_READ_RAW_MAG, (unsigned int)&magn_raw);
  if (ret < 0)
    {
      return ERROR;
    }
  raw_data[0] = magn_raw.out_x;
  raw_data[1] = magn_raw.out_y;
  raw_data[2] = magn_raw.out_z;

  return ret;
}

/****************************************************************************
 * Name: nineax_lsm9ds1_read_status_mag
 *
 * Description:
 *  Reads magnetometer-sensor status
 *
 * Input Parameters:
 * 	sts_reg_m - pointer to magnetometer status data
 * 	int_src_reg_m - pointer to magnetometer status regdata
 *
 * Returned Values:
 *   On success returns 0. On Error returns -1
 *
 ****************************************************************************/

int nineax_lsm9ds1_read_status_mag(uint8_t * sts_reg_m, uint8_t * int_src_reg_m)
{
  int ret = OK;
  lsm9ds1_mag_status_t sts_data;

  ret = ioctl(sensor.fd, LSM9DS1_IOC_READ_STATUS_MAG, (unsigned int)&sts_data);
  if (ret < 0)
    {
      return ERROR;
    }

  lldbg("Status data: 0x%02X, 0x%02X\n", sts_data.int_src_reg_m,
        sts_data.sts_reg_m);

  *sts_reg_m = sts_data.sts_reg_m;
  *int_src_reg_m = sts_data.int_src_reg_m;

  return ret;
}

/****************************************************************************
 * Name: nineax_lsm9ds1_read_gyro
 *
 * Description:
 *  Reads gyro-sensor
 *
 * Input Parameters:
 * 	raw_data - pointer to raw_data array (size of the array is 3. The very
 * 	first element is always X and the last is Z)
 *
 * Returned Values:
 *   On success returns 0. On Error returns -1
 *
 ****************************************************************************/

int nineax_lsm9ds1_read_gyro(uint16_t * raw_data)
{
  int ret = OK;
  lsm9ds1_raw_data_t data;

  ret = ioctl(sensor.fd, LSM9DS1_IOC_READ_RAW_GYRO, (unsigned int)&data);
  if (ret < 0)
    {
      return ERROR;
    }
  raw_data[0] = data.out_x;
  raw_data[1] = data.out_y;
  raw_data[2] = data.out_z;

  return ret;
}

/****************************************************************************
 * Name: nineax_lsm9ds1_read_xl
 *
 * Description:
 *  Reads accelerometer-sensor
 *
 * Input Parameters:
 * 	raw_data - pointer to raw_data array (size of the array is 3. The very
 * 	first element is always X and the last is Z)
 *
 * Returned Values:
 *   On success returns 0. On Error returns -1
 *
 ****************************************************************************/

int nineax_lsm9ds1_read_xl(uint16_t * raw_data)
{
  int ret = OK;
  lsm9ds1_raw_data_t data;

  ret = ioctl(sensor.fd, LSM9DS1_IOC_READ_RAW_XL, (unsigned int)&data);
  if (ret < 0)
    {
      return ERROR;
    }
  raw_data[0] = data.out_x;
  raw_data[1] = data.out_y;
  raw_data[2] = data.out_z;

  return ret;
}

/****************************************************************************
 * Name: nineax_lsm9ds1_read_status_gyro
 *
 * Description:
 *  Reads gyro-sensor's status
 *
 * Input Parameters:
 * 	fifo_sts - pointer to fifo status value
 * 	int_gyro_sts - pointer to interrupt status value
 *	int_xl_sts - pointer to interrupt status value
 * 	status_reg_data - pointer to status register data value
 *
 * Returned Values:
 *   On success returns 0. On Error returns -1
 *
 ****************************************************************************/

int nineax_lsm9ds1_read_status_gyro(uint8_t * fifo_sts, uint8_t * int_gyro_sts,
                                    uint8_t * int_xl_sts,
                                    uint8_t * status_reg_data)
{
  int ret = OK;
  lsm9ds1_gyro_status_t int_info;

  ret = ioctl(sensor.fd, LSM9DS1_IOC_READ_STATUS_GYRO, (unsigned int)&int_info);
  if (ret < 0)
    {
      return ERROR;
    }
  *fifo_sts = int_info.fifo_src_data;
  *int_gyro_sts = int_info.int_gyro_sts_data;
  *int_xl_sts = int_info.int_xl_sts_data;
  *status_reg_data = int_info.status_reg_data;

  lldbg("Got values: 0x%02X 0x%02X 0x%02X 0x%02X\n", *fifo_sts, *int_gyro_sts,
        *int_xl_sts, *status_reg_data);

  return ret;
}

/****************************************************************************
 * Name: nineax_lsm9ds1_wait_for_sensor
 *
 * Description:
 *  Waits for events
 *
 * Input Parameters:
 *
 * Returned Values:
 *   On success returns 0. On Error returns -1
 *
 ****************************************************************************/

int nineax_lsm9ds1_wait_for_sensor(void)
{
  int ret = OK;
  struct pollfd fds = {
    .fd = sensor.fd,
    .events = POLLIN
  };

  /* All set, block */
  ret = poll(&fds, 1, -1);
  if (ret == -1)
    {
      perror("poll");
      return ERROR;
    }

  if (!ret)
    {
      set_errno(ETIMEDOUT);
      return ERROR;
    }

  return ret;
}

/****************************************************************************
 * Name: nineax_lsm9ds1_get_fd
 *
 * Description:
 *  Export sensor fd
 *
 * Input Parameters:
 *
 * Returned Values:
 *   Open file descriptor, or -1 if sensor is not open.
 *
 ****************************************************************************/

int nineax_lsm9ds1_get_fd(void)
{
  return sensor.fd;
}

/****************************************************************************
 * Name: nineax_lsm9ds1_read_bias
 *
 * Description:
 *  Reads sensor-combo calibration biases
 *
 * Output Parameters:
 *
 * Returned Values:
 *   On success returns 0. On Error returns -1
 *
 ****************************************************************************/

int nineax_lsm9ds1_read_bias(int16_t bias[static 9])
{
  int ret = OK;
  lsm9ds1_sensor_bias_t b;

  ret = ioctl(sensor.fd, LSM9DS1_IOC_READ_BIAS, (unsigned int)&b);
  if (ret < 0)
    {
      return ERROR;
    }

  memcpy(bias, &b, sizeof(b));

  lldbg("Read bias data: [%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd]\n",
        b.gyro_bias[0], b.gyro_bias[1], b.gyro_bias[2],
        b.xl_bias[0], b.xl_bias[1], b.xl_bias[2],
        b.mag_bias[0], b.mag_bias[1], b.mag_bias[2]);

  return ret;
}

/****************************************************************************
 * Name: nineax_lsm9ds1_write_bias
 *
 * Description:
 *  Write sensor-combo calibration biases
 *
 * Output Parameters:
 *
 * Returned Values:
 *   On success returns 0. On Error returns -1
 *
 ****************************************************************************/

int nineax_lsm9ds1_write_bias(int16_t bias[static 9])
{
  int ret;
  lsm9ds1_sensor_bias_t b;

  memcpy(&b, bias, sizeof(b));
  ret = ioctl(sensor.fd, LSM9DS1_IOC_WRITE_BIAS, (unsigned int)&b);
  if (ret < 0)
    {
      return ERROR;
    }

  lldbg("Wrote bias data: [%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd]\n",
        b.gyro_bias[0], b.gyro_bias[1], b.gyro_bias[2],
        b.xl_bias[0], b.xl_bias[1], b.xl_bias[2],
        b.mag_bias[0], b.mag_bias[1], b.mag_bias[2]);

  return ret;
}

/****************************************************************************
 * Name: nineax_lsm9ds1_load_bias_from_eeprom
 *
 * Description:
 *  Read sensor-combo calibration biases from EEPROM, and if they are valid,
 *  write them to sensor device.
 *
 * Output Parameters:
 *   bias, values from EEPROM
 *
 * Returned Values:
 *   On success returns 0. On Error returns -1
 *
 ****************************************************************************/

int nineax_lsm9ds1_load_bias_from_eeprom(int16_t bias[static 9])
{
  int ret;
  char buf[TS_DEVICE_PROD_DATA_LSM9DS1_REF_SIZE];

  ret = ts_device_prod_data_get_entry("lsm9ds1_ref_bias", buf, sizeof(buf));
  if (!ret)
    {
      lldbg("Cannot read bias, ret = %d\n", ret);
      return ERROR;
    }

  ret = sscanf(buf, "[%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd]",
               &bias[0], &bias[1], &bias[2], &bias[3], &bias[4], &bias[5],
               &bias[6], &bias[7], &bias[8]);
  if (ret != 9)
    {
      lldbg("Bad bias value in eeprom, ret = %d\n", ret);
      return ERROR;
    }

  ret = nineax_lsm9ds1_write_bias(bias);
  if (ret != OK)
    {
      lldbg("Cannot write bias, ret = %d\n", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: nineax_lsm9ds1_read_all
 *
 * Description:
 *  Reads all sensor-combo values
 *
 * Output Parameters:
 *
 * Returned Values:
 *   On success returns 0. On Error returns -1
 *
 ****************************************************************************/

int nineax_lsm9ds1_read_all(int16_t data[static 9])
{
  int ret;
  const size_t size = 9 * sizeof(int16_t);

  ret = read(sensor.fd, (char *)data, size);
  if (ret != size)
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: nineax_lsm9ds1_read_resolutions
 *
 * Description:
 *  Reads resolutions for all sensors.
 *
 * Output Parameters:
 *
 * Returned Values:
 *   On success returns 0. On Error returns -1
 *
 ****************************************************************************/

int nineax_lsm9ds1_read_resolutions(float *gyro_reso, float *xl_reso,
                                    float *mag_reso)
{
  int ret = OK;

  *gyro_reso = sensor.gyro_reso;
  *xl_reso = sensor.xl_reso;
  *mag_reso = sensor.mag_reso;

  return ret;
}

/****************************************************************************
 * Name: nineax_lsm9ds1_read_temperature
 *
 * Description:
 *  Reads temperature from sensor
 *
 * Input Parameters:
 *      int_temper - pointer to converted temperature. User must divide by
 *                   LSM9DS1_TEMPERATURE_PRECISION to get degrees of Celsius.
 * 	raw_temper - pointer to read temperature value in device format
 *
 * Returned Values:
 *   On success returns 0. On Error returns -1
 *
 ****************************************************************************/

int nineax_lsm9ds1_read_temperature(int32_t * int_temper, int16_t * raw_temper)
{
  int ret = OK;

  ret = ioctl(sensor.fd, LSM9DS1_IOC_READ_TEMP, (unsigned int)raw_temper);
  if (ret < 0)
    {
      return ERROR;
    }

  /* 12-bit value, max. range -2048..+2047. Sensitivity according to datasheet
   * is quoted at 16 LSB/⁰C (Typ.), so we use that. Measurements indicate
   * that higher divisor like 32 would give more plausible values than ST's
   * "typical" ambiguous specification. TODO: investigate further. Zero raw
   * value = 25 ⁰C. */

  *int_temper = *raw_temper * LSM9DS1_TEMPERATURE_PRECISION / 16 +
    25 * LSM9DS1_TEMPERATURE_PRECISION;

  return ret;
}

/****************************************************************************
 * Name: nineax_lsm9ds1_stop
 *
 * Description:
 * 	Does operations needed to properly close 9-axels sensor
 *
 * Input Parameters:
 *
 * Returned Values:
 *   On success returns 0. On Error returns -1
 *
 ****************************************************************************/

int nineax_lsm9ds1_stop(void)
{
  int ret;

  ret = close(sensor.fd);
  sensor.fd = -1;

  return ret;
}

