/****************************************************************************
 * apps/thingsee/nineaxls/lsm9ds0_module.c
 *
 *   Copyright (C) 2014-2015 Haltian Ltd. All rights reserved.
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

#include <nuttx/sensors/lsm9ds0.h>

#include "lsm9ds0_module.h"

typedef struct lsm9ds0_dev_t
  {
    int fd;
    lsm9ds0_fifo_mode_t fifo_mode_g;
    uint8_t fifo_samples_nbr_g;
    lsm9ds0_fifo_mode_t fifo_mode_xm;
    uint8_t fifo_samples_nbr_xm;
  } lsm9ds0_dev_t;

static lsm9ds0_dev_t sensor;

/****************************************************************************
 * Name: nineax_lsm9ds0_reset_fifo
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

int nineax_lsm9ds0_reset_gyro_fifo(bool first_time)
{
  int ret = OK;
  lsm9ds0_gyro_config_data_t config_data = {
    .fifo_mode = LSM9DS0_FIFO_BYPASS,
    .fifo_samples_nbr = 0
  };

  ret = ioctl(sensor.fd, LSM9DS0_IOC_RST_FIFO_GYRO, (unsigned int)&config_data);
  if (ret < 0)
    {
      return ERROR;
    }

  if (!first_time)
    {
      config_data.fifo_mode = sensor.fifo_mode_g;
      config_data.fifo_samples_nbr = sensor.fifo_samples_nbr_g;
      ret = ioctl(sensor.fd, LSM9DS0_IOC_RST_FIFO_GYRO, (unsigned int)&config_data);
    }

  return ret;
}

/****************************************************************************
 * Name: nineax_lsm9ds0_reset_xm_fifo
 *
 * Description:
 *  Resets FIFO for acc/magn
 *
 * Input Parameters:
 * 	first_time - first time initialization reset
 *
 * Returned Values:
 *   On success returns 0. On Error returns -1
 *
 ****************************************************************************/

int nineax_lsm9ds0_reset_xm_fifo(bool first_time)
{
  int ret = OK;
  lsm9ds0_gyro_config_data_t config_data = {
    .fifo_mode = LSM9DS0_FIFO_BYPASS,
    .fifo_samples_nbr = 0
  };

  ret = ioctl(sensor.fd, LSM9DS0_IOC_RST_FIFO_ACC_MAGN, (unsigned int)&config_data);
  if (ret < 0)
    {
      return ERROR;
    }

  if (!first_time)
    {
      config_data.fifo_mode = sensor.fifo_mode_g;
      config_data.fifo_samples_nbr = sensor.fifo_samples_nbr_g;
      ret = ioctl(sensor.fd, LSM9DS0_IOC_RST_FIFO_ACC_MAGN,
                  (unsigned int)&config_data);
    }

  return ret;
}

/****************************************************************************
 * Name: nineax_lsm9ds0_who_am_i
 *
 * Description:
 *  Gets sensors' ids
 *
 * Input Parameters:
 * 	gyro_id - gyro-sensor's id. Should return 0xD4
 * 	acc_mag_id - accelerometer/magnetometer's id. Should return 0x49
 *
 * Returned Values:
 *   On success returns 0. On Error returns -1
 *
 ****************************************************************************/

int nineax_lsm9ds0_who_am_i(uint8_t * gyro_id, uint8_t * acc_mag_id)
{
  int ret = OK;
  lsm9ds0_who_am_i_t who_am_i;

  ret = ioctl(sensor.fd, LSM9DS0_IOC_WHO_AM_I, (unsigned int)&who_am_i);
  if (ret < 0)
    {
      return ERROR;
    }

  *gyro_id = who_am_i.gyro;
  *acc_mag_id = who_am_i.acc_magn;

  return ret;
}

/****************************************************************************
 * Name: nineax_lsm9ds0_start
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

int nineax_lsm9ds0_start(void)
{
  int ret = OK;

  sensor.fd = open("/dev/lsm9ds0", O_RDWR);
  if (sensor.fd < 0)
    {
      perror("Cannot open file");
      return ERROR;
    }

  return ret;
}

/****************************************************************************
 * Name: nineax_lsm9ds0_config_gyro
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

int nineax_lsm9ds0_config_gyro(void)
{
  int ret = OK;
  uint8_t fifo_sts = 0;
  uint8_t int1_sts = 0;
  uint8_t status_reg = 0;
  lsm9ds0_gyro_config_data_t config_data = {
    .axels_mask = LSM9DS0_GYRO_AXEL_YEN | LSM9DS0_GYRO_AXEL_ZEN | LSM9DS0_GYRO_AXEL_XEN,
    .cutoff = LSM9DS0_GYRO_CUTOFF_01,
    .odr = LSM9DS0_GYRO_ODR_380HZ,
    .state = LSM9DS0_GYRO_STATE_NORMAL,
    .hpm = LSM9DS0_GYRO_HPM_NORMAL_MODE,
    .hpcf = LSM9DS0_GYRO_HPCF1001,
    .fs = LSM9DS0_GYRO_FS_2000DPS,
    .fifo_mode = LSM9DS0_FIFO_STREAMTOFIFO,
    .fifo_samples_nbr = 0x1F,
    .is_reboot = true,
    .fifo_en = true,
    .hpen = true,
    .bdu = true,
    .latch_int = true,
    .is_high = true,
    .threshold_int = true,
    .watermark_int = true,
    .x_threshold = 0x7F,
    .y_threshold = 0x7F,
    .z_threshold = 0xFF,
    .interrupt_duration = 0
  };

  ret = nineax_lsm9ds0_reset_gyro_fifo(true);
  if (ret < 0)
    {
      return ERROR;
    }

  ret = nineax_lsm9ds0_get_status_g(&fifo_sts, &int1_sts, &status_reg);
  if (ret < 0)
    {
      return ERROR;
    }

  ret = ioctl(sensor.fd, LSM9DS0_IOC_CONFIG_GYRO, (unsigned int)&config_data);
  if (ret < 0)
    {
      return ERROR;
    }
  sensor.fifo_mode_g = config_data.fifo_mode;
  sensor.fifo_samples_nbr_g = config_data.fifo_samples_nbr;

  return ret;
}

/****************************************************************************
 * Name: nineax_lsm9ds0_self_test_gyro
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

int nineax_lsm9ds0_self_test_gyro(void)
{
  int ret = OK;
  uint16_t raw_data[3] = { 0 };
  lsm9ds0_gyro_config_data_t config_data = {
    .axels_mask = LSM9DS0_GYRO_AXEL_YEN | LSM9DS0_GYRO_AXEL_ZEN | LSM9DS0_GYRO_AXEL_XEN,
    .state = LSM9DS0_GYRO_STATE_NORMAL,
    .st = LSM9DS0_GYRO_ST_0,
    .fifo_en = false,
    .hpen = false,
    .bdu = false,
    .latch_int = false,
    .is_high = false,
    .threshold_int = false,
    .watermark_int = false,
    .fs = LSM9DS0_GYRO_FS_245DPS,
    .odr = LSM9DS0_GYRO_ODR_380HZ,
  };

  ret = ioctl(sensor.fd, LSM9DS0_IOC_CONFIG_GYRO, (unsigned int)&config_data);
  if (ret < 0)
    {
      return ERROR;
    }

  ret = nineax_lsm9ds0_read_gyro(raw_data);

  lldbg("First self-test gyro data: 0x%04X, 0x%04X, 0x%04X\n", raw_data[0],
        raw_data[1], raw_data[2]);

  config_data.st = LSM9DS0_GYRO_ST_1;

  ret = ioctl(sensor.fd, LSM9DS0_IOC_CONFIG_GYRO, (unsigned int)&config_data);
  if (ret < 0)
    {
      return ERROR;
    }

  ret = nineax_lsm9ds0_read_gyro(raw_data);

  lldbg("Second self-test gyro data: 0x%04X, 0x%04X, 0x%04X\n", raw_data[0],
        raw_data[1], raw_data[2]);

  return ret;
}

/****************************************************************************
 * Name: nineax_lsm9ds0_config_xm
 *
 * Description:
 *  Setups accelerometer/magnetometer-sensor
 *
 * Input Parameters:
 *
 * Returned Values:
 *   On success returns 0. On Error returns -1
 *
 ****************************************************************************/

int nineax_lsm9ds0_config_xm(void)
{
  int ret = OK;
  uint16_t magn_offset_xyz[3] = { 0 };

  lsm9ds0_xm_config_data_t config_data = {
    .int_maxels_mask =
      LSM9DS0_MAGN_AXEL_ZIEN | LSM9DS0_MAGN_AXEL_YIEN | LSM9DS0_MAGN_AXEL_XIEN,
    .int_high_active = true,
    .int_latch = false,
    .int_magn_en = true,
    .four_D = true,
    .magn_threshold = 0x0,
    .magn_offset_xyz = magn_offset_xyz,
    .fifo_en = true,
    .fifo_samples_nbr = 0x0A,
    .wtm_en = true,
    .bdu = true,
    .acc_ax_mask =
      LSM9DS0_ACC_AXEL_XEN | LSM9DS0_ACC_AXEL_YEN | LSM9DS0_ACC_AXEL_ZEN,
    .aodr = LSM9DS0_ACC_AODR_3_125HZ,
    .acc_aa_bw = LSM9DS0_ACC_ABW_50HZ,
    .acc_fullscale = LSM9DS0_ACC_AFS_16G,
    .p_int_magn = true,
    .p_int_inertial1 = true,
    .p_int_inertial2 = true,
    .temp_en = false,
    .magn_high_res = true,
    .magn_odr = LSM9DS0_M_ODR_3_125HZ,
    .lir1 = false,
    .lir2 = false,
    .mfs = LSM9DS0_MFS_8GAUSS,
    .acc_ahpm = LSM9DS0_AHPM_NORMAL_MODE,
    .md1 = LSM9DS0_MD1_CONT_CONV,
    .fifo_mode = LSM9DS0_FIFO_STREAMTOFIFO,
    .six_D1 = true,
    .aoi1 = false,
    .six_D2 = true,
    .aoi2 = true,
    .int1_gen_high = true,
    .int1_gen_duration = 0,
    .int1_gen_ths = 0,
    .int2_gen_high = true,
    .int2_gen_duration = 0,
    .int2_gen_ths = 0,
  };

  sensor.fifo_samples_nbr_xm = config_data.fifo_samples_nbr;
  sensor.fifo_mode_xm = config_data.fifo_mode;

  ret = nineax_lsm9ds0_reset_xm_fifo(true);
  if (ret < 0)
    {
      return ERROR;
    }

  ret = ioctl(sensor.fd, LSM9DS0_IOC_CONFIG_ACC_MAGN, (unsigned int)&config_data);
  if (ret < 0)
    {
      return ERROR;
    }

  return ret;
}

/****************************************************************************
 * Name: nineax_lsm9ds0_self_test_xm
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

int nineax_lsm9ds0_self_test_xm(void)
{
  int ret = OK;
  uint16_t magn_data[3] = { 0 };
  uint16_t acc_data[3] = { 0 };

  lsm9ds0_xm_config_data_t config_data = {
    .int_high_active = false,
    .int_latch = false,
    .int_magn_en = false,
    .four_D = false,
    .magn_threshold = 0x0,
    .fifo_en = false,
    .wtm_en = false,
    .bdu = false,
    .p_int_magn = false,
    .p_int_inertial1 = false,
    .p_int_inertial2 = false,
    .temp_en = false,
    .lir1 = false,
    .lir2 = false,
    .acc_ax_mask =
      LSM9DS0_ACC_AXEL_XEN | LSM9DS0_ACC_AXEL_YEN | LSM9DS0_ACC_AXEL_ZEN,
    .six_D1 = false,
    .aoi1 = false,
    .six_D2 = false,
    .aoi2 = false,
    .int1_gen_high = false,
    .int1_gen_duration = 0,
    .int1_gen_ths = 0,
    .int2_gen_high = false,
    .int2_gen_duration = 0,
    .int2_gen_ths = 0,
  };

  ret = ioctl(sensor.fd, LSM9DS0_IOC_CONFIG_ACC_MAGN, (unsigned int)&config_data);
  if (ret < 0)
    {
      return ERROR;
    }

  ret = nineax_lsm9ds0_read_xm(magn_data, acc_data);
  if (ret < 0)
    {
      return ERROR;
    }

  lldbg("Positive magn: 0x%04X, 0x%04X, 0x%04X acc: 0x%04X, 0x%04X, 0x%04X\n",
        magn_data[0], magn_data[1], magn_data[2], acc_data[0], acc_data[1],
        acc_data[2]);

  config_data.s_test_mode = LSM9DS0_XM_STST_NEG_SIGN_TST;

  ret = ioctl(sensor.fd, LSM9DS0_IOC_CONFIG_ACC_MAGN, (unsigned int)&config_data);
  if (ret < 0)
    {
      return ERROR;
    }

  ret = nineax_lsm9ds0_read_xm(magn_data, acc_data);
  if (ret < 0)
    {
      return ERROR;
    }

  lldbg("Negative magn: 0x%04X, 0x%04X, 0x%04X acc: 0x%04X, 0x%04X, 0x%04X\n",
        magn_data[0], magn_data[1], magn_data[2], acc_data[0], acc_data[1],
        acc_data[2]);

  return ret;
}

/****************************************************************************
 * Name: nineax_lsm9ds0_read_xm
 *
 * Description:
 *  Reads xm-sensor
 *
 * Input Parameters:
 * 	magn_data_xyz - pointer to raw_data array (size of the array is 3. The very
 * 	first element is always X and the last is Z)
 * 	acc_data_xyz - pointer to raw_data array (size of the array is 3. The very
 * 	first element is always X and the last is Z)
 *
 * Returned Values:
 *   On success returns 0. On Error returns -1
 *
 ****************************************************************************/

int nineax_lsm9ds0_read_xm(uint16_t * magn_data_xyz, uint16_t * acc_data_xyz)
{
  int ret = OK;
  lsm9ds0_acc_magn_raw_data_t acc_magn_raw = {
    .magn_xyz = magn_data_xyz,
    .acc_xyz = acc_data_xyz,
  };

  ret = ioctl(sensor.fd, LSM9DS0_IOC_READ_RAW_ACC_MAGN,
              (unsigned int)&acc_magn_raw);

  return ret;
}

/****************************************************************************
 * Name: nineax_lsm9ds0_read_status_xm
 *
 * Description:
 *  Reads xm-sensor status
 *
 * Input Parameters:
 * 	sts_reg_m - pointer to magnetometer status data
 * 	int_src_reg_m - pointer to magnetometer status regdata
 * 	sts_reg_a - pointer to accelerometer status data
 * 	int_gen_1_src - pointer 1 to accelerometer gen status data
 * 	int_gen_2_src - pointer 2 to accelerometer gen status data
 *
 * Returned Values:
 *   On success returns 0. On Error returns -1
 *
 ****************************************************************************/

int nineax_lsm9ds0_read_status_xm(uint8_t * sts_reg_m, uint8_t * int_src_reg_m,
                                  uint8_t * sts_reg_a, uint8_t * int_gen_1_src,
                                  uint8_t * int_gen_2_src)
{
  int ret = OK;
  lsm9ds0_acc_magn_sts_data_t sts_data;

  ret = ioctl(sensor.fd, LSM9DS0_IOC_READ_STS_BITS_ACC_MAGN,
              (unsigned int)&sts_data);
  if (ret < 0)
    {
      return ERROR;
    }

  lldbg("Status data: 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
        sts_data.int_src_reg_m, sts_data.int_gen_1_src, sts_data.int_gen_2_src,
        sts_data.sts_reg_m, sts_data.sts_reg_a);

  *sts_reg_m = sts_data.sts_reg_m;
  *int_src_reg_m = sts_data.int_src_reg_m;
  *sts_reg_a = sts_data.sts_reg_a;
  *int_gen_1_src = sts_data.int_gen_1_src;
  *int_gen_2_src = sts_data.int_gen_2_src;

  return ret;
}

/****************************************************************************
 * Name: nineax_lsm9ds0_read_gyro
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

int nineax_lsm9ds0_read_gyro(uint16_t * raw_data)
{
  int ret = OK;
  lsm9ds0_gyro_raw_data_t data;

  ret = ioctl(sensor.fd, LSM9DS0_IOC_READ_RAW_GYRO, (unsigned int)&data);
  if (ret < 0)
    {
      return ERROR;
    }
  raw_data[0] = data.out_x_g;
  raw_data[1] = data.out_y_g;
  raw_data[2] = data.out_z_g;

  return ret;
}

/****************************************************************************
 * Name: nineax_lsm9ds0_get_status_g
 *
 * Description:
 *  Reads gyro-sensor's status
 *
 * Input Parameters:
 * 	fifo_sts - pointer to fifo status value
 * 	int_sts - pointer to interrupt status value
 * 	status_reg_data - pointer to status register data value
 *
 * Returned Values:
 *   On success returns 0. On Error returns -1
 *
 ****************************************************************************/

int nineax_lsm9ds0_get_status_g(uint8_t * fifo_sts, uint8_t * int_sts,
                                uint8_t * status_reg_data)
{
  int ret = OK;
  lsm9ds0_gyro_int_info_t int_info;

  ret = ioctl(sensor.fd, LSM9DS0_IOC_READ_STS_BITS_GYRO, (unsigned int)&int_info);
  if (ret < 0)
    {
      return ERROR;
    }
  *fifo_sts = int_info.fifo_src_data;
  *int_sts = int_info.int1_sts_data;
  *status_reg_data = int_info.status_reg_data;

  lldbg("Got values: 0x%02X  0x%02X  0x%02X\n", *fifo_sts, *int_sts,
        *status_reg_data);

  return ret;
}

/****************************************************************************
 * Name: nineax_lsm9ds0_wait_for_sensor
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

int nineax_lsm9ds0_wait_for_sensor(void)
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
      perror("Cannot poll for some reasons");
      return ERROR;
    }

  if (!ret)
    {
      set_errno(ETIMEDOUT);
      perror("Still not there");
      return ERROR;
    }

  return ret;
}

/****************************************************************************
 * Name: nineax_lsm9ds0_stop
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

int nineax_lsm9ds0_stop(void)
{
  int ret = OK;

  close(sensor.fd);

  return ret;
}
