/****************************************************************************
 * apps/thingsee/nineaxls/lsm9ds1_module.h
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

#ifndef LSM9DS1_MODULE_H_
#  define LSM9DS1_MODULE_H_

#  define NINEAX_MAG_VALID_ID                   0x3D
#  define NINEAX_GYRO_ACC_VALID_ID              0x68

/****************************************************************************
 * Name: nineax_lsm9ds1_who_am_i
 *
 * Description:
 *  Gets sensors' ids
 *
 * Input Parameters:
 * 	gyro_id - accelerometer/gyro-sensor's id. Should be NINEAX_GYRO_ACC_VALID_ID
 * 	mag_id - magnetometer's id. Should be NINEAX_GYRO_MAG_VALID_ID
 *
 * Returned Values:
 *   On success returns 0. On Error returns -1
 *
 ****************************************************************************/

int nineax_lsm9ds1_who_am_i(uint8_t * gyro_id, uint8_t * mag_id);

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

int nineax_lsm9ds1_start(void);

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

int nineax_lsm9ds1_stop(void);

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

int nineax_lsm9ds1_config_gyro(void);

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

int nineax_lsm9ds1_self_test_gyro(void);

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

int nineax_lsm9ds1_read_gyro(uint16_t * raw_data);

/****************************************************************************
 * Name: nineax_lsm9ds1_read_xl
 *
 * Description:
 *  Reads accelerometer-sensor
 *
 * Input Parameters:
 *      raw_data - pointer to raw_data array (size of the array is 3. The very
 *      first element is always X and the last is Z)
 *
 * Returned Values:
 *   On success returns 0. On Error returns -1
 *
 ****************************************************************************/

int nineax_lsm9ds1_read_xl(uint16_t * raw_data);

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

int nineax_lsm9ds1_wait_for_sensor(void);

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

int nineax_lsm9ds1_get_fd(void);

/****************************************************************************
 * Name: nineax_lsm9ds1_reset_fifo
 *
 * Description:
 *  Resets FIFO for gyro and accelerometer
 *
 * Input Parameters:
 * 	first_time - first time initialization reset
 *
 * Returned Values:
 *   On success returns 0. On Error returns -1
 *
 ****************************************************************************/

int nineax_lsm9ds1_reset_fifo(bool first_time);

/****************************************************************************
 * Name: nineax_lsm9ds1_read_status_gyro
 *
 * Description:
 *  Reads gyro-sensor's status
 *
 * Input Parameters:
 * 	fifo_sts - pointer to fifo status value
 * 	int_gyro_sts - pointer to interrupt status value
 * 	int_xl_sts - pointer to interrupt status value
 * 	status_reg_data - pointer to status register data value
 *
 * Returned Values:
 *   On success returns 0. On Error returns -1
 *
 ****************************************************************************/

int nineax_lsm9ds1_read_status_gyro(uint8_t * fifo_sts, uint8_t * int_gyro_sts,
                                    uint8_t * int_xl_sts,
                                    uint8_t * status_reg_data);

/****************************************************************************
 * Name: nineax_lsm9ds1_get_bias
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

int nineax_lsm9ds1_read_bias(int16_t bias[static 9]);

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

int nineax_lsm9ds1_write_bias(int16_t bias[static 9]);

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

int nineax_lsm9ds1_load_bias_from_eeprom(int16_t bias[static 9]);

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

int nineax_lsm9ds1_read_all(int16_t data[static 9]);

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
                                    float *mag_reso);

/****************************************************************************
 * Name: nineax_lsm9ds1_read_temperature
 *
 * Description:
 *  Reads temperature from sensor
 *
 * Input Parameters:
 *      int_temper - pointer to converted temperature. User must divide by
 *                   LSM9DS1_TEMPERATURE_PRECISION to get degrees of Celsius.
 *      raw_temper - pointer to read temperature value in device format
 *
 * Returned Values:
 *   On success returns 0. On Error returns -1
 *
 ****************************************************************************/

int nineax_lsm9ds1_read_temperature(int32_t * int_temper, int16_t * raw_temper);

/****************************************************************************
 * Name: nineax_lsm9ds1_config_mag
 *
 * Description:
 *  Setups magnetometer
 *
 * Input Parameters:
 *
 * Returned Values:
 *   On success returns 0. On Error returns -1
 *
 ****************************************************************************/

int nineax_lsm9ds1_config_mag(void);

/****************************************************************************
 * Name: nineax_lsm9ds1_read_mag
 *
 * Description:
 *  Reads xm-sensor
 *
 * Input Parameters:
 * 	magn_data_xyz - pointer to raw_data array (size of the array is 3. The very
 * 	first element is always X and the last is Z)
 *
 * Returned Values:
 *   On success returns 0. On Error returns -1
 *
 ****************************************************************************/

int nineax_lsm9ds1_read_mag(uint16_t * magn_data_xyz);

/****************************************************************************
 * Name: nineax_lsm9ds1_read_status_mag
 *
 * Description:
 *  Reads magnetic sensor status
 *
 * Input Parameters:
 * 	sts_reg_m - pointer to magnetometer status data
 * 	int_src_reg_m - pointer to magnetometer status regdata
 *
 * Returned Values:
 *   On success returns 0. On Error returns -1
 *
 ****************************************************************************/

int nineax_lsm9ds1_read_status_mag(uint8_t * sts_reg_m,
                                   uint8_t * int_src_reg_m);

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

int nineax_lsm9ds1_self_test_mag(void);

#endif /* LSM9DS1_MODULE_H_ */
