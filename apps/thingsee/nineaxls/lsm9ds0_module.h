/****************************************************************************
 * apps/thingsee/nineaxls/lsm9ds0_module.h
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

#ifndef LSM9DS0_MODULE_H_
#define LSM9DS0_MODULE_H_

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

int nineax_lsm9ds0_who_am_i(uint8_t * gyro_id, uint8_t * acc_mag_id);

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

int nineax_lsm9ds0_start(void);

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

int nineax_lsm9ds0_stop(void);

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

int nineax_lsm9ds0_config_gyro(void);

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

int nineax_lsm9ds0_self_test_gyro(void);

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

int nineax_lsm9ds0_read_gyro(uint16_t * raw_data);

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

int nineax_lsm9ds0_wait_for_sensor(void);

/****************************************************************************
 * Name: nineax_lsm9ds0_reset_gyro_fifo
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

int nineax_lsm9ds0_reset_gyro_fifo(bool first_time);

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

int nineax_lsm9ds0_reset_xm_fifo(bool first_time);

/****************************************************************************
 * Name: nineax_lsm9ds0_get_status
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
                                uint8_t * status_reg_data);

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

int nineax_lsm9ds0_config_xm(void);

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

int nineax_lsm9ds0_read_xm(uint16_t * magn_data_xyz, uint16_t * acc_data_xyz);

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
                                  uint8_t * int_gen_2_src);

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

int nineax_lsm9ds0_self_test_xm(void);

#endif /* LSM9DS0_MODULE_H_ */
