/****************************************************************************
 *
 *   Copyright (C) 2014-2015 Haltian Ltd. All rights reserved.
 *
 * Authors:
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
 ****************************************************************************/

#ifndef BOARD_BT_H_
#define BOARD_BT_H_

/****************************************************************************
 * Name: board_bt_power_device_up
 *
 * Description:
 *   Powers BT device ON/OFF
 *
 * Input Parameters:
 *   enable:
 *     		true -  Powers device ON if not already powered ON
 *     		false - Powers device down if not already powered down
 *
 * Returned Values:
 *
 ****************************************************************************/

void board_bt_power_device_up(bool enable);

/****************************************************************************
 * Name: board_bt_init_dev
 *
 * Description:
 *   Opens BT's file descriptor for further use
 *
 * Input Parameters:
 *
 * Returned Values:
 * 	 On success returns file descriptor. On failure an ERROR, which, I guess,
 * 	 is equal to -1
 *
 ****************************************************************************/

int board_bt_init_dev(void);

/****************************************************************************
 * Name: board_bt_deinit_dev
 *
 * Description:
 *   Closes BT's file descriptor
 *
 * Input Parameters:
 * 	 fd - file descriptor opened earlier
 *
 * Returned Values:
 *
 ****************************************************************************/

int board_bt_deinit_dev(int fd);

/****************************************************************************
 * Name: board_bt_hw_reset
 *
 * Description:
 *   Executes HW-reset
 *
 * Input Parameters:
 *
 * Returned Values:
 * 	 On success returns OK. If error occurred, sets errno to ETIMEDOUT
 *
 ****************************************************************************/

void board_bt_hw_reset(void);

/****************************************************************************
 * Name: board_bt_change_baud_rate
 *
 * Description:
 *   Changes baud-rate
 *
 * Input Parameters:
 * 		baud_rate - baud rate to apply
 * 		fd - file descriptor opened earlier
 *
 * Returned Values:
 * 		On success returns 0. On Error -1
 *
 ****************************************************************************/

int board_bt_change_baud_rate(unsigned int baud_rate, int fd);

/****************************************************************************
 * Name: board_bt_backdoor
 *
 * Description:
 *   Enable firmware updating
 *
 * Input Parameters:
 * 		enable - boolean to enable backdoor
 *
 * Returned Values:
 * 		None
 *
 ****************************************************************************/


void board_bt_backdoor(bool enable);
#endif /* BOARD_BT_H_ */
