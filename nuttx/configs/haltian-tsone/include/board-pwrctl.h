/************************************************************************************
 * configs/haltian-tsone/include/board-pwrctl.h
 * include/arch/board/board-pwrctl.h
 *
 *   Copyright (C) 2014-2015 Haltian Ltd. All rights reserved.
 *
 * Authors:
 *   Jussi Kivilinna <jussi.kivilinna@haltian.com>
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
 ************************************************************************************/

#ifndef __BOARD_PWRCTL_H__
#define __BOARD_PWRCTL_H__

/************************************************************************************
 * Included Files
 ************************************************************************************/

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Various devices on the board are powered by separate regulators and
 * power-switches for power-management. Some of these share same
 * pin/power-line. */

enum e_board_pwrctl {
  PWRCTL_REGULATOR_GPS = 0,
  PWRCTL_REGULATOR_WLAN,
  PWRCTL_REGULATOR_SDCARD,
  PWRCTL_REGULATOR_BLUETOOTH,
  PWRCTL_REGULATOR_DISPLAY,
  PWRCTL_REGULATOR_FLASH,
  PWRCTL_REGULATOR_EXTRA,
  PWRCTL_SWITCH_MODEM,
  PWRCTL_SWITCH_CAPSENSE_SENSOR,
  PWRCTL_SWITCH_9AXIS_INERTIAL_SENSOR,
  PWRCTL_SWITCH_ADC_VBAT_MEASUREMENT,
};

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/* Initialize power-control/regulator GPIO pins and set initial output low. */
void board_initialize_pwrctl_pins(void);

/* Get power-control for use. Powers on the power-control/regulator when
 * necessary. */
void board_pwrctl_get(enum e_board_pwrctl pwrctl_id);

/* Put power-control from use. Powers down the power-control when no users
 * left. */
void board_pwrctl_put(enum e_board_pwrctl pwrctl_id);

/* Force all power-control gpios to power-off state. */
void board_pwrctl_all_off(void);

#endif /*__BOARD_PWRCTL_H__*/
