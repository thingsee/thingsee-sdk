/****************************************************************************
 * include/nuttx/input/lp5521.c
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
 *   Author: Jussi Kivilinna <jussi.kivilinna@haltian.com>
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

#ifndef __INCLUDE_NUTTX_LED_LP5521_H_
#define __INCLUDE_NUTTX_LED_LP5521_H_

#include <nuttx/compiler.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdbool.h>

/* Controller configuration for LP5521 device */

enum lp5521_clock_select_e
{
  LP5521_CLOCK_EXTERNAL_CLK_32K = 0,
  LP5521_CLOCK_INTERNAL,
  LP5521_CLOCK_AUTOMATIC_SELECTION
};

enum lp5521_charge_pump_mode_e
{
  LP5521_CHARGE_PUMP_MODE_OFF = 0,
  LP5521_CHARGE_PUMP_MODE_FORCED_1X,
  LP5521_CHARGE_PUMP_MODE_FORCED_1_5X,
  LP5521_CHARGE_PUMP_MODE_AUTOMATIC_SELECTION
};

enum lp5521_channel_mode_e
{
  LP5521_CHANNEL_MODE_NO_CHANGE = -1,
  LP5521_CHANNEL_MODE_DISABLED,
  LP5521_CHANNEL_MODE_RUN_PROGRAM,
  LP5521_CHANNEL_MODE_DIRECT_CONTROL
};

struct lp5521_conf_s
{
  struct
  {
    enum lp5521_channel_mode_e initial_mode;   /* Initial mode for channel. */
    uint8_t initial_pwm;                       /* Valid: 0 to 255. */
    int16_t current;                           /* Valid: 0 to 25500 µA.
                                                * Use default if -1. */
  } r, g, b;

  bool logarithmic:1;       /* Enable logarithmic PWM adjustment. */
  bool auto_powersave:1;    /* Enable automatic power saving. */
  bool pwm_clock_558hz:1;   /* Use 558 Hz PWM clock (internal osc),
                             * if false use 256 Hz (CLK_32K). */
  bool r_channel_to_batt:1; /* R channel supply connection from
                             * battery instead of charge pump. */

  /* Controller clock: External/Internal/Auto-detection. */

  enum lp5521_clock_select_e clock_select:2;

  /* Charge pump operation mode. */

  enum lp5521_charge_pump_mode_e charge_pump_mode:2;
} packed_struct;

/* Write commands to LP5521 driver. */

enum lp5521_cmd_e
{
  LED_LP5521_CMD_SET_PWM = -9,
  LED_LP5521_CMD_SET_MODE,
  LED_LP5521_CMD_LOAD_PROGRAM,
  LED_LP5521_CMD_RECONFIGURE
};

/* LED_LP5521_CMD_SET_PWM command structure. */

struct lp5521_cmd_pwm_s
{
  enum lp5521_cmd_e type;

  /* PWM values for channels R/G/B.
   * Valid values: 0 to 255, and -1. Old PWM value kept if new value -1. */

  int16_t r, g, b;
} packed_struct;

/* LED_LP5521_CMD_SET_MODE command structure. */

struct lp5521_cmd_mode_s
{
  enum lp5521_cmd_e type;
  enum lp5521_channel_mode_e r, g, b;      /* Modes for channels. */
} packed_struct;

/* LED_LP5521_CMD_RECONFIGURE command structure. */

struct lp5521_cmd_reconfig_s
{
  enum lp5521_cmd_e type;
  struct lp5521_conf_s config;
} packed_struct;

/* Board configuration */

struct lp5521_board_s
{
  int (*set_power) (FAR const struct lp5521_board_s *state, bool on);
  int (*set_enable) (FAR const struct lp5521_board_s *state, bool enable);
};

/* Device registration */

int led_lp5521_register(FAR const char *devpath,
                        FAR struct i2c_dev_s *i2c_bus,
                        uint8_t i2c_devaddr,
                        FAR const struct lp5521_board_s *board_config,
                        FAR const struct lp5521_conf_s *chip_config);

#endif /* __INCLUDE_NUTTX_LED_LP5521_H_ */
