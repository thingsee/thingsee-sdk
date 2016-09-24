/************************************************************************************
 * configs/haltian-tsone/include/board-modem.h
 * include/arch/board/board-modem.h
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

#ifndef __CONFIG_HALTIAN_TSONE_B1_INCLUDE_BOARD_MODEM_H
#define __CONFIG_HALTIAN_TSONE_B1_INCLUDE_BOARD_MODEM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <stdbool.h>
#include <nuttx/config.h>
#include <arch/chip/chip.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* Board support implements 'board_set_modem_activity()'. */

#define BOARD_HAS_SET_MODEM_ACTIVITY    1

/* Board has TX_BURST on GPIO1? */

#define BOARD_MODEM_HAS_TX_BURST_GPIO1  1

/****************************************************************************
 * Public Type Definition
 ****************************************************************************/

enum e_board_modem_activity
{
  BOARD_MODEM_ACTIVITY_HIGH = 0,
  BOARD_MODEM_ACTIVITY_LOW,

  __BOARD_MODEM_ACTIVITY_MAX,
};

enum e_board_modem_pin_ctrl
{
  BOARD_MODEM_PIN_CTRL_ALT_MODE = 0,
  BOARD_MODEM_PIN_CTRL_LOW,
  BOARD_MODEM_PIN_CTRL_HIGH,
  BOARD_MODEM_PIN_CTRL_READ
};

enum e_board_modem_pin
{
  BOARD_MODEM_PIN_RTS = 0,
  BOARD_MODEM_PIN_CTS,
  BOARD_MODEM_PIN_TX_BURST,
};

/****************************************************************************
 * Name: board_modem_reset_pin_set
 *
 * Description:
 *   Control RESET_N gpio.
 *
 * Input Parameters:
 *   set:  enable or disable pin.
 *
 * Return value:
 *   Recommended time in milliseconds to sleep after setting pin for proper
 *   operation of modem.
 *
 ****************************************************************************/

EXTERN uint32_t board_modem_reset_pin_set(bool set);

/****************************************************************************
 * Name: board_modem_poweron_pin_set
 *
 * Description:
 *   Control POWER_ON gpio.
 *
 * Input Parameters:
 *   set:  enable or disable pin.
 *
 * Return value:
 *   Recommended time in milliseconds to sleep after setting pin for proper
 *   operation of modem.
 *
 ****************************************************************************/

EXTERN uint32_t board_modem_poweron_pin_set(bool set);

/****************************************************************************
 * Name: board_modem_vcc_set
 *
 * Description:
 *   Set modem VCC.
 *
 * Input Parameters:
 *   on:   New requested VCC on/off state.
 *
 * Return:
 *   Return new VCC state.
 *
 ****************************************************************************/

EXTERN bool board_modem_vcc_set(bool on);

/****************************************************************************
 * Name: board_modem_initialize
 *
 * Description:
 *   Initialize the u-blox modem.
 *
 * Input Parameters:
*    is_vcc_off:   Value is set 'true' if modem is fully powered off at
*                  start-up.
 *
 * Return:
 *   Opened modem serial port file descriptor.
 *
 ****************************************************************************/

EXTERN int board_modem_initialize(bool *is_vcc_off);

/****************************************************************************
 * Name: board_modem_deinitialize
 *
 * Description:
 *   Deinitialize the u-blox modem.
 *
 ****************************************************************************/

EXTERN int board_modem_deinitialize(int fd);

/****************************************************************************
 * Name: board_set_modem_activity
 *
 * Description:
 *   Change activity state, for controlling power-saving/deep-sleep activity
 *
 ****************************************************************************/

EXTERN void board_set_modem_activity(enum e_board_modem_activity type,
                                     bool active);

/****************************************************************************
 * Name: board_modem_pin_ctrl
 *
 * Description:
 *   Control modem RTS/CTS gpio.
 *
 ****************************************************************************/

EXTERN bool board_modem_pin_ctrl(enum e_board_modem_pin pin,
                                 enum e_board_modem_pin_ctrl ctrl);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __CONFIG_HALTIAN_TSONE_B1_INCLUDE_BOARD_MODEM_H */
