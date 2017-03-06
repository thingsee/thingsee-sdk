/************************************************************************************
 * configs/haltian-tsone/include/board-gps.h
 * include/arch/board/board-gps.h
 *
 *   Copyright (C) 2014-2015 Haltian Ltd. All rights reserved.
 *
 * Authors:
 *   Authors: Sami Pelkonen <sami.pelkonen@haltian.com>
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

#ifndef __CONFIG_HALTIAN_TSONE_B1_INCLUDE_BOARD_GPS_H
#define __CONFIG_HALTIAN_TSONE_B1_INCLUDE_BOARD_GPS_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <stdbool.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

#define BOARD_HAS_GPS_PM_SET_NEXT_MESSAGE_TIME 1

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: board_gps_power
 *
 * Description:
 *   Power GPS chip on / off
 *
 ****************************************************************************/

EXTERN void board_gps_power(bool on);

/****************************************************************************
 * Name: board_gps_initialize
 *
 * Description:
 *   Initialize the u-blox GPS.
 *
 * Return:
 *   Opened GPS serial port file descriptor.
 *
 ****************************************************************************/

EXTERN int board_gps_initialize(void);

/****************************************************************************
 * Name: board_gps_deinitialize
 *
 * Description:
 *   Deinitialize the u-blox GPS.
 *
 ****************************************************************************/

EXTERN int board_gps_deinitialize(int fd);

/****************************************************************************
 * Name: board_gps_pm_set_next_message_time
 *
 * Description:
 *   Power-management hint for board level. GPS library can inform board level
 *   when next (navigation) message is expected to be received over UART.
 *   Expected time is given as absolute time in CLOCK_MONOTONIC domain.
 *
 ****************************************************************************/

EXTERN void board_gps_pm_set_next_message_time(const struct timespec *msg_abstime);

/****************************************************************************
 * Name: board_gps_tx_buffer_empty
 *
 * Description:
 *   Check whether GPS UART TX buffer is empty
 *
 ****************************************************************************/

EXTERN bool board_gps_tx_buffer_empty(int fd);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __CONFIG_HALTIAN_TSONE_B1_INCLUDE_BOARD_GPS_H */
