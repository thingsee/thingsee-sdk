/************************************************************************************
 * configs/haltian-tsone/include/board-reset.h
 * include/arch/board/board-reset.h
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

#ifndef __CONFIG_HALTIAN_TSONE_B1_INCLUDE_BOARD_RESET_H
#define __CONFIG_HALTIAN_TSONE_B1_INCLUDE_BOARD_RESET_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Reset reason flags. */

#define BOARD_RESET_REASON_LOW_POWER                (1 << 0)
#define BOARD_RESET_REASON_WINDOW_WATCHDOG          (1 << 1)
#define BOARD_RESET_REASON_INDEPENDENT_WATCHDOG     (1 << 2)
#define BOARD_RESET_REASON_SOFTWARE                 (1 << 3)
#define BOARD_RESET_REASON_POR_PDR                  (1 << 4)
#define BOARD_RESET_REASON_NRST_PIN                 (1 << 5)
#define BOARD_RESET_REASON_OPTIONS_BYTES_LOADING    (1 << 6)
#define BOARD_RESET_REASON_HARDFAULT                (1 << 7)
#define BOARD_RESET_REASON_STANDBY_WAKEUP           (1 << 8)

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/****************************************************************************
 * Name: board_rtc_time_is_set
 *
 * Description:
 *   Is the RTC clock set.
 *
 ****************************************************************************/

bool board_rtc_time_is_set(time_t *when_was_set);

/****************************************************************************
 * Name: board_reset_get_reason
 *
 * Description:
 *   Get reset reason flags, and clear reset flags from HW.
 *
 ****************************************************************************/

uint32_t board_reset_get_reason(bool clear);

/****************************************************************************
 * Name: board_systemreset
 *
 * Description:
 *   Perform MCU reset.
 *
 ****************************************************************************/

void board_systemreset(void) noreturn_function;

/****************************************************************************
 * Name: board_reset_to_system_bootloader
 *
 * Description:
 *   Perform MCU reset to system bootloader / DFU flash mode
 *
 ****************************************************************************/

void board_reset_to_system_bootloader(void) noreturn_function;

/****************************************************************************
 * Name: board_go_to_standby
 *
 * Description:
 *   Drive MCU to standby mode (lowest), with wake-up from power-button EXTI
 *   GPIO. Note: All HW modules must be driven to powered-off state before
 *   calling this function.
 *
 ****************************************************************************/

void board_go_to_standby(void) noreturn_function;

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __CONFIG_HALTIAN_TSONE_B1_INCLUDE_BOARD_RESET_H */
