/************************************************************************************
 * configs/haltian-tsone/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2014-2015 Haltian Ltd. All rights reserved.
 *
 * Authors:
 *   Sami Pelkonen <sami.pelkonen@haltian.com>
 *   Jussi Kivilinna <jussi.kivilinna@haltian.com>
 *   Juha Niskanen <juha.niskanen@haltian.com>
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

#ifndef __CONFIG_HALTIAN_TSONE_INCLUDE_BOARD_H
#define __CONFIG_HALTIAN_TSONE_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/
#include <nuttx/config.h>
#include <stdbool.h>

#if defined (CONFIG_ARCH_BOARD_HALTIAN_TSONE_B15)
#include "board-b15.h"
#else
#error "No board file founded for selected board"
#endif

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Button definitions ***************************************************************/
/* The ThingseeOne support one button.  The button will read "0" when open and
 * "1" when closed.
 *
 * Device node "/dev/buttons0" is made for available for reading button state
 * and receive poll notifications on button interrupts.
 *
 * POWERKEY       -- Connected to PC13
 *
 * For the interrupts are generated on both edges (press and release).
 */

#define BOARD_BUTTON_POWERKEY      0

#define BOARD_NUM_BUTTONS          1

#define BOARD_BUTTON_POWERKEY_BIT  (1 << BOARD_BUTTON_POWERKEY)

/* Misc *****************************************************************************/

#define CONFIG_HARDFAULT_MAGIC_BKREG  STM32_RTC_BK20R
#define CONFIG_HARDFAULT_MAGIC        ((uint32_t)0xFEE1DEAD)

#define CONFIG_STANDBYMODE_MAGIC_BKREG  STM32_RTC_BK18R
#define CONFIG_STANDBYMODE_MAGIC        ((uint32_t)0x57099ed)

#define CONFIG_ASSERT_COUNT_BKREG       STM32_RTC_BK19R
#define CONFIG_ASSERT_COUNT_MASK        ((uint32_t)0xffffU)
#define CONFIG_ASSERT_COUNT_MAGIC       ((uint32_t)0xDEADU << 16)
#define CONFIG_ASSERT_COUNT_MAGIC_MASK  ((uint32_t)0xffffU << 16)
#define CONFIG_ASSERT_COUNT_TO_DFU_MODE 10

/************************************************************************************
 * Public Data
 ************************************************************************************/

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

/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

void stm32_boardinitialize(void);

/************************************************************************************
 * Board-specific LED and LCD functions.
 ************************************************************************************/

void board_setleds(uint8_t ledset);
void board_setled(int led, bool ledon);
void board_lcdoff(void);
void board_lcdon(void);

/****************************************************************************
 * Name: board_pwr_charger_connected
 *
 * Description:
 *  This is called by charger module to inform board-level SW about charging
 *  status.
 *
 ****************************************************************************/

void board_pwr_charger_connected(const char *porttype);

/****************************************************************************
 * Name: board_pwr_checkvbat
 *
 * Description:
 *  Check if operating voltage is too low, and go to standby mode if it is.
 *
 ****************************************************************************/

void board_pwr_checkvbat(bool checktwice);

/****************************************************************************
 * Name: board_pwr_checkpvd
 *
 * Description:
 *  Check if operating voltage is too low, and go to standby mode if it is.
 *
 *  This is called both from PVD interrupt handler and deepsleep code.
 *
 ****************************************************************************/

void board_pwr_checkpvd(void);

/****************************************************************************
 * Name: board_pwr_enablepvd
 *
 * Description:
 *  Triggers interrupt if operating voltage is too low
 *
 ****************************************************************************/

void board_pwr_enablepvd(void);

/****************************************************************************
 * Name: up_set_hardfault_magic
 *
 * Description:
 *   Set hard-fault magic to backup register.
 *
 ****************************************************************************/

#if defined(CONFIG_HARDFAULT_MAGIC_BKREG) && defined(CONFIG_HARDFAULT_MAGIC)
void up_set_hardfault_magic(void);
#endif

/****************************************************************************
 * Name: up_add_assert_count
 *
 * Description:
 *   Increase count in assert counter
 *
 ****************************************************************************/

#if defined(CONFIG_ASSERT_COUNT_BKREG)
unsigned int up_add_assert_count(unsigned int val);
#endif

/****************************************************************************
 * Name: board_reset_assert_count
 *
 * Description:
 *   Reset assert counter
 *
 ****************************************************************************/

void board_reset_assert_count(void);

/****************************************************************************
 * Name: up_reset_to_system_bootloader
 *
 * Description:
 *   Perform MCU reset to system bootloader / DFU flash mode (from kernel)
 *
 ****************************************************************************/

void up_reset_to_system_bootloader(void) noreturn_function;

/****************************************************************************
 * Name: board_rtc_is_mass_storage_disallowed
 *
 * Description:
 *  Checks if mass-storage mode was disallowed by the user
 *
 ****************************************************************************/

bool board_rtc_is_mass_storage_disallowed(void);

/****************************************************************************
 * Name: board_rtc_block_mass_storage
 *
 * Description:
 *   Block/Unblock mass storage mode
 *
 ****************************************************************************/

void board_rtc_block_mass_storage(bool block);

/****************************************************************************
 * Name: board_rtc_save_value
 *
 * Description:
 *   Persist a value
 *
 ****************************************************************************/

void board_rtc_save_value(uint32_t value, uint32_t index);

/****************************************************************************
 * Name: board_rtc_read_value
 *
 * Description:
 *   Read a persisted value
 *
 ****************************************************************************/

uint32_t board_rtc_read_value(uint32_t index);

/****************************************************************************
 * Name: board_wdginitialize_autokick()
 *
 * Description:
 *   Perform architecture-specific initialization of the Watchdog hardware
 *   to automatic kicking mode.
 *
 ****************************************************************************/

int board_wdginitialize_autokick(void (*irq_tracefn)(void));

void board_hwwdg_kick(void);

/****************************************************************************
 * Name: stm32_slcd_initialize
 *
 * Description:
 *   Initialize the STM32L-Discovery LCD hardware and register the character
 *   driver as /dev/slcd.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_LCD
int stm32_slcd_initialize(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */


#endif  /* __CONFIG_HALTIAN_TSONE_INCLUDE_BOARD_H */
