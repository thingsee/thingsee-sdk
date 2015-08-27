/****************************************************************************************************
 * configs/haltian-tsone/src/haltian-tsone.h
 * arch/arm/src/board/haltian-tsone.h
 *
 *   Copyright (C) 2015 Haltian ltd. All rights reserved.
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
 ****************************************************************************************************/

#ifndef __CONFIGS_HALTIAN_TSONE_SRC_HALTIAN_TSONE_INTERNAL_H
#define __CONFIGS_HALTIAN_TSONE_SRC_HALTIAN_TSONE_INTERNAL_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include "chip.h"

#if defined (CONFIG_ARCH_BOARD_HALTIAN_TSONE_B15)
#  include "haltian-tsone-b15.h"
#else
#  error "No board file founded for selected board"
#endif

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

/* Backup registers used by boot-loader *************************************************************/

#define CONFIG_BOOTLOADER_ADDR_BKREG   STM32_RTC_BK22R    /* Jump address */
#define CONFIG_BOOTLOADER_BRSN_BKREG   STM32_RTC_BK23R    /* Boot reason */
#define CONFIG_BOOTLOADER_NOWD_BKREG   STM32_RTC_BK24R    /* Disable iwdog */
#define CONFIG_BOOTLOADER_FWUS_BKREG   STM32_RTC_BK25R    /* Status for last FW
                                                           * update attempt. */
#define CONFIG_BOOTLOADER_CBTC_BKREG   STM32_RTC_BK26R    /* Current boot try
                                                           * count. */
#define CONFIG_BOOTLOADER_CUTC_BKREG   STM32_RTC_BK27R    /* Current update try
                                                           * count. */
#define CONFIG_BOOTLOADER_RESERVED0_BKREG STM32_RTC_BK28R

/* Firmware locations *******************************************************************************/

#define BOARD_BOOTLOADER_BASE_ADDR     (STM32_FLASH_BASE)
#define BOARD_SYSMEM_BASE_ADDR         (STM32_SYSMEM_BASE)

#ifdef CONFIG_NUTTX_DFU_IMAGE_FLASHING_ADDRESS
#  define BOARD_FIRMWARE_BASE_ADDR     CONFIG_NUTTX_DFU_IMAGE_FLASHING_ADDRESS
#else
#  define BOARD_FIRMWARE_BASE_ADDR     (STM32_FLASH_BASE)
#endif

/* Building bootloader enabled image? ***************************************************************/

#undef BOARD_HAS_BOOTLOADER
#if (BOARD_FIRMWARE_BASE_ADDR != BOARD_BOOTLOADER_BASE_ADDR)
#  define BOARD_HAS_BOOTLOADER 1
#endif

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/* Dynamic pin configuration */
enum e_board_dyn_gpios {
  DYN_GPIO_WIFI_INT = 0,
  DYN_GPIO_CTRL_VBAT,
  DYN_GPIO_LED1,
  DYN_GPIO_PAD_J9007,
  DYN_GPIO_PAD_J9016,
  DYN_GPIO_BT_RESERVED1,
  DYN_GPIO_BT_RESERVED2,
  DYN_GPIO_MODEM_TX_BURST,
  DYN_GPIO_HWWDG_WAKE,
  DYN_GPIO_HWWDG_DONE,

  __DYN_GPIO_MAX,
  CONFIG_NUM_DYN_GPIO_MAX = __DYN_GPIO_MAX,
};

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

extern bool g_board_wakeup_from_standby;

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

int up_bringup(void);

/****************************************************************************************************
 * Name: up_dyngpio
 ****************************************************************************************************/

uint32_t up_dyngpio(enum e_board_dyn_gpios id);

#endif /* __ASSEMBLY__ */

#endif /* __CONFIGS_HALTIAN_TSONE_SRC_HALTIAN_TSONE_INTERNAL_H */
