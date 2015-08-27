/************************************************************************************
 * configs/haltian-tsone/include/board-deepsleep.h
 * include/arch/board/board-deepsleep.h
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

#ifndef __CONFIG_HALTIAN_TSONE_B1_INCLUDE_BOARD_DEEPSLEEP_H
#define __CONFIG_HALTIAN_TSONE_B1_INCLUDE_BOARD_DEEPSLEEP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <stdint.h>
#include <nuttx/config.h>
#include <arch/chip/chip.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Maximum deep-sleep time in seconds currently supported. */

#define BOARD_DEEPSLEEP_MAX_SECS (0xffff)

/************************************************************************************
 * Public function prototypes
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
* Name: board_deepsleep_with_stopmode
*
* Description:
*   Drive MCU to STOP mode for 'secs' seconds. Wake-up is performed either by RTC
*   or by external interrupt on EXTI line.
*
*   NOTE: TODO: Wake-up by RTC not implemented yet.
*
************************************************************************************/

EXTERN void board_deepsleep_with_stopmode(uint32_t secs);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __CONFIG_HALTIAN_TSONE_B1_INCLUDE_BOARD_DEEPSLEEP_H */
