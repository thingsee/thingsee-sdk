/************************************************************************************
 * configs/haltian-tsone/include/board-device.h
 * include/arch/board/board-device.h
 *
 *   Copyright (C) 2014-2015 Haltian Ltd. All rights reserved.
 *
 * Authors:
 *   Sami Pelkonen <sami.pelkonen@haltian.com>
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

#ifndef __CONFIG_HALTIAN_TSONE_B1_INCLUDE_BOARD_DEVICE_H
#define __CONFIG_HALTIAN_TSONE_B1_INCLUDE_BOARD_DEVICE_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <stdbool.h>
#include <nuttx/config.h>
#include <arch/chip/chip.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define BOARD_HWVER_B1_5    0x0105
#define BOARD_HWVER_B1_7    0x0107
#define BOARD_HWVER_B2_0    0x0200
#define BOARD_HWVER_UNKNOWN -1

/************************************************************************************
 * Definitions
 ************************************************************************************/

struct device_uid_s {
  uint32_t id0;
  uint32_t id1;
  uint32_t id2;
};

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Name: board_device_get_uid
 *
 * Description:
 *   Get device UID
 *
 * Returns:
 *   true on success
 *   false on failure
 *
 ************************************************************************************/

EXTERN bool board_device_get_uid(struct device_uid_s * const uid);

/************************************************************************************
 * Name: board_get_hw_ver
 *
 * Description:
 *   Detect and get board HW version
 *
 * Returns:
 *   HW version number
 *
 ************************************************************************************/

EXTERN int32_t board_get_hw_ver(void);

/************************************************************************************
 * Name: board_detect_board_hwver
 *
 * Description:
 *   Detect board HW version
 *
 ************************************************************************************/

int32_t board_detect_board_hwver(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __CONFIG_HALTIAN_TSONE_B1_INCLUDE_BOARD_DEVICE_H */
