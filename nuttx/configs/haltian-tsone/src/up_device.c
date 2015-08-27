/************************************************************************************
 * configs/haltian-tsone/src/up_device.c
 * arch/arm/src/board/up_device.c
 *
 *   Copyright (C) 2014-2015 Haltian Ltd. All rights reserved.
 *
 * Authors:
 *   Sami Pelkonen <sami.pelkonen@haltian.com>
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

#include <arch/board/board.h>
#include <arch/board/board-device.h>
#include <arch/board/board-eeprom.h>

#include "up_arch.h"
#include "up_internal.h"
#include "haltian-tsone.h"

#include "stm32_flash.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#if defined(CONFIG_STM32_MEDIUMPLUSDENSITY) || defined(CONFIG_STM32_HIGHDENSITY)
#  define STM32_UID_BASE     (0x1ff800d0)
#else
#  define STM32_UID_BASE     (0x1ff80050)
#endif

#define STM32_UID ((struct stm32_uid_s *)STM32_UID_BASE)

/************************************************************************************
 * Definitions
 ************************************************************************************/

struct stm32_uid_s {
  uint32_t id0;                 /* U_ID bits 31:0 at offset 0x00 */
  uint32_t id1;                 /* U_ID bits 63:32 at offset 0x04 */
  uint32_t reserved[3];         /* Reserved at offset 0x08 - 0x10 */
  uint32_t id2;                 /* U_ID bits 95:64 at offset 0x14 */
} __attribute__((packed));

/************************************************************************************
 * Private Data
 ************************************************************************************/

static int32_t g_hwver = BOARD_HWVER_UNKNOWN;

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static int32_t get_hwver_from_proddata(void)
{
  size_t slen = 0;
  const char *prod;
  const char *findstr;
  const char *hwid_str;
  const char *pc;
  unsigned long hwid;

  /* Get production data. */

  prod = board_eeprom_get_prod_data(&slen);
  if (!prod)
    {
      /* Attempt to use backup production data. */

      prod = board_eeprom_get_prod_data_backup(&slen);
      if (!prod)
        {
          goto err_no_prod_data;
        }
    }

  /* Poor man's JSON parsing... make use of fact that production data JSON has
   * extra whitespaces stripped. */

  /* Get position for HWID */

  findstr = "\"hwid\":\"";
  pc = strstr(prod, findstr);
  if (!pc)
    {
      goto err_no_hwid_data;
    }

  hwid_str = pc + strlen(findstr);

  /* Convert hex-string to integer. */

  hwid = strtoul(hwid_str, NULL, 16);

  if (hwid < 0x0300U)
    {
      /* Before B1.7 */

      return BOARD_HWVER_B1_5;
    }

  if ((hwid & 0xff00U) == 0x0300U)
    {
      /* B1.7 */

      return BOARD_HWVER_B1_7;
    }

  if ((hwid & 0xff00U) == 0x0400U)
    {
      /* B2.0 (and possibly B3.0) */

      return BOARD_HWVER_B2_0;
    }

  /* Unknown HWID? */

err_no_prod_data:
err_no_hwid_data:

  /* No production data or no/unknown HWID. Default to current HW version. */

  return BOARD_HWVER_B2_0;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

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

bool board_device_get_uid(struct device_uid_s * const uid)
{
  if (!uid)
    return false;

  uid->id0 = STM32_UID->id0;
  uid->id1 = STM32_UID->id1;
  uid->id2 = STM32_UID->id2;

  return true;
}

/************************************************************************************
 * Name: board_get_hw_ver
 *
 * Description:
 *   Get board HW version
 *
 * Returns:
 *   HW version number
 *
 ************************************************************************************/

int32_t board_get_hw_ver(void)
{
  DEBUGASSERT(g_hwver != BOARD_HWVER_UNKNOWN);

  return g_hwver;
}

/************************************************************************************
 * Name: board_detect_board_hwver
 *
 * Description:
 *   Detect board HW version
 *
 ************************************************************************************/

int32_t board_detect_board_hwver(void)
{
  g_hwver = get_hwver_from_proddata();
  return g_hwver;
}
