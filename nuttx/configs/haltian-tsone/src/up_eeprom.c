/****************************************************************************
 * config/haltian-tsone/src/up_eeprom.c
 * arch/arm/src/board/up_eeprom.c
 *
 *   Copyright (C) 2014-2015 Haltian Ltd. All rights reserved.
 *   Authors: Jussi Kivilinna <jussi.kivilinna@haltian.com>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <string.h>

#include <nuttx/arch.h>

#include <arch/board/board.h>
#include <arch/board/board-eeprom.h>

#include "haltian-tsone.h"

#include "stm32.h"
#include "stm32_flash.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/*
 * EEPROM allocation
 *
 * STM32L162VEY has 16 KiB of EEPROM (base: 0x08080000) in two 8 KiB banks.
 *  Bank 1: 0x08080000-0x08081fff
 *  Bank 2: 0x08082000-0x08083fff
 *
 * Offset   Size    Use
 *  0x0000   1024    Production data
 *  0x0400   1024    Firmware update data
 *  0x0800   6144    Thingsee Engine property & profiles
 *  0x2000   1024    Production data backup/mirror
 *  0x2400    512    Assert crash data
 *  0x2600   6656    <Unused>
 *
 * (TODO: Move production data mirror to 0x3C00-0x3fff to allow larger block
 *        for Engine profiles?)
 */

#define BOARD_EEPROM_SECTION_PROD_INFO_ADDR        (0 * 1024)
#define BOARD_EEPROM_SECTION_PROD_INFO_SIZE        1024

#define BOARD_EEPROM_SECTION_FWUPDATE_ADDR         (1 * 1024)
#define BOARD_EEPROM_SECTION_FWUPDATE_SIZE         1024

#define BOARD_EEPROM_SECTION_ENGINE_PROPERTY_ADDR   (2 * 1024)
#define BOARD_EEPROM_SECTION_ENGINE_PROPERTY_SIZE   (1 * 1024)

#define BOARD_EEPROM_SECTION_ENGINE_PROFILE_ADDR   (3 * 1024)
#define BOARD_EEPROM_SECTION_ENGINE_PROFILE_SIZE   (5 * 1024)

#define BOARD_EEPROM_SECTION_PROD_INFO_BACKUP_ADDR (8 * 1024)
#define BOARD_EEPROM_SECTION_PROD_INFO_BACKUP_SIZE \
        BOARD_EEPROM_SECTION_PROD_INFO_SIZE

#define BOARD_EEPROM_SECTION_ASSERT_CRASH_DATA_ADDR (9 * 1024)
#define BOARD_EEPROM_SECTION_ASSERT_CRASH_DATA_SIZE (512)

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct eeprom_header_s {
  uint16_t datalen;
  uint32_t chksum;
} packed_struct;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint32_t crc_table[16] = {
  0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
  0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
  0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
  0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static intptr_t sect_to_addr(enum board_eeprom_section_e sect, size_t *size)
{
  intptr_t sectaddr;
  size_t sectsize;

  switch (sect)
    {
    case BOARD_EEPROM_SECTION_PROD_INFO:
      sectaddr = BOARD_EEPROM_SECTION_PROD_INFO_ADDR;
      sectsize = BOARD_EEPROM_SECTION_PROD_INFO_SIZE;
      break;
    case BOARD_EEPROM_SECTION_FWUPDATE:
      sectaddr = BOARD_EEPROM_SECTION_FWUPDATE_ADDR;
      sectsize = BOARD_EEPROM_SECTION_FWUPDATE_SIZE;
      break;
    case BOARD_EEPROM_SECTION_ENGINE_PROPERTY:
      sectaddr = BOARD_EEPROM_SECTION_ENGINE_PROPERTY_ADDR;
      sectsize = BOARD_EEPROM_SECTION_ENGINE_PROPERTY_SIZE;
      break;
    case BOARD_EEPROM_SECTION_ENGINE_PROFILE:
      sectaddr = BOARD_EEPROM_SECTION_ENGINE_PROFILE_ADDR;
      sectsize = BOARD_EEPROM_SECTION_ENGINE_PROFILE_SIZE;
      break;
    case BOARD_EEPROM_SECTION_PROD_INFO_BACKUP:
      sectaddr = BOARD_EEPROM_SECTION_PROD_INFO_BACKUP_ADDR;
      sectsize = BOARD_EEPROM_SECTION_PROD_INFO_BACKUP_SIZE;
      break;
    case BOARD_EEPROM_SECTION_ENGINE_ASSERT_CRASH_DATA:
      sectaddr = BOARD_EEPROM_SECTION_ASSERT_CRASH_DATA_ADDR;
      sectsize = BOARD_EEPROM_SECTION_ASSERT_CRASH_DATA_SIZE;
      break;

    default:
      sectaddr = -1;
      sectsize = 0;
      break;
    }

  DEBUGASSERT(sectsize > 0);

  if (size)
    *size = sectsize;
  return sectaddr;
}

static ssize_t
board_eeprom_erase_write_section(enum board_eeprom_section_e sect,
                                 size_t offs, const void *buf, size_t len)
{
  size_t addr;
  size_t size;
  ssize_t ret;

  addr = sect_to_addr(sect, &size);

  if (offs >= size || len > size)
    return -EINVAL;
  if (offs > size - len)
    return -EINVAL;

  if (buf)
    ret = stm32_eeprom_write(addr + offs, buf, len);
  else
    ret = stm32_eeprom_erase(addr + offs, len);

  if (ret < (ssize_t)len)
    return ret < 0 ? ret : -EIO;

  return len;
}

static uint32_t crc_next(uint32_t crc, uint8_t data)
{
  /* Code generated by universal_crc by Danjel McGougan
   *
   * CRC parameters used:
   *   bits:       32
   *   poly:       0x04c11db7
   *   init:       0xffffffff
   *   xor:        0xffffffff
   *   reverse:    true
   *   non-direct: false
   *   algorithm:  tab16
   *
   * CRC of the string "123456789" is 0xcbf43926
   */
  crc ^= data;
  crc = (crc >> 4) ^ crc_table[crc & 15];
  crc = (crc >> 4) ^ crc_table[crc & 15];
  return crc;
}

static uint32_t crc_final(uint32_t crc)
{
  return ~crc;
}

static uint32_t crc_init(void)
{
  static bool selftest_done = false;

  if (!selftest_done)
    {
      uint32_t crc;
      const char *teststr = "123456789";

      selftest_done = true;

      crc = crc_init();
      while (*teststr)
        crc = crc_next(crc, *(teststr++));
      crc = crc_final(crc);

      DEBUGASSERT(crc == 0xcbf43926);
    }

  /* TODO: STM32L1xxx has CRC peripheral, we could use it instead. */

  return 0xffffffff;
}

const void *eeprom_get_prod_data(enum board_eeprom_section_e sect,
                                 size_t *size)
{
  const void *addr;
  uint32_t chksum;
  struct eeprom_header_s hdr;
  const uint8_t *buf;
  unsigned int i;
  size_t sectsize;

  addr = board_eeprom_get_section(sect, &sectsize);
  if (!addr || sectsize < sizeof(hdr))
    return NULL;

  hdr = *(const struct eeprom_header_s *)(addr);

  buf = (const void *)(addr + sizeof(hdr));

  /* Length value over bounds? */

  if (hdr.datalen > sectsize - sizeof(hdr))
    return NULL;

  /* Data length must include size of null-terminating byte. */

  if (hdr.datalen < 1)
    return NULL;

  /* Calculate checksum. */

  chksum = crc_init();

  for (i = 0; i < hdr.datalen; i++)
    {
      chksum = crc_next(chksum, buf[i]);
    }

  chksum = crc_final(chksum);

  /* Verify */

  if (chksum != hdr.chksum)
    return NULL;

  /* Checksum correct, buffer should contain JSON string.
   * Output string length.
   */

  if (size)
    *size = hdr.datalen - 1;

  return buf;
}

static int do_prod_data_write(enum board_eeprom_section_e sect,
                              const struct eeprom_header_s *hdr,
                              const char *buf, size_t buflen)
{
  const char *eeprom_addr;
  size_t size;

  eeprom_addr = board_eeprom_get_section(sect, &size);
  if (!eeprom_addr)
    return -EINVAL;

  if (sizeof(*hdr) + buflen > size)
    return -EINVAL;

  /* Reprogram only if data in EEPROM is different. */
  if (memcmp(hdr, &eeprom_addr[0], sizeof(*hdr)) != 0 ||
      memcmp(buf, &eeprom_addr[sizeof(*hdr)], buflen) != 0)
    {
      ssize_t ret;
      size_t left;

      /* Write header. */

      ret = board_eeprom_write_section(sect, 0, hdr, sizeof(*hdr));
      if (ret < (ssize_t)sizeof(*hdr))
        {
          /* Writing failed? */
          return ret < 0 ? ret : -EIO;
        }

      /* Write data. */

      ret = board_eeprom_write_section(sect, sizeof(*hdr), buf, buflen);
      if (ret < (ssize_t)buflen)
        {
          /* Writing failed? */
          return ret < 0 ? ret : -EIO;
        }

      /* Zeroize the trailing bytes. */

      left = size - sizeof(*hdr) - buflen;
      if (left == 0)
        return OK;

      ret = board_eeprom_erase_section(sect, sizeof(*hdr) + buflen, left);
      if (ret < (ssize_t)left)
        {
          /* Erasing failed? */
          return ret < 0 ? ret : -EIO;
        }
    }

  return OK;
}

static int eeprom_write_new_prod_data(const char *buf, size_t buflen)
{
  struct eeprom_header_s hdr;
  unsigned int i;
  ssize_t ret;

  if (!buf)
    return -EINVAL;
  if (buflen > BOARD_EEPROM_SECTION_PROD_INFO_SIZE - sizeof(hdr))
    return -EINVAL;

  hdr.datalen = buflen;

  /* Calculate checksum. */

  hdr.chksum = crc_init();

  for (i = 0; i < hdr.datalen; i++)
    {
      hdr.chksum = crc_next(hdr.chksum, buf[i]);
    }

  hdr.chksum = crc_final(hdr.chksum);

  /* Write to main production data memory. */

  ret = do_prod_data_write(BOARD_EEPROM_SECTION_PROD_INFO, &hdr,
                           buf, buflen);
  if (ret != OK)
    return ret;

  /* Write to backup memory. */

  ret = do_prod_data_write(BOARD_EEPROM_SECTION_PROD_INFO_BACKUP, &hdr,
                           buf, buflen);
  if (ret != OK)
    return ret;

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

const void *board_eeprom_get_section(enum board_eeprom_section_e sect,
                                     size_t *size)
{
  intptr_t addr;

  addr = sect_to_addr(sect, size);
  if (addr == -1)
    return NULL;

  return (const void *)(stm32_eeprom_getaddress() + addr);
}

ssize_t board_eeprom_write_section(enum board_eeprom_section_e sect,
                                   size_t offs, const void *buf, size_t len)
{
  if (!buf)
    return -EINVAL;

  return board_eeprom_erase_write_section(sect, offs, buf, len);
}

ssize_t board_eeprom_erase_section(enum board_eeprom_section_e sect,
                                   size_t offs, size_t len)
{
  return board_eeprom_erase_write_section(sect, offs, NULL, len);
}

const char *board_eeprom_get_prod_data(size_t *size)
{
  return eeprom_get_prod_data(BOARD_EEPROM_SECTION_PROD_INFO, size);
}

const char *board_eeprom_get_prod_data_backup(size_t *size)
{
  return eeprom_get_prod_data(BOARD_EEPROM_SECTION_PROD_INFO_BACKUP, size);
}

int board_eeprom_write_new_prod_data(const char *str)
{
  return eeprom_write_new_prod_data(str, strlen(str) + 1);
}
