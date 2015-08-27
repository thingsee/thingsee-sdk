/************************************************************************************
 * configs/haltian-tsone/include/board-eeprom.h
 * include/arch/board/board-eeprom.h
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

#ifndef __BOARD_EEPROM_H__
#define __BOARD_EEPROM_H__

/************************************************************************************
 * Included Files
 ************************************************************************************/

/************************************************************************************
 * Definitions
 ************************************************************************************/

enum board_eeprom_section_e
{
  /* See up_eeprom.c for EEPROM allocation. */
  BOARD_EEPROM_SECTION_PROD_INFO = 0,
  BOARD_EEPROM_SECTION_PROD_INFO_BACKUP,
  BOARD_EEPROM_SECTION_FWUPDATE,
  BOARD_EEPROM_SECTION_ENGINE_PROPERTY,
  BOARD_EEPROM_SECTION_ENGINE_PROFILE,
  BOARD_EEPROM_SECTION_ENGINE_ASSERT_CRASH_DATA,
};

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/* Get pointer to and size of EEPROM section. */
const void *board_eeprom_get_section(enum board_eeprom_section_e sect,
                                     size_t *size);

/* Write buffer 'buf' of length 'len' to EEPROM section at offset 'offs'. */
ssize_t board_eeprom_write_section(enum board_eeprom_section_e sect,
                                   size_t offs, const void *buf, size_t len);

/* Erase area of length 'len' in EEPROM section at offset 'offs'. */
ssize_t board_eeprom_erase_section(enum board_eeprom_section_e sect,
                                   size_t offs, size_t len);

/* Get production data from EEPROM. On success, returns pointer to string and
 * stores length of this string to 'size'. On error, returns NULL. */
const char *board_eeprom_get_prod_data(size_t *size);

/* Get production data backup from EEPROM. On success, returns pointer to string
 * and stores length of this string to 'size'. On error, returns NULL. */
const char *board_eeprom_get_prod_data_backup(size_t *size);

/* Write production data string to EEPROM. On success returns OK. On error,
 * negative error code. */
int board_eeprom_write_new_prod_data(const char *str);

#endif /*__BOARD_EEPROM_H__*/
