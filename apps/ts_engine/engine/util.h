/****************************************************************************
 * apps/ts_engine/engine/util.h
 *
 * Copyright (C) 2015-2016 Haltian Ltd. All rights reserved.
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
 * Authors:
 *   Pekka Ervasti <pekka.ervasti@haltian.com>
 *
 ****************************************************************************/

#ifndef __APPS_TS_ENGINE_ENGINE_UTIL_H__
#define __APPS_TS_ENGINE_ENGINE_UTIL_H__

#include <arch/board/board-eeprom.h>
#include <apps/thingsee/modules/ts_emmc.h>

#include "parse.h"

#define SDCARD_CLOUD_PROPERTY_FILENAME          TS_EMMC_MOUNT_PATH "/cloud.jsn"
#define SDCARD_DEVICE_PROPERTY_FILENAME         TS_EMMC_MOUNT_PATH "/device.jsn"

bool __ts_engine_sdcard_inserted(void);

const char *__ts_engine_sdcard_read(const char * const filename);

const char *__ts_engine_sdcard_read(const char * const filename);

int __ts_engine_sdcard_write(const char * const filename,
    const char * const data, size_t len, bool append);

const char * __ts_engine_eeprom_read(enum board_eeprom_section_e section);

int __ts_engine_eeprom_write(enum board_eeprom_section_e section,
    char * const data, size_t len);

bool __ts_engine_find_threshold(struct ts_cause *cause,
    enum ts_threshold_t thr_type, double *value, bool *valuebool);

int __ts_engine_full_read(int fd, const void *buf, size_t count);

int __ts_engine_full_write(int fd, const void *buf, size_t count);

const char * const __ts_engine_util_find_log(const char * const dirname,
    const char * const basename);

const char * const __ts_engine_util_gen_next_log(const char * const dirname,
    const char * const basename);

int __ts_engine_util_cp(const char * const src, const char * const dst);

#endif
