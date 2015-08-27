/****************************************************************************
 * apps/thingsee/lib/ts_devinfo.h
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
 *   Authors: Sami Pelkonen <sami.pelkonen@haltian.com>
 *            Jussi Kivilinna <jussi.kivilinna@haltian.com>
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

#ifndef __APPS_INCLUDE_THINGSEE_TS_DEVINFO_H
#define __APPS_INCLUDE_THINGSEE_TS_DEVINFO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/version.h>
#include <sys/types.h>
#include <arch/board/board-device.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TS_DEVICE_PROD_DATA_LSM9DS1_REF_SIZE 128

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct device_information_s
{
  struct device_uid_s uid;
  const char * sn;
  uint16_t hwid;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ts_device_prod_data_get_entry
 *
 * Description:
 *   Get data string from production data set
 ****************************************************************************/

int ts_device_prod_data_get_entry(const char *entry, char *buf, size_t buflen);

/****************************************************************************
 * Name: ts_device_prod_data_set_entry
 *
 * Description:
 *   Set data string to production data set
 ****************************************************************************/

int ts_device_prod_data_set_entry(const char *entry, const char *valuestr);

/****************************************************************************
 * Name: ts_fill_missing_prod_data
 *
 * Description:
 *   Fill missing production data for early prototypes.
 *
 ****************************************************************************/

void ts_fill_missing_prod_data(void);

/****************************************************************************
 * Name: ts_device_identify
 *
 * Description:
 *   Get device information (mapping MCU UID to SN + HWID)
 *
 ****************************************************************************/

struct device_information_s const * const
ts_device_identify( struct device_uid_s const * const uid);

/****************************************************************************
 * Name: ts_device_is_prototype_before_B17
 *
 * Description:
 *   Detect early engineering prototype dynamically at run-time.
 *
 ****************************************************************************/
int ts_device_is_prototype_before_B17(void);

#endif /* __APPS_INCLUDE_THINGSEE_TS_DEVINFO_H */
