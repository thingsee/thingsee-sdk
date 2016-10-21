/****************************************************************************
 * thingsee/system_properties/sys_prop.c
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
 *   Authors: Dmitry Nikolaev <dmitry.nikolaev@haltian.com>
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

#include <stdio.h>
#include <sys/types.h>
#include <stdlib.h>
#include <assert.h>
#include <debug.h>
#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <string.h>

#include <apps/thingsee/ts_core.h>
#include <apps/thingsee/ts_devinfo.h>

#include "sys_prop.h"



/****************************************************************************
 * Name: sys_prop_check_flag
 *
 * Description:
 *  Reads flag status from EEPROM
 *
 * Input Parameters:
 *  flag - flag to be checked
 *
 * Returned Values:
 *  true if flag is set. Otherwise false
 *
 ****************************************************************************/

bool sys_prop_check_flag(uint32_t flag)
{
    char buf[8] = { 0 };
    uint32_t resp;
    int ret;

    ret = ts_device_prod_data_get_entry("boot_mask", buf, sizeof(buf));
    if (!ret) {
        /* If we cannot read this flag, then rather wait for it
         * to be checked with HEAT-interface manually set from screen menu */
        return false;
    }

    resp = (uint32_t)atoi(buf);

    return !!(resp & flag);
}

/****************************************************************************
 * Name: sys_prop_clear_flag
 *
 * Description:
 *  Clears flag in EEPROM
 *
 * Input Parameters:
 *  flag - flag to be cleared
 *
 * Returned Values:
 *  true if flag is set. Otherwise false
 *
 ****************************************************************************/

int sys_prop_clear_flag(uint32_t flag)
{
    uint32_t boot_mask = 0;
    char buf[8] = { 0 };
    int ret;

    /* First, we have to read the old data. At the very first time it returns 0
     * At the very first time there is no any value with the key "boot_mask" */

    ts_device_prod_data_get_entry("boot_mask", buf, sizeof(buf));

    /* Now we have to convert data to the number */

    boot_mask = (uint32_t)atoi(buf);
    boot_mask &= ~flag;

    /* Convert value to string */

    memset(buf, 0, sizeof(buf));
    sprintf(buf, "%u", boot_mask);

    /* Write data to EEPROM */

    ret = ts_device_prod_data_set_entry("boot_mask", buf);
    if (ret < 0) {
        return ERROR;
    }

    return OK;
}
