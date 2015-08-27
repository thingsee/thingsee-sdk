/****************************************************************************
 * apps/thingsee/lib/ts_devinfo.c
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
 *   Authors: Jussi Kivilinna <jussi.kivilinna@haltian.com>
 *            Sami Pelkonen <sami.pelkonen@haltian.com>
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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>

#include <apps/thingsee/ts_devinfo.h>
#include <apps/netutils/cJSON.h>
#include <arch/board/board-eeprom.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifndef CONFIG_THINGSEE_DISABLE_DEVICE_INFO_TABLE
static const struct device_information_s device_information[] =
{
};
#endif

struct {
  const char *eeprom_prod_str;
  const char *eeprom_backup_str;
  cJSON *json_root;
  cJSON *json_prod;
} prod_data;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int load_prod_data(bool backup)
{
  const char *prod_str;
  size_t prod_strlen;
  cJSON *json_root;
  cJSON *json_prod;

  prod_str = (!backup ? board_eeprom_get_prod_data
                      : board_eeprom_get_prod_data_backup)(&prod_strlen);
  if (!prod_str || prod_strlen != strlen(prod_str))
    return ERROR;

  if (!backup)
    prod_data.eeprom_prod_str = prod_str;
  else
    prod_data.eeprom_backup_str = prod_str;

  /* Attempt to parse JSON string. */

  json_root = cJSON_Parse(prod_str);
  if (!json_root)
    {
      /* Not valid JSON? */

      return ERROR;
    }

  /* Check that 'production data' sub-object exists. */

  json_prod = cJSON_GetObjectItem(json_root, "prod_info");
  if (!json_prod)
    {
      /* No production info yet? */

      cJSON_Delete(json_root);

      return ERROR;
    }

  /* Success. */

  prod_data.json_root = json_root;
  prod_data.json_prod = json_prod;

  return OK;
}

static int ts_init_prod_data(void)
{
  int ret;

  if (prod_data.json_prod)
    {
      /* Already loaded. */

      return OK;
    }

  /* Attempt to load production data from EEPROM. */

  ret = load_prod_data(false);
  if (ret < 0)
    {
      /* Attempt to load backup region. */

      ret = load_prod_data(true);
    }

  if (ret == OK)
    return OK;

  /* No production data available. Create fresh cJSON object. */

  prod_data.json_root = cJSON_CreateObject();
  DEBUGASSERT(prod_data.json_root);

  prod_data.json_prod = cJSON_CreateObject();
  DEBUGASSERT(prod_data.json_prod);

  cJSON_AddItemToObject(prod_data.json_root, "prod_info",
                        prod_data.json_prod);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ts_device_prod_data_get_entry
 *
 * Description:
 *   Get data string from production data set
 ****************************************************************************/

int ts_device_prod_data_get_entry(const char *entry, char *buf, size_t buflen)
{
  cJSON *item;

  if (buflen == 0)
    return 0;

  buf[0] = 0;

  if (ts_init_prod_data() != OK)
    return 0;

  item = cJSON_GetObjectItem(prod_data.json_prod, entry);
  if (!item)
    return 0;

  switch (item->type)
    {
    case cJSON_Number:
      return snprintf(buf, buflen, "%d", item->valueint);
    case cJSON_String:
      return snprintf(buf, buflen, "%s", item->valuestring);
    default:
      return 0;
    }
}

/****************************************************************************
 * Name: ts_device_prod_data_set_entry
 *
 * Description:
 *   Set data string to production data set
 ****************************************************************************/

int ts_device_prod_data_set_entry(const char *entry, const char *valuestr)
{
  cJSON *new_item, *old_item, *item;
  char *json_str;

  if (ts_init_prod_data() != OK)
    return ERROR;

  if (!valuestr)
    valuestr = "";

  /* Allocate new item. */

  new_item = cJSON_CreateString(valuestr);
  if (!new_item)
    return ERROR;

  /* Detach old item from object. */

  old_item = cJSON_DetachItemFromObject(prod_data.json_prod, entry);

  /* Add new item to object.*/

  cJSON_AddItemToObject(prod_data.json_prod, entry, new_item);

  /* Verify that new item went in. */

  item = cJSON_GetObjectItem(prod_data.json_prod, entry);
  if (!item)
    goto err;
  if (strcmp(item->valuestring, valuestr) != 0)
	goto err;

  /* Save new data object to EEPROM. */

  json_str = cJSON_PrintUnformatted(prod_data.json_root);
  if (!json_str)
    goto err;

  if (board_eeprom_write_new_prod_data(json_str) < 0)
    {
      free(json_str);
      goto err;
    }

  free(json_str);
  if (old_item)
    cJSON_Delete(old_item);
  return OK;

err:

  /* EEPROM store failed. Restore old item back to JSON object, so that
   * RAM does not contain different values than on EEPROM. */

  cJSON_DeleteItemFromObject(prod_data.json_prod, entry);

  if (old_item)
    cJSON_AddItemToObject(prod_data.json_prod, entry, old_item);

  return ERROR;
}

/****************************************************************************
 * Name: ts_fill_missing_prod_data
 *
 * Description:
 *   Fill missing production data for early prototypes.
 *
 ****************************************************************************/

void ts_fill_missing_prod_data(void)
{
  const struct device_information_s *devi;
  struct device_uid_s uid;
  char buf[16];

  /* Get device ID */

  if (!board_device_get_uid(&uid))
    return;

  if (ts_init_prod_data() == OK)
    {
      /* Check if production data already has PSN. */

      if (ts_device_prod_data_get_entry("psn", buf, sizeof(buf)) > 0)
        return; /* No need to fill PSN to EEPROM. */
    }

  devi = ts_device_identify(&uid);
  if (devi == NULL)
    return;

  /* Fill data to EEPROM. */

  ts_device_prod_data_set_entry("psn", devi->sn);

  snprintf(buf, sizeof(buf), "%04X", devi->hwid);
  ts_device_prod_data_set_entry("hwid", buf);
}

/****************************************************************************
 * Name: ts_device_identify
 *
 * Description:
 *   Get device information
 *
 ****************************************************************************/

struct device_information_s const * const
ts_device_identify(struct device_uid_s const * const uid)
{
#ifndef CONFIG_THINGSEE_DISABLE_DEVICE_INFO_TABLE
  uint32_t i;
#endif
  char buf[16];

  if (!uid)
    return NULL;

  /* First try load device information from EEPROM. */

  if (ts_device_prod_data_get_entry("psn", buf, sizeof(buf)) > 0)
    {
      static char psn[sizeof(buf)];
      static struct device_information_s devinfo;

      memcpy(psn, buf, sizeof(psn));
      devinfo.uid = *uid;
      devinfo.sn = psn;

      ts_device_prod_data_get_entry("hwid", buf, sizeof(buf));
      devinfo.hwid = strtol(buf, NULL, 16);

      return &devinfo;
    }

#ifndef CONFIG_THINGSEE_DISABLE_DEVICE_INFO_TABLE
  /* Then try mapping MCU UID to device information. */

  for (i = 0; i < ARRAY_SIZE(device_information); i++)
    {
      if (memcmp(&device_information[i].uid, uid, sizeof(struct device_uid_s)) == 0)
        {
          return &device_information[i];
        }
    }
#endif

  return NULL;
}

/****************************************************************************
 * Name: ts_device_is_prototype_before_B17
 *
 * Description:
 *   Detect early engineering prototype dynamically at run-time.
 *
 * Return -1 if detection failed. Very old prototypes don't even have HWID.
 *
 ****************************************************************************/
int ts_device_is_prototype_before_B17(void)
{
  char buf[16] = { 0 };
  long hwid;
  int ret;

  ret = ts_device_prod_data_get_entry("hwid", buf, sizeof(buf));
  if (!ret)
    {
      return -1;
    }

  /* TODO: This is wrong, assumes that device is ThingseeOne. Board type
   * checks should be in board level source. */

  errno = 0;
  hwid = strtol(buf, NULL, 16);
  if (hwid < 0 || errno != 0)
    {
      return -1;
    }
  else if (hwid < 0x0300)
   {
     return 1;
   }

  return 0;
}

