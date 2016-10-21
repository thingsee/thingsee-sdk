/****************************************************************************
 * apps/ts_engine/engine/device_property.c
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

#include <nuttx/config.h>
#include <nuttx/version.h>

#include <sys/types.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>

#include <arch/board/board-eeprom.h>
#include <apps/netutils/cJSON.h>

#include "eng_dbg.h"
#include "parse.h"
#include "sense.h"
#include "eng_error.h"

#define STRLABEL(x) static const char const g_##x##_str[] = #x

STRLABEL(device);
STRLABEL(swName);
STRLABEL(swVersion);
STRLABEL(hwVersion);
STRLABEL(serial);
STRLABEL(module);

struct prod_data
{
  cJSON *json_root;
  cJSON *json_prod;
};

static struct prod_data g_prod_data = { 0 };

static int load_prod_data(bool backup)
{
  const char *prod_str;
  size_t prod_strlen;
  cJSON *json_root;
  cJSON *json_prod;

  prod_str = (
      !backup ? board_eeprom_get_prod_data : board_eeprom_get_prod_data_backup)(
      &prod_strlen);
  if (!prod_str || prod_strlen != strlen(prod_str))
    {
      return ERROR;
    }

  json_root = cJSON_Parse(prod_str);
  if (!json_root)
    {
      eng_dbg("Production data json parse failed\n");
      return ERROR;
    }

  json_prod = cJSON_GetObjectItem(json_root, "prod_info");
  if (!json_prod)
    {
      eng_dbg("prod_info object not found\n");
      cJSON_Delete(json_root);
      return ERROR;
    }

  g_prod_data.json_root = json_root;
  g_prod_data.json_prod = json_prod;

  return OK;
}

static void prod_data_get_entry(const char *entry, char *buf, size_t buflen)
{
  cJSON *item;
  int ret;

  if (!g_prod_data.json_prod)
    {
      ret = load_prod_data(false);
      if (ret < 0)
        {
          eng_dbg("primary prod data load failed\n");

          ret = load_prod_data(true);
          if (ret < 0)
            {
              eng_dbg("backup prod data load failed\n");

              snprintf(buf, buflen, "%s", "n/a");
              return;
            }
        }
    }

  item = cJSON_GetObjectItem(g_prod_data.json_prod, entry);
  if (!item)
    {
      snprintf(buf, buflen, "%s", "n/a");
    }
  else
    {
      switch (cJSON_type(item))
        {
        case cJSON_Number:
          snprintf(buf, buflen, "%d", cJSON_int(item));
        break;
        case cJSON_String:
          snprintf(buf, buflen, "%s", cJSON_string(item));
        break;
        default:
        break;
        }
    }
}

static void free_prod_data(void)
{
  cJSON_Delete(g_prod_data.json_root);
}

int device_property_get(const char **device_property)
{
  cJSON *root;
  cJSON *device;
  cJSON *module;
  char buf[32];

  memset(&g_prod_data, 0, sizeof(g_prod_data));

  root = cJSON_CreateObject();
  if (!root)
    {
      eng_dbg("root object creation failed\n");
      return -TS_ENGINE_ERROR_OOM;
    }

  device = cJSON_CreateObject();
  if (!device)
    {
      eng_dbg("device object creationg failed\n");
      cJSON_Delete(root);
      return -TS_ENGINE_ERROR_OOM;
    }

  cJSON_AddStringToObject(device, g_swName_str, "Thingsee SW");
  cJSON_AddStringToObject(device, g_swVersion_str, CONFIG_VERSION_BUILD);
  prod_data_get_entry("hwid", buf, sizeof(buf));
  cJSON_AddStringToObject(device, g_hwVersion_str, buf);
  prod_data_get_entry("psn", buf, sizeof(buf));
  cJSON_AddStringToObject(device, g_serial_str, buf);
  cJSON_AddItemToObject(root, g_device_str, device);
  free_prod_data();

  module = generate_module_json();
  if (module)
    {
      cJSON_AddItemToObject(root, g_module_str, module);
    }

  *device_property = cJSON_Print(root);

  cJSON_Delete(root);

  return OK;
}
