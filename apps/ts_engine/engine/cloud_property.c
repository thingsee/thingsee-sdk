/****************************************************************************
 * apps/ts_engine/engine/cloud_property.c
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

#include <sys/types.h>
#include <stdlib.h>
#include <assert.h>

#include <apps/netutils/cJSON.h>

#include "eng_dbg.h"
#include "util.h"

const char *
cloud_property_read(bool *ref)
{
#ifndef CONFIG_ARCH_SIM
  const char *properties;

  properties = __ts_engine_sdcard_read(SDCARD_CLOUD_PROPERTY_FILENAME);
  if (properties)
    {
      *ref = false;
      return properties;
    }

#ifndef CONFIG_THINGSEE_ENGINE_EEPROM
  return NULL;
#else
  *ref = true;
  return __ts_engine_eeprom_read(BOARD_EEPROM_SECTION_ENGINE_PROPERTY);
#endif
#else
  *ref = true;
  return "{}";
#endif
}

int cloud_property_write(char * const properties_json, size_t len)
{
  int ret;

  ret = __ts_engine_sdcard_write(SDCARD_CLOUD_PROPERTY_FILENAME,
      properties_json, len, false);
  if (ret == OK)
    {
      return OK;
    }

#ifndef CONFIG_THINGSEE_ENGINE_EEPROM
  return ERROR;
#else
  return __ts_engine_eeprom_write(BOARD_EEPROM_SECTION_ENGINE_PROPERTY,
      properties_json, len);
#endif
}

const char * const cloud_property_connectors(void)
{
  cJSON *cloud;
  cJSON *connectors;
  const char *cloud_str;
  const char *connectors_str = NULL;
  bool ref;

  cloud_str = cloud_property_read (&ref);
  if (!cloud_str)
    {
      eng_dbg ("cloud_properties_read failed\n");
      return NULL;
    }

  cloud = cJSON_Parse(cloud_str);
  if (!cloud)
    {
      eng_dbg("cJSON_Parse failed\n");
      goto errout_free_cloud_str;
    }

  connectors = cJSON_GetObjectItem(cloud, "connectors");
  if (!connectors)
    {
      eng_dbg("no connectors\n");
      goto errout_free_cloud;
    }

  connectors_str = cJSON_PrintUnformatted(connectors);

errout_free_cloud:
  cJSON_Delete(cloud);
errout_free_cloud_str:
  if (!ref)
    {
      free((void *)cloud_str);
    }

  return connectors_str;
}

const char * const cloud_property_connections(void)
{
  cJSON *cloud;
  cJSON *connections;
  const char *cloud_str;
  const char *connections_str = NULL;
  bool ref;

  cloud_str = cloud_property_read (&ref);
  if (!cloud_str)
    {
      eng_dbg ("cloud_properties_read failed\n");
      return NULL;
    }

  cloud = cJSON_Parse(cloud_str);
  if (!cloud)
    {
      eng_dbg("cJSON_Parse failed\n");
      goto errout_free_cloud_str;
    }

  connections = cJSON_GetObjectItem(cloud, "connections");
  if (!connections)
    {
      eng_dbg("no connections\n");
      goto errout_free_cloud;
    }

  connections_str = cJSON_PrintUnformatted(connections);

  errout_free_cloud:
    cJSON_Delete(cloud);
  errout_free_cloud_str:
    if (!ref)
      {
        free((void *)cloud_str);
      }

 return connections_str;
}

uint32_t cloud_property_sms(const char *key, uint32_t default_val)
{
  cJSON *cloud;
  cJSON *sms;
  cJSON *val;
  const char *cloud_str;
  bool ref;
  uint32_t ret_val = default_val;

  cloud_str = cloud_property_read (&ref);
  if (!cloud_str)
    {
      eng_dbg ("cloud_properties_read failed\n");
      return default_val;
    }

  cloud = cJSON_Parse(cloud_str);
  if (!cloud)
    {
      eng_dbg("cJSON_Parse failed\n");
      goto errout_free_cloud_str;
    }

  sms = cJSON_GetObjectItem(cloud, "sms");
  if (!sms)
    {
      eng_dbg("no sms\n");
      goto errout_free_cloud;
    }

  val = cJSON_GetObjectItem(sms, key);
  if (!val)
    {
      eng_dbg("no '%s' in sms\n", key);
      goto errout_free_cloud;
    }

  if (cJSON_type(val) == cJSON_Number)
    {
      ret_val = cJSON_int(val);
    }
  else
    {
      eng_dbg("value '%s' in sms is not a number\n", key);
      goto errout_free_cloud;
    }

  errout_free_cloud:
    cJSON_Delete(cloud);
  errout_free_cloud_str:
    if (!ref)
      {
        free((void *)cloud_str);
      }

 return ret_val;
}
