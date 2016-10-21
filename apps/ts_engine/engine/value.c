/****************************************************************************
 * apps/thingsee/engine/value.c
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "eng_dbg.h"
#include "value.h"
#include "parse_labels.h"

int __value_serialize(char *str, size_t size,
                      const struct ts_value * const value)
{
  const struct ts_value * array_value_entry;
  int ret = 0;
  int i;

  ret = snprintf(str, size, "%u:", value->valuetype);
  str += ret;
  size -= ret;

  switch (value->valuetype)
    {
    case VALUEDOUBLE:
#ifdef CONFIG_ARCH_SIM
      ret += snprintf(str, size, "%d", (int)(1000*sense_value->value.valuedouble));
#else
      ret += snprintf(str, size, "%.6f", value->valuedouble);
#endif
    break;

    case VALUEUINT16:
      ret += snprintf(str, size, "%u", value->valueuint16);
    break;

    case VALUEHEXSTRING:

      /* fall through */

    case VALUEUINT32:
      ret += snprintf(str, size, "%u", value->valueuint32);
    break;

    case VALUEINT16:
      ret += snprintf(str, size, "%d", value->valueint16);
    break;

    case VALUEINT32:
      ret += snprintf(str, size, "%d", value->valueint32);
    break;

    case VALUESTRING:
      ret += snprintf(str, size, "%s", value->valuestring);
    break;

    case VALUEBOOL:
      ret += snprintf(str, size, "%s", value->valuebool ? g_true_str : g_false_str);
    break;

    case VALUEARRAY:

      /* fall through */

    case VALUEARRAY_FIRSTSTRING:
      array_value_entry = value->valuearray.items;

      ret = snprintf(str, size, "[ ", value->valuetype);
      str += ret;
      size -= ret;

      for (i = 0; i < value->valuearray.number_of_items; i++)
        {
          ret = __value_serialize(str, size, &array_value_entry[i]);
          str += ret;
          size -= ret;

          if (i < (value->valuearray.number_of_items - 1))
            {
              ret = snprintf(str, size, " ");
            }
          else
            {
              ret = snprintf(str, size, " ]");
            }
          str += ret;
          size -= ret;
        }
    break;
    }

  return ret;
}

int __value_deserialize(char * value_str, struct ts_value * const value)
{
  int ret;
  char *array_str;
  char *token;
  char *saveptr;
  void *reallocptr;

  ret = sscanf(value_str, "%d:", &value->valuetype);
  if (ret != 1)
    {
      eng_dbg("sscanf for valuetype failed\n");
      return 0;
    }

  value_str = strchr(value_str, ':');
  value_str++;

  switch (value->valuetype)
    {
    case VALUEDOUBLE:
      value->valuedouble = strtod(value_str, NULL);
    break;

    case VALUEBOOL:
      value->valuebool = !strcmp(value_str, g_true_str);
    break;

    case VALUESTRING:
      value->valuestring = strdup(value_str);
      if (!value->valuestring)
        {
          ret = ERROR;
        }
    break;

    case VALUEINT16:
      value->valueint16 = strtol(value_str, NULL, 10);
    break;

    case VALUEUINT16:
      value->valueuint16 = strtoul(value_str, NULL, 10);
    break;

    case VALUEINT32:
      value->valueint32 = strtol(value_str, NULL, 10);
    break;

    case VALUEUINT32:
      value->valueuint32 = strtoul(value_str, NULL, 10);
    break;

    case VALUEARRAY:
      {
        struct ts_value *values = NULL;
        int items = 0;

        DEBUGASSERT(value_str[0] == '[' && value_str[1] == ' ');

        array_str = &value_str[2];
        token = strtok_r(array_str, " ", &saveptr);

        while (token)
          {
            if (token && token[0] == ']')
              {
                token = NULL;
              }

            if (token)
              {
                items++;

                reallocptr = realloc(values, items * sizeof(*values));
                if (!reallocptr)
                  {
                    eng_dbg("realloc %d failed\n", items * sizeof(*values));
                    free(values);
                    break;
                  }
                values = reallocptr;

                __value_deserialize(token, &values[items - 1]);
                token = strtok_r(NULL, " ", &saveptr);
              }
          }
        value->valuearray.items = values;
        value->valuearray.number_of_items = items;
      }
    break;

    default:
      eng_dbg("unhandled valuetype: %d\n", value->valuetype);
    }

  return ret;
}

/* TODO: check for mem alloc errors */

int __value_to_json(cJSON *obj, const char * const label,
                    const struct ts_value * const value)
{
  const struct ts_value * array_value_entry;

  switch (value->valuetype)
    {
    case VALUEDOUBLE:
      cJSON_AddNumberToObject(obj, label, value->valuedouble);
    break;

    case VALUEINT16:
      cJSON_AddNumberToObject(obj, label, value->valueint16);
    break;

    case VALUEINT32:
      cJSON_AddNumberToObject(obj, label, value->valueint32);
    break;

    case VALUEUINT16:
      cJSON_AddNumberToObject(obj, label, value->valueuint16);
    break;

    case VALUEUINT32:
      cJSON_AddNumberToObject(obj, label, value->valueuint32);
    break;

    case VALUEBOOL:
#if 1
      cJSON_AddNumberToObject(obj, label, value->valuebool);
#else
      if (value->valuebool)
        {
          cJSON_AddTrueToObject(obj, label);
        }
      else
        {
          cJSON_AddFalseToObject(obj, label);
        }
#endif
    break;

    case VALUESTRING:
      cJSON_AddStringToObject(obj, label, value->valuestring);
    break;

    case VALUEARRAY:
      {
        cJSON *array_json;
        cJSON *array_json_entry;
        int i;

        array_json = cJSON_CreateArray();
        if (!array_json)
          {
            eng_dbg("cJSON_CreateArray failed\n");
            return ERROR;
          }

        array_value_entry = value->valuearray.items;
        for (i = 0; i < value->valuearray.number_of_items; i++)
          {
            array_json_entry = cJSON_CreateObject();
            if (!array_json_entry)
              {
                eng_dbg("cJSON_CreateObject failed\n");
                cJSON_Delete(array_json);
                cJSON_Delete(obj);
                return ERROR;
              }
            __value_to_json(array_json_entry, label, &array_value_entry[i]);
            cJSON_AddItemToArray(array_json, array_json_entry);
          }
        cJSON_AddItemToObject(obj, label, array_json);
      }
    break;
    default:
      eng_dbg("Unhandled value type: %d\n", value->valuetype);
    break;
    }

  return OK;
}
