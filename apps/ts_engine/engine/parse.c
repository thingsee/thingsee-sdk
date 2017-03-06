/****************************************************************************
 * apps/thingsee/engine/engine_profile.c
 *
 * Copyright (C) 2014-2016 Haltian Ltd. All rights reserved.
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
#include <string.h>
#include <stdio.h>

#include <queue.h>
#include <debug.h>

#include <apps/netutils/cJSON.h>

#include "eng_dbg.h"
#include "parse.h"
#include "sense.h"
#include "eng_error.h"
#include "execute.h"

#ifndef offsetof
#define offsetof(type, member)   ( (size_t) &( ( (type *) 0)->member))
#endif

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

const char *threshold_types[] =
  {
      [isOneOf] = g_isOneOf_str,
      [isGt] = g_isGt_str,
      [isLt] = g_isLt_str,
      [isNotIn] = g_isNotIn_str,
      [isAny] = g_isAny_str,
      [isInsideGeo] = g_isInsideGeo_str,
      NULL,
  };

struct ts_parse
{
  const char * const label;
  size_t offset;
  enum ts_valuetype_t valuetype;
  bool required;
  union
  {
    int error;
    struct ts_value value;
  };
};

static int
get_json_valuestring (cJSON * parent, const char * const label,
		      char ** const value)
{
  cJSON *object;

  *value = NULL;

  if (!parent)
    {
      return ERROR;
    }

  object = cJSON_GetObjectItem (parent, label);
  if (!object)
    {
      return ERROR;
    }

  *value = strdup (cJSON_string(object));

  return OK;
}

static int
get_json_valuedouble (cJSON * parent, const char * const label,
		      double * const value)
{
  cJSON *object;

  if (!parent)
    {
      return ERROR;
    }

  object = cJSON_GetObjectItem (parent, label);
  if (!object)
    {
      return ERROR;
    }

  *value = cJSON_double(object);

  return OK;
}

static int
get_json_valueuint16 (cJSON * parent, const char * const label,
		     uint16_t * const value)
{
  cJSON *object;

  if (!parent)
    {
      return ERROR;
    }

  object = cJSON_GetObjectItem (parent, label);
  if (!object)
    {
      return ERROR;
    }

  *value = (uint16_t)cJSON_int(object);

  return OK;
}

static int
get_json_valueuint32 (cJSON * parent, const char * const label,
		     uint32_t * const value)
{
  cJSON *object;

  if (!parent)
    {
      return ERROR;
    }

  object = cJSON_GetObjectItem (parent, label);
  if (!object)
    {
      return ERROR;
    }

  *value = (uint32_t)cJSON_int(object);

  return OK;
}

static int
get_json_valueint16 (cJSON * parent, const char * const label,
		     int16_t * const value)
{
  cJSON *object;

  if (!parent)
    {
      return ERROR;
    }

  object = cJSON_GetObjectItem (parent, label);
  if (!object)
    {
      return ERROR;
    }

  *value = (int16_t)cJSON_int(object);

  return OK;
}

static int
get_json_valueint32 (cJSON * parent, const char * const label,
		     int32_t * const value)
{
  cJSON *object;

  if (!parent)
    {
      return ERROR;
    }

  object = cJSON_GetObjectItem (parent, label);
  if (!object)
    {
      return ERROR;
    }

  *value = (int32_t)cJSON_int(object);

  return OK;
}

static int
get_json_valuebool (cJSON * parent, const char * const label,
                    bool * const value)
{
  cJSON *object;

  if (!parent)
    {
      return ERROR;
    }

  object = cJSON_GetObjectItem (parent, label);
  if (!object)
    {
      return ERROR;
    }

  *value = cJSON_boolean(object);

  return OK;
}

static int
parse_json_object (cJSON *object, void *data, const struct ts_parse *parse, int items)
{
  int i;
  int j;
  int ret;
  char *field;

  for (i = 0; i < items; i++)
    {
      field = (char *) data + parse[i].offset;

      switch (parse[i].valuetype)
      {
        case VALUEDOUBLE:
          {
            double *value = (double *) field;

            ret = get_json_valuedouble (object, parse[i].label, value);
            if (ret != OK)
              {
                if (parse[i].required)
                  {
                    eng_dispdbg ("required value \"%s\" missing", parse[i].label);
                    return parse[i].error;
                  }
                else
                  {
                    *value = parse[i].value.valuedouble;
                  }
              }
          }
          break;

        case VALUEUINT16:
          {
            uint16_t *value = (uint16_t *) field;

            ret = get_json_valueuint16 (object, parse[i].label, value);
            if (ret != OK)
              {
                if (parse[i].required)
                  {
                    eng_dispdbg ("required value \"%s\" missing", parse[i].label);
                    return parse[i].error;
                  }
                else
                  {
                    *value = parse[i].value.valueuint16;
                  }
              }
          }
          break;

        case VALUEUINT32:
          {
            uint32_t *value = (uint32_t *) field;

            ret = get_json_valueuint32 (object, parse[i].label, value);
            if (ret != OK)
              {
                if (parse[i].required)
                  {
                    eng_dispdbg ("required value \"%s\" missing", parse[i].label);
                    return parse[i].error;
                  }
                else
                  {
                    *value = parse[i].value.valueuint32;
                  }
              }
          }
          break;

        case VALUEINT16:
          {
            int16_t *value = (int16_t *) field;

            ret = get_json_valueint16 (object, parse[i].label, value);
            if (ret != OK)
              {
                if (parse[i].required)
                  {
                    eng_dispdbg ("required value \"%s\" missing", parse[i].label);
                    return parse[i].error;
                  }
                else
                  {
                    *value = parse[i].value.valueint16;
                  }
              }
          }
          break;

        case VALUEINT32:
          {
            int32_t *value = (int32_t *) field;

            ret = get_json_valueint32 (object, parse[i].label, value);
            if (ret != OK)
              {
                if (parse[i].required)
                  {
                    eng_dispdbg ("required value \"%s\" missing", parse[i].label);
                    return parse[i].error;
                  }
                else
                  {
                    *value = parse[i].value.valueint32;
                  }
              }
          }
          break;

        case VALUESTRING:
          {
            char **value = (char **) field;

            ret = get_json_valuestring (object, parse[i].label, value);
            if (ret != OK)
              {
                if (parse[i].required)
                  {
                    eng_dispdbg ("required value \"%s\" missing", parse[i].label);
                    return parse[i].error;
                  }
                else
                  {
                    *value = strdup (parse[i].value.valuestring);
                  }
              }
          }
          break;

        case VALUEBOOL:
          {
            bool *value = (bool *) field;

            ret = get_json_valuebool (object, parse[i].label, value);
            if (ret != OK)
              {
                if (parse[i].required)
                  {
                    eng_dispdbg ("required value \"%s\" missing", parse[i].label);
                    return parse[i].error;
                  }
                else
                  {
                    *value = parse[i].value.valuebool;
                  }
              }
          }
          break;

        case VALUEARRAY: /* cJSON_NULL, cJSON_Array, cJSON_Object items not supported */
          {
            struct ts_value *value = (struct ts_value *) field;
            cJSON *array;
            cJSON *obj;

            value->valuearray.number_of_items = 0;
            ret = ERROR;

            array = cJSON_GetObjectItem (object, parse[i].label);
            if (array && cJSON_type(array) == cJSON_Array)
              {
                value->valuetype = VALUEARRAY;
                value->valuearray.number_of_items = cJSON_GetArraySize(array);
                value->valuearray.items = calloc (1, value->valuearray.number_of_items * sizeof(*value->valuearray.items));
                ret = OK;

                for (j = 0; j < value->valuearray.number_of_items; j++)
                  {
                    obj = cJSON_GetArrayItem (array, j);
                    if (obj)
                      {
                        int type = cJSON_type(obj);
                        if (type == cJSON_False || type == cJSON_True)
                          {
                            value->valuearray.items[j].valuetype = VALUEBOOL;
                            value->valuearray.items[j].valuebool = cJSON_boolean(obj);
                          }
                        else if (type == cJSON_Number)
                          {
                            value->valuearray.items[j].valuetype = VALUEDOUBLE;
                            value->valuearray.items[j].valuedouble = cJSON_double(obj);
                          }
                        else if (type == cJSON_String)
                          {
                            value->valuearray.items[j].valuetype = VALUESTRING;
                            value->valuearray.items[j].valuestring = strdup(cJSON_string(obj));
                            if (!value->valuearray.items[j].valuestring)
                              {
                                eng_dbg("stdup failed\n");
                                ret = ERROR;
                                value->valuearray.number_of_items = j;
                              }
                          }
                        else
                          {
                            eng_dbg("invalid cJSON object type: %d\n", type);
                            ret = ERROR;
                            value->valuearray.number_of_items = j;
                          }
                      }
                  }
              }

            if (ret != OK)
              {
                for (j = 0; j < value->valuearray.number_of_items; j++)
                  {
                    if (value->valuearray.items[j].valuetype == VALUESTRING)
                      {
                        free (value->valuearray.items[j].valuestring);
                      }
                  }
                free (value->valuearray.items);

                if (parse[i].required)
                  {
                    eng_dispdbg ("required value \"%s\" missing", parse[i].label);
                    return parse[i].error;
                  }
                else
                  {
                    memcpy(value, &parse[i].value.valuearray, sizeof(parse[i].value.valuearray));
                  }
              }
          }
          break;

        case VALUEARRAY_FIRSTSTRING:
          {
            char **value = (char **) field;
            cJSON *array;
            cJSON *obj;

            *value = NULL;
            ret = ERROR;

            array = cJSON_GetObjectItem (object, parse[i].label);
            if (array && cJSON_type(array) == cJSON_Array)
              {
                obj = cJSON_GetArrayItem(array, 0);
                if (obj && cJSON_type(obj) == cJSON_String)
                  {
                    *value = strdup(cJSON_string(obj));
                    ret = OK;
                  }
              }

            if (ret != OK)
              {
                if (parse[i].required)
                  {
                    eng_dispdbg ("required value \"%s\" missing", parse[i].label);
                    return parse[i].error;
                  }
                else
                  {
                    *value = strdup (parse[i].value.valuestring);
                  }
              }
          }
          break;

        case VALUEHEXSTRING:
          {
            uint32_t *value = (uint32_t *)field;
            char *hexstring;

            ret = get_json_valuestring (object, parse[i].label, &hexstring);
            if (ret != OK)
              {
                if (parse[i].required)
                  {
                    eng_dispdbg ("required value \"%s\" missing", parse[i].label);
                    return parse[i].error;
                  }
                else
                  {
                    *value = parse[i].value.valueuint32;
                  }
              }
            else if (hexstring != NULL)
              {
                sscanf(hexstring, "0x%x", value);
                free (hexstring);
              }
            else
              {
                *value = parse[i].value.valueuint32;
              }
          }
          break;

        default:
          DEBUGASSERT(false);
          break;
      }
    }
  return OK;
}

static void
set_threshold_value (cJSON *thr, struct ts_value *value)
{
  switch (cJSON_type(thr))
  {
    case cJSON_String:
      value->valuetype = VALUESTRING;
      value->valuestring = strdup(cJSON_string(thr));
      break;

    case cJSON_Number:
      value->valuetype = VALUEDOUBLE;
      value->valuedouble = cJSON_double(thr);
      break;

    case cJSON_True:
    case cJSON_False:
      value->valuetype = VALUEBOOL;
      value->valuebool = cJSON_boolean(thr);
      break;

    default:
      value->valuetype = VALUEBOOL;
      value->valuebool = false;
      break;
  }
}

static int
add_threshold (cJSON *json_threshold, struct ts_cause *cause, int type)
{
  struct ts_threshold *threshold;

  threshold = malloc (sizeof(struct ts_threshold));
  if (!threshold)
    {
      eng_dispdbg ("malloc for ts_threshold failed");
      return -PROFILE_ERROR_OUT_OF_MEMORY;
    }

  threshold->conf.check_threshold = NULL;

  switch (cJSON_type(json_threshold))
    {
    case cJSON_Array:
      {
        int n;
        int i;
        cJSON *item;
        struct ts_value *items;

        if (type != isOneOf && type != isNotIn && type != isInsideGeo)
          {
            eng_dispdbg ("invalid threshold");
            free(threshold);
            return -PROFILE_ERROR_INVALID_THRESHOLD;
          }

        n = cJSON_GetArraySize (json_threshold);
        if (n == 0)
          {
            eng_dispdbg ("invalid threshold");
            free(threshold);
            return -PROFILE_ERROR_INVALID_THRESHOLD;
          }

        items = malloc (n * sizeof(*items));
        if (!items)
          {
            eng_dbg("malloc %d failed\n", n * sizeof(*items));
            free(threshold);
            return -TS_ENGINE_ERROR_OOM;
          }

        threshold->conf.value.valuetype = VALUEARRAY;
        threshold->conf.value.valuearray.items = items;
        threshold->conf.value.valuearray.number_of_items = n;

        if (type == isInsideGeo)
          {
            threshold->conf.check_threshold = __ts_engine_check_geofence;
          }

        for (i = 0; i < n; i++)
          {
            item = cJSON_GetArrayItem (json_threshold, i);
            set_threshold_value (item, &items[i]);
          }
      }
      break;

    default:
      set_threshold_value (json_threshold, &threshold->conf.value);
    }

  threshold->conf.type = type;
  threshold->parent = cause;
  sq_addlast (&threshold->entry, &cause->conf.thresholds);

  return OK;
}

static int
init_threshold (cJSON *json_thresholds, struct ts_cause *cause, int type)
{
  cJSON *json_threshold;

  json_threshold = cJSON_GetObjectItem (json_thresholds, threshold_types[type]);
  if (!json_threshold)
    {
      return OK;
    }

  return add_threshold (json_threshold, cause, type);
}

static int
init_thresholds (cJSON *json_thresholds, struct ts_cause *cause)
{
  int n = 0;
  int ret;

  sq_init(&cause->conf.thresholds);

  if (!json_thresholds)
    {
      eng_dispdbg ("no thresholds");
      return OK;
    }

  while (threshold_types[n])
    {
      ret = init_threshold (json_thresholds, cause, n);
      if (ret < 0)
	{
	  return ret;
	}

      n++;
    }

  return OK;
}

static int
init_causes (cJSON *causes, struct ts_event *event)
{
  int i;
  int ret;
  cJSON *json_measurement;
  cJSON *json_threshold;

  sq_init(&event->conf.causes);

  if (!causes)
    {
      eng_dispdbg ("no causes");
      return -PROFILE_ERROR_NO_CAUSES;
    }

  event->conf.number_of_causes = cJSON_GetArraySize (causes);

  for (i = 0; i < event->conf.number_of_causes; i++)
    {
      cJSON *json_cause = cJSON_GetArrayItem (causes, i);
      struct ts_cause *cause = calloc (1, sizeof(struct ts_cause));
      if (!cause)
	{
	  eng_dispdbg ("malloc for struct ts_cause failed");
	  return -PROFILE_ERROR_OUT_OF_MEMORY;
	}

      cause->parent = event;
      cause->dyn.fd = -1;
      cause->dyn.timer_id = -1;
      sq_addlast (&cause->entry, &event->conf.causes);

      static const struct ts_parse parse_cause[] =
	{
	  { g_sId_str, offsetof(struct ts_cause, dyn.sense_value.sId),
	      VALUEHEXSTRING, true, .error = -PROFILE_ERROR_SENSE_ID_NOT_SET },
	  { g_orderId_str, offsetof(struct ts_cause, conf.orderId), VALUEINT32,
	      false, .value = { VALUEINT32, .valueint32 = 0 }, },
	  { g_senseLog_str, offsetof(struct ts_cause, conf.senseLog), VALUEINT32,
	      false, .value = { VALUEINT32, .valueint32 = false }, },
	};

      ret = parse_json_object (json_cause, cause, parse_cause,
			       ARRAY_SIZE(parse_cause));
      if (ret != OK)
	{
	  eng_dispdbg ("cause parse failed");
	  return ret;
	}

      json_measurement = cJSON_GetObjectItem (json_cause, g_measurement_str);

      static const struct ts_parse parse_measurement[] =
	  {
	      { g_log_str, offsetof(struct ts_cause, conf.measurement.log),
		  VALUEBOOL, false, .value = { VALUEBOOL, .valuebool = false }, },
	      { g_interval_str, offsetof(struct ts_cause, conf.measurement.interval),
		  VALUEINT32, false, .value = { VALUEINT32, .valueint32 = -1 }, },
	      { g_count_str, offsetof(struct ts_cause, conf.measurement.count),
		  VALUEUINT32, false, .value = { VALUEINT32, .valueint32 = 0 }, },
	      { g_send_str, offsetof(struct ts_cause, conf.measurement.send),
		  VALUEBOOL, false, .value = { VALUEBOOL, .valuebool = false }, },
	  };

      ret = parse_json_object (json_measurement, cause, parse_measurement,
			       ARRAY_SIZE(parse_measurement));
      if (ret != OK)
	{
	  eng_dispdbg ("measurement parse failed");
	  return ret;
	}

      json_threshold = cJSON_GetObjectItem (json_cause, g_threshold_str);
      static const struct ts_parse parse_threshold[] =
	  {
	      { g_count_str, offsetof(struct ts_cause, conf.threshold.count),
		  VALUEUINT32, false, .value = { VALUEINT32, .valueint32 = 0 }, },
	      { g_negate_str, offsetof(struct ts_cause, conf.threshold.negate),
		  VALUEBOOL, false, .value = { VALUEBOOL, .valuebool = false }, },
	      { g_relative_str, offsetof(struct ts_cause, conf.threshold.relative),
		  VALUEBOOL, false, .value = { VALUEBOOL, .valuebool = false }, },

	  };

      ret = parse_json_object (json_threshold, cause, parse_threshold,
			       ARRAY_SIZE(parse_threshold));
      if (ret != OK)
	{
	  eng_dispdbg ("threshold parse failed");
	  return ret;
	}

      ret = init_thresholds (
	  cJSON_GetObjectItem (json_cause, g_thresholds_str), cause);
      if (ret != OK)
	{
	  eng_dispdbg ("init_thresholds failed");
	  return ret;
	}

      cause->dyn.sense_info = get_sense_info (cause->dyn.sense_value.sId);
      if (!cause->dyn.sense_info)
	{
	  eng_dispdbg ("invalid sId: 0x%08x", cause->dyn.sense_value.sId);
	  return -PROFILE_ERROR_INVALID_SENSE_ID;
	}

      /* Set the name pointer here for easy payload generation */

      cause->dyn.sense_value.name = cause->dyn.sense_info->name;

      if (cause->conf.measurement.interval == -1 &&
          !cause->dyn.sense_info->ops.irq.init)
        {
          eng_dispdbg ("no interval no irq 0x%08x\n", cause->dyn.sense_info->sId);
          return -PROFILE_ERROR_NO_INTERVAL_NO_IRQ;
        }

    }

  return OK;
}

static int
init_events (cJSON *events, struct ts_state *state)
{
  int number_of_events;
  int i;
  int ret;
  cJSON *json_actions;
  cJSON *json_sms;
  cJSON *json_cloud;
  cJSON *json_engine;
  cJSON *json_display;

  sq_init(&state->conf.events);

  if (!events)
    {
      eng_dispdbg ("no events");
      return -PROFILE_ERROR_NO_EVENTS;
    }

  number_of_events = cJSON_GetArraySize (events);

  for (i = 0; i < number_of_events; i++)
    {
      cJSON *json_event = cJSON_GetArrayItem (events, i);
      struct ts_event *event = calloc (1, sizeof(struct ts_event));
      if (!event)
	{
	  eng_dispdbg ("malloc for struct ts_event failed");
	  return -PROFILE_ERROR_OUT_OF_MEMORY;
	}

      event->parent = state;
      sq_addlast (&event->entry, &state->conf.events);

      static const struct ts_parse parse_event[] =
	{
	  { g_evId_str, offsetof(struct ts_event, conf.evId), VALUEINT32,
	      true, .error = -PROFILE_ERROR_EVENT_ID_NOT_SET },
	  { g_name_str, offsetof(struct ts_event, conf.name), VALUESTRING,
	      false, .value = { VALUESTRING, .valuestring = NULL }, },
	  { g_eventLog_str, offsetof(struct ts_event, conf.eventLog), VALUEINT32,
	      false, .value = { VALUEINT32, .valueint32 = false }, },
	};

      ret = parse_json_object (json_event, event, parse_event,
			       ARRAY_SIZE(parse_event));
      if (ret != OK)
	{
	  eng_dispdbg ("event parse failed");
	  return ret;
	}

      json_actions = cJSON_GetObjectItem (json_event, g_actions_str);
      if (!json_actions)
	{
	  eng_dispdbg ("%s not set", g_actions_str);
	  return -PROFILE_ERROR_ACTIONS_NOT_SET;
	}

      json_sms = cJSON_GetObjectItem (json_actions, g_sms_str);
      if (json_sms)
        {
          static const struct ts_parse parse_sms[] =
              {
                  { g_text_str, offsetof(struct ts_event, conf.actions.sms.text),
                    VALUESTRING, true, .error = -PROFILE_ERROR_SMS_TEXT_NOT_SET },
                  { g_phoneNumber_str, offsetof(struct ts_event, conf.actions.sms.phoneNumber),
                    VALUEARRAY_FIRSTSTRING, true, .error = -PROFILE_ERROR_SMS_PHONE_NUMBER_NOT_SET },
              };

          ret = parse_json_object (json_sms, event, parse_sms,
                                   ARRAY_SIZE(parse_sms));
          if (ret != OK)
            {
              eng_dispdbg ("sms parse failed");
              return ret;
            }
        }
      else
        {
          /* "sms" not mandatory. */

          event->conf.actions.sms.text = NULL;
          event->conf.actions.sms.phoneNumber = NULL;
        }

      json_cloud = cJSON_GetObjectItem (json_actions, g_cloud_str);
      if (json_cloud)
        {
          static const struct ts_parse parse_cloud[] =
              {
                  { g_sendEvent_str, offsetof(struct ts_event, conf.actions.cloud.sendEvent),
                      VALUEBOOL, false, .value = { VALUEBOOL, .valuebool = false }, },
                  { g_sendLog_str, offsetof(struct ts_event, conf.actions.cloud.sendLog),
                      VALUEBOOL, false, .value = { VALUEBOOL, .valuebool = false }, },
                  { g_sendPush_str, offsetof(struct ts_event, conf.actions.cloud.sendPush),
                      VALUEBOOL, false, .value = { VALUEBOOL, .valuebool = false }, },
                  { g_host_str, offsetof(struct ts_event, conf.actions.cloud.url.host),
                      VALUESTRING, false, .value = { VALUESTRING, .valuestring = NULL }, },
                  { g_port_str, offsetof(struct ts_event, conf.actions.cloud.url.port),
                      VALUEUINT16, false, .value = { VALUEUINT16, .valueuint16 = 80 }, },
                  { g_api_str, offsetof(struct ts_event, conf.actions.cloud.url.api),
                      VALUESTRING, false, .value = { VALUESTRING, .valuestring = NULL }, },
                  { g_httpHeader_str, offsetof(struct ts_event, conf.actions.cloud.url.http_header),
                      VALUEARRAY, false, .value = { VALUEARRAY, .valuearray = { 0, NULL }, }, },
              };

          ret = parse_json_object (json_cloud, event, parse_cloud,
                                   ARRAY_SIZE(parse_cloud));
          if (ret != OK)
            {
              eng_dispdbg ("cloud parse failed");
              return ret;
            }

          if (event->conf.actions.cloud.url.host)
            {
              event->conf.actions.cloud.url.srv_ip4addr.sin_addr.s_addr = 0;
            }
        }
      else
        {
          /* "cloud" not mandatory. */

          event->conf.actions.cloud.sendEvent = false;
          event->conf.actions.cloud.sendLog = false;
          event->conf.actions.cloud.sendPush = false;
          event->conf.actions.cloud.url.host = NULL;
          event->conf.actions.cloud.url.port = -1;
          event->conf.actions.cloud.url.api = NULL;
          event->conf.actions.cloud.url.http_header.valuearray.items = NULL;
          event->conf.actions.cloud.url.http_header.valuearray.number_of_items = 0;
        }

      json_engine = cJSON_GetObjectItem (json_actions, g_engine_str);
      if (json_engine)
        {
          static const struct ts_parse parse_engine[] =
              {
                  { g_gotoStId_str, offsetof(struct ts_event, conf.actions.engine.gotoStId),
                      VALUEUINT32, false, .value = { VALUEINT32, .valueint32 = -1 }, },
                  { g_gotoPuId_str, offsetof(struct ts_event, conf.actions.engine.gotoPuId),
                      VALUEUINT32, false, .value = { VALUEINT32, .valueint32 = -1 }, },
              };

          ret = parse_json_object (json_engine, event, parse_engine,
                                   ARRAY_SIZE(parse_engine));
          if (ret != OK)
            {
              eng_dispdbg ("engine parse failed");
              return ret;
            }
        }
      else
        {
          /* "engine" not mandatory. */

          event->conf.actions.engine.gotoPuId = -1;
          event->conf.actions.engine.gotoStId = -1;
        }

      json_display = cJSON_GetObjectItem (json_actions, g_display_str);
      if (json_display)
        {
          static const struct ts_parse parse_display[] =
              {
                  { g_text_str, offsetof(struct ts_event, conf.actions.display.showText),
                    VALUESTRING, true, .error = -PROFILE_ERROR_SHOW_TEXT_NOT_SET },
              };

          ret = parse_json_object (json_display, event, parse_display,
                                   ARRAY_SIZE(parse_display));
          if (ret != OK)
            {
              eng_dispdbg ("display parse failed");
              return ret;
            }
        }
      else
        {
          /* "display" not mandatory. */

          event->conf.actions.display.showText = NULL;
        }

      ret = init_causes (cJSON_GetObjectItem (json_event, g_causes_str),
			 event);
      if (ret != OK)
	{
	  eng_dispdbg ("init_causes failed");
	  return ret;
	}
    }

  return OK;
}

static int
init_states (cJSON *states, struct ts_purpose *purpose)
{
  int number_of_states;
  int i;
  int ret;

  sq_init(&purpose->conf.states);

  if (!states)
    {
      eng_dispdbg ("no states");
      return -PROFILE_ERROR_NO_STATES;
    }

  number_of_states = cJSON_GetArraySize (states);

  for (i = 0; i < number_of_states; i++)
    {
      cJSON *json_state = cJSON_GetArrayItem (states, i);
      struct ts_state *state = calloc (1, sizeof(struct ts_state));
      if (!state)
	{
	  eng_dispdbg ("malloc for struct ts_state failed");
	  return -PROFILE_ERROR_OUT_OF_MEMORY;
	}

      state->parent = purpose;
      sq_addlast (&state->entry, &purpose->conf.states);

      static const struct ts_parse parse_state[] =
	{
	  { g_stId_str, offsetof(struct ts_state, conf.stId), VALUEINT32,
	      true, .error = -PROFILE_ERROR_STATE_ID_NOT_SET },
	  { g_name_str, offsetof(struct ts_state, conf.name), VALUESTRING,
	      false, .value = { VALUESTRING, .valuestring = NULL }, },
	  { g_isGlobal_str, offsetof(struct ts_state, conf.isGlobal), VALUEINT32,
	      false, .value = { VALUEINT32, .valueint32 = false }, },
	};

      ret = parse_json_object (json_state, state, parse_state,
			       ARRAY_SIZE(parse_state));
      if (ret != OK)
	{
	  eng_dispdbg ("state parse failed");
	  return ret;
	}

      ret = init_events (cJSON_GetObjectItem (json_state, g_events_str),
			 state);
      if (ret != OK)
	{
	  eng_dbg ("init_events failed\n");
	  return ret;
	}
    }

  return OK;
}

static int
init_purposes (cJSON *purposes, struct ts_profile *profile)
{
  int number_of_purposes;
  int i;
  int ret;

  sq_init(&profile->conf.purposes);

  if (!purposes)
    {
      eng_dispdbg ("no states");
      return -PROFILE_ERROR_NO_STATES;
    }

  number_of_purposes = cJSON_GetArraySize (purposes);

  for (i = 0; i < number_of_purposes; i++)
    {
      cJSON *json_purpose = cJSON_GetArrayItem (purposes, i);
      struct ts_purpose *purpose = calloc (1, sizeof(struct ts_purpose));
      if (!purpose)
	{
	  eng_dispdbg ("malloc for struct ts_purpose failed");
	  return -PROFILE_ERROR_OUT_OF_MEMORY;
	}

      purpose->parent = profile;
      sq_addlast (&purpose->entry, &profile->conf.purposes);

      static const struct ts_parse parse_purpose[] =
	    {
	      { g_puId_str, offsetof(struct ts_purpose, conf.puId),
		  VALUEINT32, true, .error = -PROFILE_ERROR_PURPOSE_ID_NOT_SET },
	      { g_name_str, offsetof(struct ts_purpose, conf.name),
		  VALUESTRING, false, .value = { VALUESTRING, .valuestring = NULL }, },
	      { g_initStId_str, offsetof(struct ts_purpose, conf.initStId),
		  VALUEINT32, true, .error = -PROFILE_ERROR_INIT_STATE_ID_NOT_SET },
	    };

      ret = parse_json_object (json_purpose, purpose, parse_purpose,
			       ARRAY_SIZE(parse_purpose));
      if (ret != OK)
	{
	  eng_dispdbg ("purpose parse failed");
	  return ret;
	}

      ret = init_states (cJSON_GetObjectItem (json_purpose, g_states_str),
			 purpose);
      if (ret != OK)
	{
	  eng_dispdbg ("init_states failed");
	  return ret;
	}
    }

  return OK;
}

void
profile_free (struct ts_profile *profile)
{
  struct ts_purpose *purpose, *next_purpose;
  struct ts_state *state, *next_state;
  struct ts_event *event, *next_event;
  struct ts_cause *cause, *next_cause;
  struct ts_threshold *threshold, *next_threshold;
  int i;

  purpose = (struct ts_purpose *) sq_peek(&profile->conf.purposes);
  while (purpose)
    {
      state = (struct ts_state *) sq_peek(&purpose->conf.states);
      while (state)
        {
          event = (struct ts_event *) sq_peek(&state->conf.events);
          while (event)
            {
              cause = (struct ts_cause *) sq_peek(&event->conf.causes);
              while (cause)
                {
                  threshold = (struct ts_threshold *) sq_peek(
                      &cause->conf.thresholds);
                  while (threshold)
                    {
                      next_threshold = (struct ts_threshold *) sq_next(
                          &threshold->entry);
                      if (threshold->conf.value.valuetype == VALUESTRING)
                        {
                          free (threshold->conf.value.valuestring);
                        }
                      else if (threshold->conf.value.valuetype == VALUEARRAY)
                        {
                          free (threshold->conf.value.valuearray.items);
                        }
                      free (threshold);
                      threshold = next_threshold;
                    }
                  next_cause = (struct ts_cause *) sq_next(&cause->entry);
                  DEBUGASSERT(cause->dyn.fd < 0);       /* leak! */
                  DEBUGASSERT(cause->dyn.timer_id < 0); /* leak! */
                  free (cause);
                  cause = next_cause;
                }
              next_event = (struct ts_event *) sq_next(&event->entry);
              free (event->conf.actions.cloud.url.host);
              free (event->conf.actions.cloud.url.api);
              for (i = 0; i < event->conf.actions.cloud.url.http_header.valuearray.number_of_items; i++)
                {
                  if (event->conf.actions.cloud.url.http_header.valuearray.items[i].valuetype == VALUESTRING)
                    {
                      free (event->conf.actions.cloud.url.http_header.valuearray.items[i].valuestring);
                    }
                }
              free (event->conf.actions.cloud.url.http_header.valuearray.items);

              free (event->conf.actions.display.showText);
              free (event->conf.actions.sms.phoneNumber);
              free (event->conf.actions.sms.text);
              free (event->conf.name);
              free (event);
              event = next_event;
            }
          next_state = (struct ts_state *) sq_next(&state->entry);
          free (state->conf.name);
          free (state);
          state = next_state;
        }
      next_purpose = (struct ts_purpose *) sq_next(&purpose->entry);
      free (purpose->conf.name);
      free (purpose);
      purpose = next_purpose;
    }

  free (profile->conf.apiVersion);
  free (profile->conf.pId);
  free (profile->conf.name);
  free (profile);
}

struct ts_profile *
profile_parse (const char *profile_str, int *errcode)
{
  struct ts_profile *profile;
  cJSON *json_profile;
  cJSON *json_purposes;
  int ret;

  profile = malloc (sizeof(struct ts_profile));
  if (!profile)
    {
      eng_dbg ("malloc for struct ts_profile failed\n");
      return NULL;
    }

  json_profile = cJSON_Parse (profile_str);
  if (!json_profile)
    {
      eng_dispdbg ("cJSON_Parse failed");
      *errcode = -PROFILE_ERROR_INVALID_JSON;
      goto fail_with_profile;
    }

  static const struct ts_parse parse_profile[] =
        {
          { g_apiVersion_str, offsetof(struct ts_profile, conf.apiVersion),
              VALUESTRING, true, .error = -PROFILE_ERROR_API_VERSION_NOT_SET },
          { g_pId_str, offsetof(struct ts_profile, conf.pId),
              VALUESTRING, false, .value = { VALUESTRING, .valuestring = NULL }, },
          { g_name_str, offsetof(struct ts_profile, conf.name),
              VALUESTRING, false, .value = { VALUESTRING, .valuestring = NULL }, },
          { g_initPuId_str, offsetof(struct ts_profile, conf.initPuId),
              VALUEUINT32, true, .error = -PROFILE_ERROR_INIT_PURPOSE_ID_NOT_SET },
        };

  ret = parse_json_object (json_profile, profile, parse_profile,
                           ARRAY_SIZE(parse_profile));
  if (ret != OK)
    {
      eng_dispdbg ("profile parse failed");
      *errcode = ret;
      goto fail_with_json;
    }

  json_purposes = cJSON_GetObjectItem (json_profile, g_purposes_str);
  if (!json_purposes)
    {
      eng_dispdbg ("purposes not set");
      *errcode = -PROFILE_ERROR_PURPOSES_NOT_SET;
      goto fail_with_json;
    }

  ret = init_purposes (json_purposes, profile);
  if (ret != OK)
    {
      eng_dispdbg ("init_purposes failed");
      *errcode = ret;
      goto fail_with_initialized_profile;
    }

  /* TODO: check next_state_id's, must point to existing state */

  cJSON_Delete (json_profile);

  *errcode = OK;

  return profile;

fail_with_initialized_profile:
  profile_free (profile);
  profile = NULL;
fail_with_json:
  cJSON_Delete (json_profile);
fail_with_profile:
  if (profile) free (profile);

  return NULL;
}
