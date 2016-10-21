/****************************************************************************
 * apps/ts_engine/engine/execute.c
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

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>

#include <debug.h>
#include <assert.h>

#include <apps/thingsee/ts_core.h>
#ifndef CONFIG_ARCH_SIM
#include <apps/thingsee/modules/ts_emmc.h>
#include "../../system/display/oled_display.h" /* TODO */
#ifdef CONFIG_THINGSEE_UI
#include "../../thingsee/ui/thingsee_ui.h"
#endif
#endif

#include "eng_dbg.h"
#include "parse.h"
#include "execute.h"
#include "sense.h"
#include "eng_error.h"
#include "util.h"
#include "log.h"
#include "client.h"
#include "geofence.h"
#include "cloud_property.h"

#ifdef CONFIG_ARCH_SIM
#include "sense_sim.h"
#endif
#include "connector.h"
#include "../connectors/connector.h"
#include "main.h"
#if defined(CONFIG_SYSTEM_CONMAN) && !defined(CONFIG_ARCH_SIM)
#include <apps/system/conman.h>
#endif

#include <arch/board/board-battery.h>
#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>


#define INVALID_PURPOSE (-1)
#define INVALID_STATE   (-1)

#ifdef CONFIG_ARCH_SIM
#define TS_EMMC_MOUNT_PATH "/invalid"
#define oled_dbg_printf(format, ...)
#endif

#define MAX(x,y)                ((x) > (y) ? (x) : (y))

#ifndef offset_of
#define offset_of(type, member) ((intptr_t)(&(((type *)0)->member)))
#endif

#ifndef container_of
#define container_of(ptr, type, member) \
        ((type *)((intptr_t)(ptr) - offset_of(type, member)))
#endif

#define SMS_LIMIT_DEFAULT_MIN_INTERVAL_MINUTES  (0)
#define SMS_LIMIT_DEFAULT_MAX_PER_DAY           (24)

typedef int
(*cause_init_t) (struct ts_cause *cause);

static struct ts_engine g_engine =
{ /* only one instance */
#if defined(CONFIG_SYSTEM_CONMAN) && !defined(CONFIG_ARCH_SIM)
  .conman_connid = CONMAN_CONNID_CLEAR,
#endif
};

static int
init_profile (struct ts_profile *profile, purpose_id_t purpose_id,
              state_id_t state_id);

static int
init_purpose (struct ts_purpose *purpose, state_id_t state_id);

static int
init_state (struct ts_state *state, cause_init_t cause_init);

static int
init_event (struct ts_event *event, cause_init_t cause_init);

static int
init_cause (struct ts_cause *cause);

static int
reinit_cause (struct ts_cause *cause);

static int
uninit_cause (struct ts_cause *cause);

#if defined(CONFIG_THINGSEE_UI) || defined(CONFIG_THINGSEE_ENGINE_DBG)
static const char g_str_no_name[] = "No name";
#endif
#ifdef CONFIG_THINGSEE_ENGINE_DBG
static const char g_str_initial[] = "Initial";
#endif

static void
send_log (struct ts_engine_app *app, struct url * const url);

/* Debug-only functions */
void mem_dbg (void);
void vbat_dbg (void);

static void
free_valuearray(struct ts_value *value)
{
  if (value->valuetype == VALUEARRAY &&
      value->valuearray.items)
    {
      free (value->valuearray.items);
      value->valuearray.items = NULL;
    }
}

static size_t
sense_value_copy(struct ts_sense_value *dst, struct ts_sense_value *src,
                 void *valuemem)
{
  size_t valuelen = 0;

  DEBUGASSERT(src);

  if (dst)
    {
      memcpy(dst, src, sizeof(*dst));
    }

  if (src->value.valuetype == VALUEARRAY)
    {
      valuelen = src->value.valuearray.items ?
          src->value.valuearray.number_of_items * sizeof(struct ts_value) : 0;

      if (valuemem && valuelen)
        {
          memcpy(valuemem, src->value.valuearray.items, valuelen);
          dst->value.valuearray.items = valuemem;
        }
    }
  else if (src->value.valuetype == VALUESTRING)
    {
      size_t buflen = src->value.valuestring ? strlen(src->value.valuestring) + 1 : 0;

      /* Pad length to multiple of sizeof(long). */

      valuelen = (buflen / sizeof(long) + !!(buflen % sizeof(long))) * sizeof(long);

      if (valuemem && valuelen)
        {
          strncpy(valuemem, src->value.valuestring, valuelen);
          dst->value.valuestring = valuemem;
        }
    }

  return valuelen;
}

static size_t
sense_value_extramem(struct ts_sense_value *sense_value)
{
  return sense_value_copy(NULL, sense_value, NULL);
}

static struct ts_payload *
generate_payload (const void *any, bool eventpayload)
{
  struct ts_payload *payload;
  struct ts_cause *cause;
  struct ts_event *event;
  struct ts_state *state;
  struct ts_purpose *purpose;
  struct ts_profile *profile;
  size_t extralen = 0;
  size_t extrapos = 0;
  uint8_t *extramem;
  int i;
  int number_of_causes;

  if (eventpayload)
    {
      event = (struct ts_event *)any;
      number_of_causes = event->conf.number_of_causes;

      cause = (struct ts_cause *) sq_peek(&event->conf.causes);

      for (i = 0; cause; i++)
        {
          DEBUGASSERT(i < number_of_causes);

          extralen += sense_value_extramem(&cause->dyn.sense_value);

          cause = (struct ts_cause *) sq_next(&cause->entry);
        }
    }
  else
    {
      cause = (struct ts_cause *)any;
      event = cause->parent;
      number_of_causes = 1;

      extralen += sense_value_extramem(&cause->dyn.sense_value);
    }

  state = event->parent;
  purpose = state->parent;
  profile = purpose->parent;

  payload = malloc (number_of_causes * sizeof(struct ts_sense_value)
                    + sizeof(*payload) + extralen);
  if (!payload)
    {
      eng_dbg ("malloc for payload failed\n");
      return NULL;
    }

  extramem = (void *)((uintptr_t)payload + sizeof(*payload) +
                      number_of_causes * sizeof(struct ts_sense_value));

  payload->number_of_senses = number_of_causes;

  payload->state.pId  = profile->conf.pId;
  payload->state.puId = purpose->conf.puId;
  payload->state.stId = state->conf.stId;

  if (eventpayload)
    {
      payload->state.evId = event->conf.evId;
    }
  else
    {
      payload->state.evId = -1;
    }

  clock_gettime(CLOCK_REALTIME, &payload->state.ts);

  if (!eventpayload)
    {
      extrapos += sense_value_copy(&payload->senses[0], &cause->dyn.sense_value, &extramem[extrapos]);
      DEBUGASSERT(extrapos == extralen);
      return payload;
    }

  cause = (struct ts_cause *) sq_peek(&event->conf.causes);

  for (i = 0; cause; i++)
    {
      DEBUGASSERT(i < number_of_causes);

      extrapos += sense_value_copy(&payload->senses[i], &cause->dyn.sense_value, &extramem[extrapos]);

      cause = (struct ts_cause *) sq_next(&cause->entry);
    }

  DEBUGASSERT(extrapos == extralen);
  return payload;
}

static struct ts_payload *
generate_event_payload (const struct ts_event * const event)
{
  return generate_payload(event, true);
}

static struct ts_payload *
generate_cause_payload (struct ts_cause *cause)
{
  return generate_payload(cause, false);
}

static void
log_event (struct ts_event *event)
{
#ifndef CONFIG_ARCH_SIM

  struct ts_payload *payload;

  payload = generate_event_payload(event);
  if (!payload)
    {
      eng_dbg("generate_event_payload failed\n");
      return;
    }

  __ts_engine_log_payload (payload, LOG_EVENTS);
  free (payload);

  mem_dbg();
  vbat_dbg();

#endif
}

static void
log_cause (struct ts_cause *cause)
{
#ifndef CONFIG_ARCH_SIM

  struct ts_payload *payload;

  payload = generate_cause_payload(cause);
  if (!payload)
    {
      eng_dbg("generate_cause_payload failed\n");
      return;
    }

  __ts_engine_log_payload (payload, LOG_CAUSES);
  free (payload);

  mem_dbg();
  vbat_dbg();

#endif
}

static struct ts_state *
find_state_by_id (struct ts_purpose *purpose, state_id_t state_id)
{
  struct ts_state *state;

  if (state_id == INVALID_STATE)
    {
      state_id = purpose->conf.initStId;
    }

  state = (struct ts_state *) sq_peek(&purpose->conf.states);

  while (state)
    {
      if (state->conf.stId == state_id)
        {
          break;
        }

      state = (struct ts_state *) sq_next(&state->entry);
    }

  return state;
}

static struct ts_purpose *
find_purpose_by_id (struct ts_profile *profile, purpose_id_t purpose_id)
{
  struct ts_purpose *purpose;

  if (purpose_id == INVALID_PURPOSE)
    {
      purpose_id = profile->conf.initPuId;
    }

  purpose = (struct ts_purpose *) sq_peek(&profile->conf.purposes);

  while (purpose)
    {
      if (purpose->conf.puId == purpose_id)
        {
          break;
        }

      purpose = (struct ts_purpose *) sq_next(&purpose->entry);
    }

  return purpose;
}

static void
stop_cause_timer (struct ts_cause *cause)
{
  eng_dbg ("Stopping timer_id %d\n", cause->dyn.timer_id);

  ts_core_timer_stop(cause->dyn.timer_id);

  cause->dyn.timer_id = -1;
}

static bool
is_active_relative_cause (struct ts_cause *cause)
{
  return (cause->conf.measurement.interval != -1) &&
      cause->conf.threshold.relative;
}

bool __ts_engine_check_geofence(struct ts_threshold *thr, struct ts_value *value)
{
  struct point test;
  struct poly *poly;
  int i;
  int points;
  int ret;

  DEBUGASSERT(thr->conf.value.valuetype == VALUEARRAY &&
              !(thr->conf.value.valuearray.number_of_items & 0x1));

  DEBUGASSERT(value->valuetype == VALUEARRAY &&
              value->valuearray.number_of_items == 7);

  points = thr->conf.value.valuearray.number_of_items / 2;

  if (points < 3)
    {
      eng_dbg("Not enough points in polygon\n");
      return false;
    }

  poly = malloc(sizeof(*poly) + points * (sizeof(poly->points[0])));
  if (!poly)
    {
      eng_dbg("malloc %d failed\n", sizeof(*poly) +
              points * (sizeof(poly->points[0])));
      return false;
    }

  for (i = 0; i < points; i++)
    {
      poly->points[i].x = thr->conf.value.valuearray.items[2 * i].valuedouble;
      poly->points[i].y = thr->conf.value.valuearray.items[2 * i + 1].valuedouble;
    }

  poly->number_of_points = points;

  test.x = value->valuearray.items[0].valuedouble;
  test.y = value->valuearray.items[1].valuedouble;

  ret = point_in_polygon(&test, poly);

  free(poly);
  free_valuearray(value);

  value->valuetype = VALUEBOOL;
  value->valuebool = ret;

  return ret;
}

static int
check_valuedouble_threshold (struct ts_threshold *threshold, struct ts_value *val)
{
  struct ts_cause *cause = threshold->parent;
  double value;
  double limit;
  int i;

  value = val->valuedouble;
  limit = threshold->conf.value.valuedouble;

  if (is_active_relative_cause (cause))
    {
      value -= cause->dyn.sense_bias.value.valuedouble;
    }

  switch (threshold->conf.type)
    {
    case isOneOf:
      /* TODO: value rounding ? */
      for (i = 0; i < threshold->conf.value.valuearray.number_of_items; i++)
        {
          if (value == (double)threshold->conf.value.valuearray.items[i].valuedouble)
            {
              return true;
            }
        }
      return false;
    case isGt:
      return value > limit;
    case isLt:
      return value < limit;
    case isNotIn:
      /* TODO: value rounding ? */
      for (i = 0; i < threshold->conf.value.valuearray.number_of_items; i++)
        {
          if (value == (double)threshold->conf.value.valuearray.items[i].valuedouble)
            {
              return false;
            }
        }
      return true;
    case isAny:
      return true;
    default:
      return false;
    }
}

static int
check_valueuint16_threshold (struct ts_threshold *threshold, struct ts_value *val)
{
  struct ts_cause *cause = threshold->parent;
  uint16_t value;
  uint16_t limit;
  int i;

  value = val->valueint16;
  limit = (uint16_t)threshold->conf.value.valuedouble;

  if (is_active_relative_cause (cause))
    {
      value -= cause->dyn.sense_bias.value.valueint16;
    }

  switch (threshold->conf.type)
    {
    case isOneOf:
      for (i = 0; i < threshold->conf.value.valuearray.number_of_items; i++)
        {
          if (value == (uint16_t)threshold->conf.value.valuearray.items[i].valuedouble)
            {
              return true;
            }
        }
      return false;
    case isGt:
      return value > limit;
    case isLt:
      return value < limit;
    case isNotIn:
      for (i = 0; i < threshold->conf.value.valuearray.number_of_items; i++)
        {
          if (value == (uint16_t)threshold->conf.value.valuearray.items[i].valuedouble)
            {
              return false;
            }
        }
      return true;
    case isAny:
      return true;
    default:
      return false;
    }
}

static int
check_valueuint32_threshold (struct ts_threshold *threshold, struct ts_value *val)
{
  struct ts_cause *cause = threshold->parent;
  uint32_t value;
  uint32_t limit;
  int i;

  value = val->valueint32;
  limit = (uint32_t)threshold->conf.value.valuedouble;

  if (is_active_relative_cause (cause))
    {
      value -= cause->dyn.sense_bias.value.valueint32;
    }

  switch (threshold->conf.type)
    {
    case isOneOf:
      for (i = 0; i < threshold->conf.value.valuearray.number_of_items; i++)
        {
          if (value == (uint32_t)threshold->conf.value.valuearray.items[i].valuedouble)
            {
              return true;
            }
        }
      return false;
    case isGt:
      return value > limit;
    case isLt:
      return value < limit;
    case isNotIn:
      for (i = 0; i < threshold->conf.value.valuearray.number_of_items; i++)
        {
          if (value == (uint32_t)threshold->conf.value.valuearray.items[i].valuedouble)
            {
              return false;
            }
        }
      return true;
    case isAny:
      return true;
    default:
      return false;
    }
}

static int
check_valueint16_threshold (struct ts_threshold *threshold, struct ts_value *val)
{
  struct ts_cause *cause = threshold->parent;
  int16_t value;
  int16_t limit;
  int i;

  value = val->valueint16;
  limit = (int16_t)threshold->conf.value.valuedouble;

  if (is_active_relative_cause (cause))
    {
      value -= cause->dyn.sense_bias.value.valueint16;
    }

  switch (threshold->conf.type)
    {
    case isOneOf:
      for (i = 0; i < threshold->conf.value.valuearray.number_of_items; i++)
        {
          if (value == (int16_t)threshold->conf.value.valuearray.items[i].valuedouble)
            {
              return true;
            }
        }
      return false;
    case isGt:
      return value > limit;
    case isLt:
      return value < limit;
    case isNotIn:
      for (i = 0; i < threshold->conf.value.valuearray.number_of_items; i++)
        {
          if (value == (int16_t)threshold->conf.value.valuearray.items[i].valuedouble)
            {
              return false;
            }
        }
      return true;
    case isAny:
      return true;
    default:
      return false;
    }
}

static int
check_valueint32_threshold (struct ts_threshold *threshold, struct ts_value *val)
{
  struct ts_cause *cause = threshold->parent;
  int32_t value;
  int32_t limit;
  int i;

  value = val->valueint32;
  limit = (int32_t)threshold->conf.value.valuedouble;

  if (is_active_relative_cause (cause))
    {
      value -= cause->dyn.sense_bias.value.valueint32;
    }

  switch (threshold->conf.type)
    {
    case isOneOf:
      for (i = 0; i < threshold->conf.value.valuearray.number_of_items; i++)
        {
          if (value == (int32_t)threshold->conf.value.valuearray.items[i].valuedouble)
            {
              return true;
            }
        }
      return false;
    case isGt:
      return value > limit;
    case isLt:
      return value < limit;
    case isNotIn:
      for (i = 0; i < threshold->conf.value.valuearray.number_of_items; i++)
        {
          if (value == (int32_t)threshold->conf.value.valuearray.items[i].valuedouble)
            {
              return false;
            }
        }
      return true;
    case isAny:
      return true;
    default:
      return false;
    }
}

static int
check_valuestring_threshold (struct ts_threshold *threshold, struct ts_value *val)
{
  char *value;
  char *limit;
  int i;

  value = val->valuestring;
  limit = threshold->conf.value.valuestring;

  switch (threshold->conf.type)
    {
    case isOneOf:
      for (i = 0; i < threshold->conf.value.valuearray.number_of_items; i++)
        {
          if (strcmp(value, threshold->conf.value.valuearray.items[i].valuestring) == 0)
            {
              return true;
            }
        }
      return false;
    case isGt:
      return !!strcmp (value, limit);
    case isLt:
      return !strcmp (value, limit);
    case isNotIn:
      for (i = 0; i < threshold->conf.value.valuearray.number_of_items; i++)
        {
          if (strcmp(value, threshold->conf.value.valuearray.items[i].valuestring) == 0)
            {
              return false;
            }
        }
      return true;
    case isAny:
      return true;
    default:
      return false;
    }
}

static int
check_valuebool_threshold (struct ts_threshold *threshold, struct ts_value *val)
{
  bool value;
  bool limit;
  int i;

  value = val->valuebool;
  limit = (bool)threshold->conf.value.valuedouble;

  switch (threshold->conf.type)
    {
    case isOneOf:
      for (i = 0; i < threshold->conf.value.valuearray.number_of_items; i++)
        {
          if (value == (bool)threshold->conf.value.valuearray.items[i].valuedouble)
            {
              return true;
            }
        }
      return false;
    case isGt:
      return value > limit;
    case isLt:
      return value < limit;
    case isNotIn:
      for (i = 0; i < threshold->conf.value.valuearray.number_of_items; i++)
        {
          if (value == (bool)threshold->conf.value.valuearray.items[i].valuedouble)
            {
              return false;
            }
        }
      return true;
    case isAny:
      return true;
    default:
      return false;
    }
}

static int
check_threshold (struct ts_threshold *threshold, struct ts_value *value)
{
  int ret;
  int i;

  if (threshold->conf.check_threshold)
    {
      return threshold->conf.check_threshold(threshold, value);
    }

  switch (value->valuetype)
    {
    case VALUEDOUBLE:
      return check_valuedouble_threshold (threshold, value);
      break;
    case VALUEUINT16:
      return check_valueuint16_threshold (threshold, value);
      break;
    case VALUEUINT32:
      return check_valueuint32_threshold (threshold, value);
      break;
    case VALUEINT16:
      return check_valueint16_threshold (threshold, value);
      break;
    case VALUEINT32:
      return check_valueint32_threshold (threshold, value);
      break;
    case VALUESTRING:
      return check_valuestring_threshold (threshold, value);
      break;
    case VALUEBOOL:
      return check_valuebool_threshold (threshold, value);
      break;
    case VALUEARRAY:
      DEBUGASSERT(threshold->conf.type == isOneOf || threshold->conf.type == isNotIn);

      if (threshold->conf.type == isOneOf)
        {
          ret = 0;
          for (i = 0; i < threshold->conf.value.valuearray.number_of_items; i++)
            {
              DEBUGASSERT(threshold->conf.value.valuetype != VALUEARRAY);
              ret = ret || check_threshold(threshold, &threshold->conf.value.valuearray.items[i]);
            }
        }
      else
        {
          ret = 1;
          for (i = 0; i < threshold->conf.value.valuearray.number_of_items; i++)
            {
              DEBUGASSERT(threshold->conf.value.valuetype != VALUEARRAY);
              ret = ret && check_threshold(threshold, &threshold->conf.value.valuearray.items[i]);
            }
        }
      return ret;
      break;
    default:
      ASSERT (false);
    }
}

static int
check_thresholds (struct ts_cause *cause)
{
  struct ts_threshold *threshold;
  struct ts_value *value = &cause->dyn.sense_value.value;
  int ret;

  if (cause->dyn.measure_bias)
    {
      memcpy(&cause->dyn.sense_bias, &cause->dyn.sense_value, sizeof(cause->dyn.sense_bias));
      cause->dyn.measure_bias = false;
      return false;
    }

  if (cause->dyn.measurement_counter > 0)
    {
      cause->dyn.measurement_counter--;

      eng_dbg ("measurement_counter: %d / %d\n",
               cause->conf.measurement.count - cause->dyn.measurement_counter,
               cause->conf.measurement.count);

      if (cause->dyn.measurement_counter == 0)
        {
          uninit_cause (cause);
          return 1;
        }
      else
        {
           return 0;
        }
    }

  threshold = (struct ts_threshold *) sq_peek(&cause->conf.thresholds);

  if (!threshold)
    {
      return 0;
    }

  ret = 1;

  while (threshold)
    {
      ret = check_threshold (threshold, value) && ret;

      threshold = (struct ts_threshold *) sq_next(&threshold->entry);
    }

  if (cause->conf.threshold.negate)
    {
      ret = !ret;
    }

  if (!ret)
    {
      if (cause->conf.orderId)
        {
          init_event (cause->parent, reinit_cause);
        }
      else
        {
          cause->dyn.threshold_counter = cause->conf.threshold.count;
        }

      eng_dbg("value not in limits\n");

      return 0;
    }

  eng_dbg ("value in limits\n");

  if (cause->dyn.threshold_counter > 0)
    {
      cause->dyn.threshold_counter--;

      eng_dbg ("threshold_counter: %d / %d\n",
               cause->conf.threshold.count - cause->dyn.threshold_counter,
               cause->conf.threshold.count);

      if (cause->dyn.threshold_counter > 0)
        {
          return 0;
        }
    }

  if (cause->conf.orderId)
    {
      if (cause->conf.orderId != cause->parent->dyn.expected_order_id)
        {
          init_event (cause->parent, reinit_cause);

          return 0;
        }

      cause->parent->dyn.expected_order_id++;
    }

  return 1;
}

static int
all_causes_triggered (sq_queue_t causes)
{
  struct ts_cause *cause;

  cause = (struct ts_cause *) sq_peek(&causes);

  while (cause)
    {
      if (cause->dyn.triggered == false)
        {
          return 0;
        }

      cause = (struct ts_cause *) sq_next(&cause->entry);
    }

  return 1;
}

#ifdef CONFIG_THINGSEE_CONNECTORS
static int
send_cb (int result, const void *priv)
{
  struct ts_engine_client client;
  struct ts_engine_connector_cb_info info;
  int ret;

  eng_dbg ("result: %d\n", result);

  if (g_engine_pid == getpid())
    {
      /* TODO: What to do? */
      eng_dbg("send_cb from engine context!\n");
      return ERROR;
    }

  ret = ts_engine_client_init(&client);
  if (ret < 0)
    {
      eng_dbg("ts_engine_client_init failed\n");
      return ERROR;
    }

  info.result = result;
  info.priv = priv;

  ret = ts_engine_client_connector_result_shm(&client, &info, sizeof(info));
  if (ret < 0)
    {
      eng_dbg("ts_engine_client_connector_result_shm failed\n");
      ret = ERROR;
    }

  ts_engine_client_uninit(&client);
  return ret;
}
#endif

int
__ts_engine_send(struct ts_payload *payload, struct url * const url,
                 void const *priv)
{
#ifdef CONFIG_THINGSEE_CONNECTORS
  const struct ts_connector *con;
  int ret;

  ret = ts_engine_select_connector(0, &con);
  if (ret < 0)
    {
      eng_dbg("ts_engine_select_connector failed\n");
      return ERROR;
    }

  if (con->send)
    {
      if (url && con->send_url)
        {
          ret = con->send_url (payload, send_cb, url, priv);
        }
      else
        {
          ret = con->send (payload, send_cb, priv);
        }

      if (ret != OK)
        {
          eng_dbg ("send to connector_d %d failed\n", 0);
          return ERROR;
        }
    }
  else
    {
      eng_dbg("connector does not support send\n");
      return ERROR;
    }
#endif

  return OK;
}

int
__ts_engine_multisend(struct ts_payload **payloads, int number_of_payloads,
                      struct url * const url, void const *priv)
{
#ifdef CONFIG_THINGSEE_CONNECTORS
  const struct ts_connector *con;
  int ret;

  ret = ts_engine_select_connector(0, &con);
  if (ret != OK)
    {
      eng_dbg("ts_engine_select_connector failed\n");
      return ERROR;
    }

  if (con->multisend)
    {
      if (url && con->multisend_url)
        {
          ret = con->multisend_url (payloads, number_of_payloads, send_cb, url, priv);
        }
      else
        {
          ret = con->multisend (payloads, number_of_payloads, send_cb, priv);
        }

      if (ret != OK)
        {
          eng_dbg ("multisend to connector_d %d failed\n", 0);
          return ERROR;
        }
    }
  else
    {
      eng_dbg("connector does not support multisend\n");
      return ERROR;
    }
#endif

  return OK;
}

static int
send_event (struct ts_event *event)
{
  struct ts_payload *payload;
  int ret;
  struct url *url;

  eng_dbg ("send payload to connector %d\n", 0);

  payload = generate_event_payload(event);
  if (!payload)
    {
      eng_dbg ("generate_event_payload failed\n");
      return ERROR;
    }

  if (event->conf.actions.cloud.url.host)
    {
      url = &event->conf.actions.cloud.url;
    }
  else
    {
      url = NULL;
    }

  ret = __ts_engine_send(payload, url, event);
  if (ret < 0)
    {
      eng_dbg("__ts_engine_send failed, logging payload\n");
      __ts_engine_log_payload (payload, LOG_EVENTS);
      ret = ERROR;
    }

  free(payload);

  mem_dbg();
  vbat_dbg();

  return ret;
}

static int
send_ping_event (struct ts_engine *engine)
{
  struct ts_payload payload;
  static char ping_magic[] = "ping";

  payload.number_of_senses = 0;
  payload.state.pId = (engine ? engine->profile->conf.pId : NULL);
  payload.state.puId = -1;
  payload.state.stId = -1;
  payload.state.evId = -1;
  clock_gettime(CLOCK_REALTIME, &payload.state.ts);

  eng_color_dbg(BLACK, WHITE, "Sending ping event\n");

  return __ts_engine_send(&payload, NULL, ping_magic);
}

void __ts_engine_ping_result(struct ts_engine_app *app, const void *priv, int result)
{
  eng_color_dbg(BLACK, WHITE, "ping result: %d\n", result);

  if (app->backend_update.cb == NULL)
    {
      return;
    }

  if (result == OK)
    {
      send_log (app, NULL);

      app->backend_update.cb ((app->backend_update.profile_updated ?
          BACKEND_UPDATE_PROFILE_UPDATED : BACKEND_UPDATE_OK));
    }
  else
    {
      app->backend_update.cb (BACKEND_UPDATE_PING_FAILED);
    }
}

static int
send_cause (struct ts_cause *cause)
{
  struct ts_payload *payload = NULL;
  int ret;

#ifdef CONFIG_THINGSEE_CONNECTORS
  const struct ts_connector *con;

  ret = ts_engine_select_connector(0, &con);
  if (ret == OK && con->send)
    {
      payload = generate_cause_payload (cause);
      if (!payload)
        {
          eng_dbg ("generate_cause_payload failed\n");
          return ERROR;
        }

     if (cause->parent->conf.actions.cloud.url.host &&
         con->send_url)
        {
          ret = con->send_url (payload, send_cb, &cause->parent->conf.actions.cloud.url, cause);
        }
      else
        {
          ret = con->send (payload, send_cb, cause);
        }

      if (ret != OK)
        {
          eng_dbg ("send failed, logging payload\n");
          __ts_engine_log_payload (payload, LOG_CAUSES);
          ret = ERROR;
        }
    }
#endif

  free (payload);
  return ret;
}

static void
show_text (struct ts_event *event)
{
  eng_dbg ("show text \"%s\" on display\n", event->conf.actions.display.showText);

  /* TODO */

  mem_dbg();
  vbat_dbg();
}


static bool
are_dates_on_same_day(struct tm *t1, struct tm *t2)
{
  return (t1->tm_mday == t2->tm_mday) &&
      (t1->tm_mon == t2->tm_mon) &&
      (t1->tm_year == t2->tm_year);
}


static bool
can_send_sms(void)
{
  bool can_send = false;
  uint32_t sent_per_day_count;
  time_t last_sent_time;
  time_t current_time;
  struct timespec current_timespec;

  int min_interval_secs = g_engine.sms_limit.min_interval_minutes * 60;
  int max_per_day = g_engine.sms_limit.max_per_day;

  /* Load persistent variables. */

  sent_per_day_count = board_rtc_read_value(0);
  last_sent_time = board_rtc_read_value(1);
  eng_dbg("sms: loaded persistent sent_per_day_count=%d last_sent_time=%d\n",
    sent_per_day_count, last_sent_time);

  clock_gettime (CLOCK_REALTIME, &current_timespec);
  current_time = current_timespec.tv_sec;

  if ((current_time - last_sent_time) >= min_interval_secs)
    {
      struct tm current_time_tm, last_sent_time_tm;

      gmtime_r(&current_time, &current_time_tm);
      gmtime_r(&last_sent_time, &last_sent_time_tm);
      eng_dbg ("sms: last sent date %02d/%02d/%02d at %02d:%02d:%02d\n", last_sent_time_tm.tm_mday, last_sent_time_tm.tm_mon, last_sent_time_tm.tm_year, last_sent_time_tm.tm_hour, last_sent_time_tm.tm_min, last_sent_time_tm.tm_sec);
      eng_dbg ("sms: current date %02d/%02d/%02d at %02d:%02d:%02d\n", current_time_tm.tm_mday, current_time_tm.tm_mon, current_time_tm.tm_year, current_time_tm.tm_hour, current_time_tm.tm_min, current_time_tm.tm_sec);

      if (!are_dates_on_same_day(&last_sent_time_tm, &current_time_tm))
        {
          sent_per_day_count = 0;
          eng_dbg ("sms: day has changed\n");
        }

      if (sent_per_day_count < max_per_day)
        {
          can_send = true;
        }
      else
        {
          eng_dbg ("sms: count per day %d >= max %d\n", sent_per_day_count, max_per_day);
        }
    }
  else
    {
      eng_dbg ("sms: minimum send interval (%d - %d) = %d < %d\n", current_time, last_sent_time, (current_time - last_sent_time), min_interval_secs);
    }

  if (can_send)
    {
      sent_per_day_count += 1;
      last_sent_time = current_time;

      /* Save persistent variables. */

      board_rtc_save_value(sent_per_day_count, 0);
      eng_dbg ("sms: saving sent_per_day_count %d\n", sent_per_day_count);
      board_rtc_save_value(last_sent_time, 1);
      eng_dbg ("sms: saving last_sent_time %d\n", last_sent_time);
    }

  return can_send;
}

static void
send_sms (struct ts_event *event)
{
  int ret;

  eng_dbg ("send SMS \"%s\" to \"%s\"\n", event->conf.actions.sms.text,
           event->conf.actions.sms.phoneNumber);

  if (!can_send_sms())
    {
      eng_dbg("not allowed to send SMS\n");
      return;
    }

#ifdef CONFIG_SYSTEM_CONMAN
  struct conman_client_s conman_client;

  ret = conman_client_init(&conman_client);
  if (ret != OK)
    {
      eng_dbg("conman_client_init failed\n");
    }
  else
    {
      ret = conman_client_send_sms(&conman_client,
                                   event->conf.actions.sms.phoneNumber,
                                   event->conf.actions.sms.text);
      if (ret != OK)
        {
          eng_dbg("conman_client_send_sms failed\n");
        }

      conman_client_uninit(&conman_client);
    }
#endif

  mem_dbg();
  vbat_dbg();
}

static void
send_log (struct ts_engine_app *app, struct url * const url)
{
  bool multisend;
  const struct ts_connector *con;
  int ret;

  eng_dbg ("send log to connector %d\n", 0);

  ret = ts_engine_select_connector(0, &con);
  if (ret < 0)
    {
      eng_dbg("ts_engine_select_connector failed\n");
    }

  multisend = (con->multisend != NULL);

  __ts_engine_log_start (&app->handle, multisend, url);

  mem_dbg();
  vbat_dbg();
}

static int
do_event_actions (struct ts_event *event)
{
  struct ts_engine *engine = event->parent->parent->parent->parent;
  struct url *url;

  if (event->conf.eventLog && !event->conf.actions.cloud.sendEvent &&
      !event->conf.actions.cloud.sendPush)
    {
      log_event (event);
    }

  if (event->conf.actions.cloud.sendEvent || event->conf.actions.cloud.sendPush)
    {
      send_event (event);
    }

  if (event->conf.actions.cloud.sendLog)
    {
      if (event->conf.actions.cloud.url.host)
        {
          url = &event->conf.actions.cloud.url;
        }
      else
        {
          url = NULL;
        }
      send_log (engine->app, url);
    }

  if (event->conf.actions.display.showText)
    {
      show_text (event);
    }

  if (event->conf.actions.sms.phoneNumber && event->conf.actions.sms.text)
    {
      send_sms (event);
    }

  return OK;
}

bool
handle_cause (struct ts_cause *cause)
{
  void *cause_iterator;
  bool state_changing = false;

  cause_iterator = sq_peek(&cause->parent->parent->dyn.active_causes);

  while (cause_iterator)
    {
      struct ts_cause *cause_container = container_of(cause_iterator,
						      struct ts_cause,
						      active_entry);

      if (cause_container->dyn.sense_value.sId == cause->dyn.sense_value.sId)
        {
          memcpy (&cause_container->dyn.sense_value, &cause->dyn.sense_value,
                  sizeof(cause->dyn.sense_value));
          if (handle_cause_event (cause_container) == true)
            {
              state_changing = true;
            }
        }

      cause_iterator = sq_next(&cause_container->active_entry);
    }

  return state_changing;
}

bool
handle_cause_event (struct ts_cause *cause)
{
  int ret;
  struct ts_event *event = cause->parent;
  struct ts_state *state = event->parent;
  struct ts_purpose *purpose = state->parent;
  struct ts_profile *profile = purpose->parent;
  struct ts_engine *engine = profile->parent;
  char valuestr[VALUE_STR_MAX_LEN];
  bool purpose_changing;
  bool state_changing = false;

  __value_serialize (valuestr, VALUE_STR_MAX_LEN, &cause->dyn.sense_value.value);
  eng_dispdbg("0x%08x %s", cause->dyn.sense_value.sId, valuestr);

  /* Timestamp the sense */

  clock_gettime (CLOCK_REALTIME, &cause->dyn.sense_value.ts);

  if (cause->conf.measurement.send)
    {
      send_cause (cause);
    }

  /* Check thresholds */

  ret = check_thresholds (cause);

  /* Did we trigger */

  if (ret)
    {
      /* Mark the sense as triggered */

      cause->dyn.triggered = true;

      if (cause->conf.senseLog || cause->conf.measurement.log)
        {
          log_cause (cause);
        }
    }
  else
    {
      cause->dyn.triggered = false;

      if (cause->conf.measurement.log)
        {
          log_cause (cause);
        }

      goto out;
    }

  /* Are all the senses for the event triggered */

  if (all_causes_triggered (event->conf.causes))
    {
      eng_dbg ("EV \"%s\" %d\n", (event->conf.name ? event->conf.name : g_str_no_name),
                                 event->conf.evId);

      /* Do actions */

      do_event_actions (event);

      purpose_changing = (event->conf.actions.engine.gotoPuId != -1)
          && (engine->current_purpose->conf.puId != event->conf.actions.engine.gotoPuId);

      state_changing = purpose_changing || ((event->conf.actions.engine.gotoStId != -1)
          && (event->conf.actions.engine.gotoStId != engine->current_state->conf.stId));

      /* Re-init event's causes if we are not going to change to a new state */

      if (!state_changing)
        {
          init_event (event, reinit_cause);
          goto out;
        }

      /* Change purpose & state */

      if (purpose_changing)
        {
          init_profile (engine->profile, event->conf.actions.engine.gotoPuId,
                        event->conf.actions.engine.gotoStId);
        }
      else
        {
          init_profile (engine->profile, engine->current_purpose->conf.puId,
                        event->conf.actions.engine.gotoStId);
        }
    }

out:

  free_valuearray(&cause->dyn.sense_value.value);

  return state_changing;
}

static int
init_cause (struct ts_cause *cause)
{
  eng_dbg ("sId: 0x%08x\n", cause->dyn.sense_value.sId);

  cause->dyn.timer_id = -1;
  cause->dyn.fd = -1;
  cause->dyn.triggered = false;
  cause->dyn.threshold_counter = cause->conf.threshold.count;

  if (cause->conf.measurement.count == 0)
    {
      cause->dyn.measurement_counter = -1;
    }
  else
    {
      cause->dyn.measurement_counter = cause->conf.measurement.count;
    }

  if (is_active_relative_cause (cause))
    {
      cause->dyn.measure_bias = true;
    }
  else
    {
      cause->dyn.measure_bias = false;
    }

  sq_addlast(&cause->active_entry, &cause->parent->parent->dyn.active_causes);

  return engine_cause_request_value (cause);
}

static int
reinit_cause (struct ts_cause *cause)
{
  eng_dbg ("sId: 0x%08x\n", cause->dyn.sense_value.sId);

  if (cause->dyn.measurement_counter == 0)
    {
      return init_cause(cause);
    }

  cause->dyn.triggered = false;
  cause->dyn.threshold_counter = cause->conf.threshold.count;

  return OK;
}

static int
uninit_cause (struct ts_cause *cause)
{
  int ret;

  eng_dbg ("sId: 0x%08x\n", cause->dyn.sense_value.sId);

  sq_rem (&cause->active_entry, &cause->parent->parent->dyn.active_causes);

  if (cause->dyn.timer_id != -1)
    {
      stop_cause_timer (cause);
    }

  if (cause->dyn.fd != -1)
    {
      eng_dbg ("closing fd: %d\n", cause->dyn.fd);

      ret = ts_core_fd_unregister (cause->dyn.fd);
      if (ret != OK)
        {
          eng_dbg ("ts_core_fd_unregister failed\n");
        }

      close (cause->dyn.fd);
    }

  if (cause->conf.measurement.interval > 0 && cause->dyn.sense_info->ops.active.read)
    {
      if (cause->dyn.sense_info->ops.active.uninit)
        {
          cause->dyn.sense_info->ops.active.uninit (cause);
        }
    }
  else
    {
      if (cause->dyn.sense_info->ops.irq.uninit)
        {
          cause->dyn.sense_info->ops.irq.uninit (cause);
        }
    }

  free_valuearray(&cause->dyn.sense_value.value);
  return OK;
}

static int
init_event (struct ts_event *event, cause_init_t cause_init)
{
  struct ts_cause *cause;
  int ret;

  cause = (struct ts_cause *) sq_peek(&event->conf.causes);

  while (cause)
    {
      ret = cause_init (cause);
      if (ret < 0)
        {
          eng_dbg ("cause_init failed\n");
          return ERROR;
        }

      cause = (struct ts_cause *) sq_next(&cause->entry);
    }

  event->dyn.expected_order_id = FIRST_ORDER_ID;

  return OK;
}

static int
init_state (struct ts_state *state, cause_init_t cause_init)
{
  struct ts_event *event;
  int ret;

  sq_init (&state->dyn.active_causes);

  event = (struct ts_event *) sq_peek(&state->conf.events);

  while (event)
    {
      ret = init_event (event, cause_init);
      if (ret < 0)
        {
          eng_dbg ("init_event failed\n");
          return ret;
        }

      event = (struct ts_event *) sq_next(&event->entry);
    }

  return OK;
}

void
mem_dbg (void)
{
#ifdef CONFIG_THINGSEE_ENGINE_DBG
  struct mallinfo mem;

  mem = mallinfo ();

  eng_dbg ("             total       used       free    largest\n");
  eng_dbg ("Mem:   %11d%11d%11d%11d\n", mem.arena, mem.uordblks, mem.fordblks,
       mem.mxordblk);

#endif
}

void
vbat_dbg(void)
{
#ifdef CONFIG_THINGSEE_ENGINE_DBG
  int ret;
  uint32_t val;

  ret = board_get_battery_voltage(&val);
  if (ret < 0)
    {
      eng_dbg ("Cannot read battery voltage: %d\n", ret);
      return;
    }

  eng_dbg ("Battery: %u mV\n", val);
#endif
}

static int
add_global_state (struct ts_state *state)
{
  struct ts_global_state *global_state;
  struct ts_engine *engine;

  global_state = (struct ts_global_state *) calloc (
      1, sizeof(struct ts_global_state));
  if (!global_state)
    {
      eng_dbg ("malloc failed for struct ts_global_state\n");
      return -TS_ENGINE_ERROR_OOM;
    }

  global_state->state = state;

  engine = state->parent->parent->parent;

  sq_addlast (&global_state->entry, &engine->global_states);

  return init_state (state, init_cause);
}

static int
init_global_states (struct ts_engine *engine, struct ts_purpose *purpose)
{
  struct ts_state *state;
  int ret;

  state = (struct ts_state *) sq_peek(&purpose->conf.states);

  while (state)
    {
      if (state->conf.isGlobal)
        {
          ret = add_global_state (state);
          if (ret < 0)
            {
              eng_dbg ("add_global_state failed\n");
              return ret;
            }
        }

      state = (struct ts_state *) sq_next(&state->entry);
    }

  return OK;
}

static void
uninit_global_states (struct ts_engine *engine)
{
  struct ts_global_state *current, *next;

  current = (struct ts_global_state *) sq_peek(&engine->global_states);

  while (current)
    {
      next = (struct ts_global_state *) sq_next(&current->entry);

      if (current->state->conf.isGlobal)
        {
          (void)init_state (current->state, uninit_cause);
          free (current);
        }

      current = next;
    }

  sq_init(&engine->global_states);
}

static int
init_purpose (struct ts_purpose *purpose, state_id_t state_id)
{
  struct ts_engine *engine = purpose->parent->parent;
  struct ts_state *state;
  int ret;

  mem_dbg ();
  vbat_dbg ();

  state = find_state_by_id (purpose, state_id);
  if (!state)
    {
      eng_dbg ("state_id: %d does not exist\n", state_id);
      return ERROR;
    }

#ifdef CONFIG_THINGSEE_UI
  thingsee_UI_set_purpose_and_state (purpose->conf.name ? purpose->conf.name : g_str_no_name,
	  state->conf.name ? state->conf.name : g_str_no_name);
#endif

  if (engine->current_state != state)
    {
      eng_dbg ("ST %s %d:%s %d\n",
	   (engine->current_state ? (engine->current_state->conf.name ? engine->current_state->conf.name : g_str_no_name) : g_str_initial),
	   (engine->current_state ? engine->current_state->conf.stId : INVALID_STATE),
	   (state->conf.name ? state->conf.name : g_str_no_name),
	   state->conf.stId);

      if (engine->current_state)
        {
          ret = init_state (engine->current_state, uninit_cause);
          if (ret < 0)
            {
              eng_dbg ("init_state failed\n");
              return ERROR;
            }
        }

      ret = init_state (state, init_cause);
      if (ret < 0)
        {
          eng_dbg ("init_state failed\n");
          return ERROR;
        }

      engine->current_state = state;
    }

  return OK;
}

static int
init_profile (struct ts_profile *profile, purpose_id_t purpose_id,
	      state_id_t state_id)
{
  struct ts_purpose *purpose;
  bool_t purpose_changed;
  struct ts_engine *engine = profile->parent;
  int ret;

  purpose = find_purpose_by_id (profile, purpose_id);
  if (!purpose)
    {
      eng_dbg ("purpose_id: %d does not exist\n", purpose_id);
      return ERROR;
    }

  ret = init_purpose (purpose, state_id);
  if (ret < 0)
    {
      eng_dbg ("init_purpose failed\n");
      return ERROR;
    }

  purpose_changed = (engine->current_purpose != purpose);
  if (purpose_changed)
    {
      eng_dbg ("PURPOSE: %s (%2d) -> %s (%2d)\n",
	   (engine->current_purpose ? (engine->current_purpose->conf.name ? engine->current_purpose->conf.name : g_str_no_name) : g_str_initial),
	   (engine->current_purpose ? engine->current_purpose->conf.puId : INVALID_PURPOSE),
	   (purpose->conf.name ? purpose->conf.name : g_str_no_name),
	   purpose->conf.puId);

      uninit_global_states (engine);
      ret = init_global_states (engine, purpose);
      if (ret < 0)
        {
          eng_dbg ("init_global_states failed\n");
          return ERROR;
        }

      ret = send_ping_event(engine);
      if (ret < 0)
        {
          eng_dbg ("send_ping_event failed\n");
        }
    }

  engine->current_purpose = purpose;

  return OK;
}

struct ts_engine *
profile_main (struct ts_engine_app *app, struct ts_profile *profile)
{
  struct ts_engine *engine;
  int ret;

  engine = &g_engine; /* only one instance */

  profile->parent = engine;

  engine->profile = profile;
  engine->current_purpose = NULL;
  engine->current_state = NULL;
  engine->app = app;
  sq_init(&engine->global_states);

  engine->sms_limit.min_interval_minutes = cloud_property_sms("min_interval_minutes", SMS_LIMIT_DEFAULT_MIN_INTERVAL_MINUTES);
  engine->sms_limit.max_per_day = cloud_property_sms("max_per_day", SMS_LIMIT_DEFAULT_MAX_PER_DAY);
  eng_dbg ("sms limit: min_interval_minutes=%d, max_per_day=%d\n", engine->sms_limit.min_interval_minutes, engine->sms_limit.max_per_day);

  ret = init_profile (engine->profile, INVALID_PURPOSE, INVALID_STATE);
  if (ret != OK)
    {
      eng_dbg ("init_profile failed\n");
      return NULL;
    }

  engine->state = ENGINE_RUNNING;

  return engine;
}

int
profile_stop(struct ts_engine *engine, bool free, bool uninit_connector)
{
  int ret;

  if (engine->state != ENGINE_RUNNING)
    {
      eng_dbg ("engine not running\n");
      return -TS_ENGINE_ERROR_NOT_RUNNING;
    }

  __ts_engine_cancel_connection();

#ifdef CONFIG_THINGSEE_CONNECTORS
  const struct ts_connector *con;

  if (uninit_connector)
    {
      ret = ts_engine_select_connector(-1, &con);
      if (ret < 0)
        {
          eng_dbg ("uninitializing connector failed\n");
        }
    }
#endif

  uninit_global_states (engine);
  ret = init_state (engine->current_state, uninit_cause);
  if (ret < 0)
    {
      eng_dbg ("init_state failed\n");
    }

  if (free)
    {
      engine->state = ENGINE_STOPPED;
    }
  else
    {
      engine->state = ENGINE_PAUSED;
    }

  return OK;
}

int
profile_continue (struct ts_engine *engine)
{
  int ret;

  if (engine->state != ENGINE_PAUSED)
    {
      eng_dbg ("engine not paused\n");
      return -TS_ENGINE_ERROR_NOT_PAUSED;
    }

  ret = init_global_states (engine, engine->current_purpose);
  if (ret < 0)
    {
      eng_dbg ("init_global_states failed\n");
      return ret;
    }

  ret = init_state (engine->current_state, init_cause);
  if (ret < 0)
    {
      eng_dbg ("init_state failed\n");
      return  ret;
    }

  engine->state = ENGINE_RUNNING;

  return OK;
}

int ts_engine_backend_update(struct ts_engine_app *app, backend_update_cb_t cb)
{
  app->backend_update.cb = cb;
  app->backend_update.profile_updated = false;

  return send_ping_event(app->engine);
}
