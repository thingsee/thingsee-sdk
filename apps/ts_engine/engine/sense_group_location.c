/****************************************************************************
 * apps/ts_engine/engine/sense_group_location.c
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

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <assert.h>

#include <arch/board/board-reset.h>
#include <apps/thingsee/modules/ts_gps.h>
#include <apps/thingsee/ts_core.h>

#include "eng_dbg.h"
#include "parse.h"
#include "sense.h"
#include "execute.h"

/* Timeout in seconds to reach fix acquired state. */

#define GPS_FIX_ACQUIRED_TIMEOUT  (3*60)

/* Timeout in seconds to retry position fix. */

#define GPS_FIX_RETRY_PERIOD      (5*60)

struct client_s
{
  sq_entry_t entry;
  struct ts_cause *cause;
};

struct location_s
{
  bool valid;
  time_t time;
  double latitude;
  double longitude;
  int32_t altitude;
  double speed; /* km/h */
  double heading; /* degrees */
  uint32_t accuracy; /* meters */
};

struct gps_s
{
  int refcount;
  int fix_retry_timer;
  sq_queue_t clients;

  struct
  {
    bool gps_started :1;
    bool initialized :1;
  };

  struct
  {
    struct location_s current;
    struct location_s last;
  } location;
};

static struct gps_s g_gps =
  { .refcount = 0 , .initialized = 0 };

static int
gps_set_time (uint16_t const year, uint8_t const month, uint8_t const day,
	      uint8_t const hour, uint8_t const min, uint8_t const sec)
{
  struct timespec ts;
  struct tm t;

  /* Prepare time structure */

  t.tm_year = year - 1900;
  t.tm_mon = month - 1;
  t.tm_mday = day;
  t.tm_hour = hour;
  t.tm_min = min;
  t.tm_sec = sec;

  ts.tv_sec = mktime (&t);
  ts.tv_nsec = 0;

  /* Set clock */

  clock_settime (CLOCK_REALTIME, &ts);

  eng_dbg ("System time set: %04d.%02d.%02d %02d:%02d:%02d.\n", year, month, day,
      hour, min, sec);

  return OK;
}

static int
gps_fix_retry_timeout(const int timer_id, const struct timespec *date,
                       void * const priv)
{
  int ret;
  bool time_set = false;

  if (timer_id == g_gps.fix_retry_timer)
    {
      ts_core_timer_stop(g_gps.fix_retry_timer);
      g_gps.fix_retry_timer = -1;

      if (board_rtc_time_is_set(NULL))
        {
          time_set = true;
        }

      ret = ts_gps_set_aiding_params (time_set,
        "/media/GPS/info.txt",
        1,
        g_gps.location.last.latitude?1:0,
        g_gps.location.last.latitude,
        g_gps.location.last.longitude,
        g_gps.location.last.altitude,
        g_gps.location.last.accuracy);

      DEBUGASSERT(ret == OK);

      eng_dbg("Request fix acquired state\n");

      if (ts_gps_request_state (GPS_STATE_FIX_ACQUIRED,
        GPS_FIX_ACQUIRED_TIMEOUT) != OK)
        {
          eng_dbg("Request failed\n");
        }
    }

  return OK;
}

static void
gps_cb (void const * const e, void * const priv)
{
  struct gps_event_s const * const event = e;
  struct client_s *client;
  char latlon_str[32];

  DEBUGASSERT(event);

  /* Check if GPS was started by Thingsee engine */

  if (!g_gps.gps_started)
    {
      return;
    }

  switch (event->id)
    {
    case GPS_EVENT_TARGET_STATE_NOT_REACHED:
    case GPS_EVENT_TARGET_STATE_TIMEOUT:
      {
        struct timespec ts = {};
        struct gps_event_target_state_s const * const target = e;

        eng_dbg ("GPS_EVENT_TARGET_STATE_TIMEOUT, target:%d, current:%d. "
          "Timeout:%ds.\n",
          target->target_state, target->current_state,
          GPS_FIX_ACQUIRED_TIMEOUT);

        if (target->current_state != GPS_STATE_POWER_OFF)
          {
            dbg ("Shutting down GPS.\n");

            /* Power down GPS */

            ts_gps_request_state (GPS_STATE_POWER_OFF, 0);
          }

        /* Start retry timer */

        clock_gettime(CLOCK_MONOTONIC, &ts);
        ts.tv_sec += GPS_FIX_RETRY_PERIOD;
        g_gps.fix_retry_timer = ts_core_timer_setup_date(&ts,
          gps_fix_retry_timeout, NULL);

        if (g_gps.fix_retry_timer < TS_CORE_FIRST_TIMER_ID)
          {
            eng_dbg ("ts_core_timer_setup failed\n");
          }

        break;
      }

    case GPS_EVENT_TARGET_STATE_REACHED:
      {
        struct gps_event_target_state_s const * const target = e;

        eng_dbg ("GPS_EVENT_TARGET_STATE_REACHED:%d.\n", target->target_state);

        if ((target->target_state == GPS_STATE_FIX_ACQUIRED) &&
          (g_gps.fix_retry_timer >= TS_CORE_FIRST_TIMER_ID))
          {
            ts_core_timer_stop(g_gps.fix_retry_timer);
            g_gps.fix_retry_timer = -1;
          }

        break;
      }

    case GPS_EVENT_TIME:
      {
        struct gps_event_time_s const * const gps = e;

        eng_dbg ("GPS_EVENT_TIME\n");

        /* Check if time & date is available */

        if (!gps->time->validity.time || !gps->time->validity.date)
          {
            break;
          }

        if (!ts_gps_callback_unregister (gps_cb))
          {
            eng_dbg ("Disable GPS time event\n");

            if (ts_gps_callback_register (GPS_EVENT_TARGET_STATE_NOT_REACHED |
              GPS_EVENT_TARGET_STATE_REACHED | GPS_EVENT_TARGET_STATE_TIMEOUT |
              GPS_EVENT_LOCATION, gps_cb, NULL))
              {
                eng_dbg ("failed to re-enable GPS callback\n");
              }
          }

        if (board_rtc_time_is_set(NULL))
          {
            /* System time is already set */

            break;
          }

        /* Set system time */

        gps_set_time (gps->time->year, gps->time->month, gps->time->day,
          gps->time->hour, gps->time->min, gps->time->sec);

        break;
      }

    case GPS_EVENT_LOCATION:
      {
        struct gps_event_location_s const * const gps = e;

        eng_dbg ("GPS_EVENT_LOCATION\n");

        if (gps->location->fix_type == GPS_FIX_NOT_AVAILABLE)
          {
            break;
          }

        /* Update location from event */

        g_gps.location.current.valid = true;
        g_gps.location.current.time = time (0);
        g_gps.location.current.latitude = gps->location->latitude / 10000000.0;
        g_gps.location.current.longitude = gps->location->longitude / 10000000.0;
        g_gps.location.current.altitude = gps->location->height / 1000;

        /* Convert speed from mm/s to m/s */

        g_gps.location.current.speed = gps->location->ground_speed / 1000.0;
        g_gps.location.current.heading = gps->location->heading / 10000.0;
        g_gps.location.current.accuracy = gps->location->horizontal_accuracy / 1000;

        /* Update last known location */

        memcpy (&g_gps.location.last, &g_gps.location.current,
          sizeof(g_gps.location.last));

        client = (struct client_s *) sq_peek(&g_gps.clients);

        while (client)
          {
            struct ts_cause *cause;

            cause = client->cause;

            switch (cause->dyn.sense_value.sId)
            {
              case SENSE_ID_IS_INSIDE_GEOFENCE:
                {
                  struct ts_value *items;

                  items = malloc(7 * sizeof(*items));
                  if (!items)
                    {
                      eng_dbg("malloc %d failed\n", 7 * sizeof(*items));
                      break;
                    }

                  items[0].valuetype = VALUEDOUBLE;
                  items[0].valuedouble = g_gps.location.current.latitude;
                  items[1].valuetype = VALUEDOUBLE;
                  items[1].valuedouble = g_gps.location.current.longitude;
                  items[2].valuetype = VALUEINT32;
                  items[2].valueint32 = g_gps.location.current.altitude;
                  items[3].valuetype = VALUEUINT32;
                  items[3].valueuint32 = g_gps.location.current.accuracy;
                  items[4].valuetype = VALUEUINT32;
                  items[4].valueuint32 = g_gps.location.current.time;
                  items[5].valuetype = VALUEDOUBLE;
                  items[5].valuedouble = g_gps.location.current.speed;
                  items[6].valuetype = VALUEDOUBLE;
                  items[6].valuedouble = g_gps.location.current.heading;

                  cause->dyn.sense_value.value.valuetype = VALUEARRAY;
                  cause->dyn.sense_value.value.valuearray.number_of_items = 7;
                  cause->dyn.sense_value.value.valuearray.items = items;
                }
                break;
              case SENSE_ID_LATITUDE:
                cause->dyn.sense_value.value.valuedouble =
                    g_gps.location.current.latitude;
                break;
              case SENSE_ID_LONGITUDE:
                cause->dyn.sense_value.value.valuedouble =
                    g_gps.location.current.longitude;
                break;
              case SENSE_ID_ALTITUDE:
                cause->dyn.sense_value.value.valueint32 =
                    g_gps.location.current.altitude;
                break;
              case SENSE_ID_ACCURACY:
                cause->dyn.sense_value.value.valueuint32 =
                    g_gps.location.current.accuracy;
                break;
              case SENSE_ID_GPS_TIMESTAMP:
                cause->dyn.sense_value.value.valueuint32 =
                    (uint32_t) g_gps.location.current.time;
                break;
              case SENSE_ID_GROUND_SPEED:
                cause->dyn.sense_value.value.valuedouble =
                    g_gps.location.current.speed;
                break;
              case SENSE_ID_HEADING:
                cause->dyn.sense_value.value.valuedouble =
                    g_gps.location.current.heading;
                break;
              case SENSE_ID_LATLON:
                snprintf(latlon_str, sizeof(latlon_str), "%.7f,%.7f",
                        g_gps.location.current.latitude,
                        g_gps.location.current.longitude);
                cause->dyn.sense_value.value.valuestring = latlon_str;
                break;
              default:
                continue;
            }

            if (handle_cause_event (cause))
              {
                break;
              }

            client = (struct client_s *) sq_next(&client->entry);
          }
        break;
      }

    default:
      break;
    }
}

#if 0 /* for testing if fix not possible */
static int fake_event(const int timer_id, void * const priv)
{
  struct gps_event_location_s event;
  struct gps_location_s location;

  event.super.id = GPS_EVENT_LOCATION;

  location.fix_type = GPS_FIX_3D;
  location.longitude = 65.666;
  location.latitude = 25.666;

  event.location = &location;

  gps_cb(&event, priv);

  return OK;
}
#endif

int
sense_location_init (struct ts_cause *cause)
{
  int ret;
  struct client_s *client;
  uint32_t navigation_rate;
  uint32_t event_mask;
  bool time_set = false;

  DEBUGASSERT(cause);

  client = zalloc (sizeof(*client));
  if (!client)
    {
      eng_dbg("zalloc failed\n");
      return ERROR;
    }

  client->cause = cause;

  if (g_gps.refcount == 0)
    {
      if (g_gps.initialized == false)
        {
          memset (&g_gps, 0, sizeof(g_gps));

          eng_dbg("Initializing u-blox GPS module\n");

          ret = ts_gps_initialize ();
          DEBUGASSERT(ret == OK);

          g_gps.initialized = true;
        }

      if (board_rtc_time_is_set(NULL))
        {
          eng_dbg("Use system time for aiding\n");
          time_set = true;
        }

      eng_dbg("Latitude %.7f, longitude %.7f, altitude %dm, accuracy %dm\n",
        g_gps.location.last.latitude,
        g_gps.location.last.longitude,
        g_gps.location.last.altitude,
        g_gps.location.last.accuracy);

      ret = ts_gps_set_aiding_params (time_set,
        "/media/GPS/info.txt",
        1,
        g_gps.location.last.latitude?1:0,
        g_gps.location.last.latitude,
        g_gps.location.last.longitude,
        g_gps.location.last.altitude,
        g_gps.location.last.accuracy);

      DEBUGASSERT(ret == OK);

      if (cause->conf.measurement.interval > 0)
        {
          navigation_rate = (uint32_t)cause->conf.measurement.interval;

          eng_dbg("Set navigation rate to: %u ms\n", navigation_rate);

          ret = ts_gps_config(GPS_CONFIG_NAVIGATION_RATE, &navigation_rate);
          if (ret < 0)
            {
              eng_dbg("ts_gps_config failed\n");
              goto errout_free;
            }
        }

      eng_dbg("Register gps callback\n");

      event_mask = GPS_EVENT_TARGET_STATE_NOT_REACHED | GPS_EVENT_TARGET_STATE_REACHED |
        GPS_EVENT_TARGET_STATE_TIMEOUT | GPS_EVENT_LOCATION;

      if (!time_set)
        {
          eng_dbg("Register for time event\n");
          event_mask |= GPS_EVENT_TIME;
        }

      ret = ts_gps_callback_register (event_mask, gps_cb, NULL);
      DEBUGASSERT(ret == OK);

      eng_dbg("Search fix\n");

      ret = ts_gps_request_state (GPS_STATE_FIX_ACQUIRED,
        GPS_FIX_ACQUIRED_TIMEOUT);
      if (ret != OK)
        {
          eng_dbg("ts_gps_request_state failed\n");
          goto errout_free;
        }

      g_gps.gps_started = true;

      sq_init(&g_gps.clients);

#if 0
      ts_core_timer_setup(TS_TIMER_TYPE_TIMEOUT , 1000, fake_event, cause);
#endif
    }

  sq_addlast (&client->entry, &g_gps.clients);

  g_gps.refcount++;

  return OK;

  errout_free: free (client);

  return ERROR;
}

int
sense_location_uninit (struct ts_cause *cause)
{
  struct client_s *client;
  int ret;

  DEBUGASSERT(cause);

  client = (struct client_s *) sq_peek(&g_gps.clients);

  while (client)
    {
      if (client->cause == cause)
        {
          sq_rem(&client->entry, &g_gps.clients);
          free(client);
          break;
        }
      client = (struct client_s *) sq_next(&client->entry);
    }

  g_gps.refcount--;

  if (g_gps.refcount == 0)
    {
      if (g_gps.fix_retry_timer >= TS_CORE_FIRST_TIMER_ID)
        {
          ts_core_timer_stop(g_gps.fix_retry_timer);
          g_gps.fix_retry_timer = -1;
        }

      if (ts_gps_get_state () != GPS_STATE_POWER_OFF)
        {
          eng_dbg("Power off gps\n");

          ts_gps_request_state (GPS_STATE_POWER_OFF, 0);
          g_gps.gps_started = false;
        }

      eng_dbg("Unregister gps callback\n");

      ret = ts_gps_callback_unregister (gps_cb);
      DEBUGASSERT(ret == OK);
    }

  return OK;
}
