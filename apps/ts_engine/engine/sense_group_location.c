/****************************************************************************
 * apps/ts_engine/engine/sense_group_location.c
 *
 * Copyright (C) 2015-2017 Haltian Ltd. All rights reserved.
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
 *   Jussi Kivilinna <jussi.kivilinna@haltian.com>
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

/* ON/OFF Software power-save: minimum location interval that enable SW PSM. */

#define GPS_ONOFF_POWERSAVE_MIN_INTERVAL_SECS 50

/* ON/OFF Software power-save: how many seconds in advance before location
 *        interval should GPS be turned on. */

#define GPS_ONOFF_POWERSAVE_WAKE_ADVANCE_SECS 20

/* How many seconds GPS and MCU clock are allowed to differ? Set MCU time
 * with GPS time if difference is greater. */

#define GPS_MAX_ALLOWABLE_TIME_DIFFERENCE   3

/* Limit for GPS and MCU clock difference. Set MCU time with GPS time MCU RTC
 * not already configured or if GPS/MCU difference is smaller than this setting.
 * (Prevent setting MCU clock with invalid/corrupted GPS clock). */

#define GPS_INVALID_TIME_LIMIT              (5 * 60)

/* Private structure definitions: */

struct client_s
{
  sq_entry_t entry;
  struct ts_cause *cause;
  struct timespec last_update;
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
  int wakeup_timer;
  sq_queue_t clients;

  struct
  {
    bool gps_started :1;
    bool initialized :1;
  };

  struct
  {
    struct location_s current;
  } location;
};

static struct gps_s g_gps;

static void init_gps_struct(struct gps_s *gps)
{
  struct gps_s init_gps = { .wakeup_timer = -1, };
  *gps = init_gps;
}

static int64_t timespec_to_msec(const struct timespec * const ts)
{
  return (int64_t)ts->tv_sec * 1000 + ts->tv_nsec / (1000 * 1000);
}

static uint32_t gcd_u32(uint32_t a, uint32_t b)
{
  uint32_t r;

  if (!a || !b)
    return 0;

  while (!!(r = a % b))
    {
      a = b;
      b = r;
    }

  return b;
}

static int gps_set_time(uint16_t const year, uint8_t const month,
                        uint8_t const day, uint8_t const hour,
                        uint8_t const min, uint8_t const sec)
{
  struct timespec curr;
  struct timespec ts;
  int32_t time_diff;
  struct tm t;

  /* Prepare time structure */

  t.tm_year = year - 1900;
  t.tm_mon = month - 1;
  t.tm_mday = day;
  t.tm_hour = hour;
  t.tm_min = min;
  t.tm_sec = sec;

  ts.tv_sec = mktime(&t);
  ts.tv_nsec = 0;

  if (board_rtc_time_is_set(NULL))
    {
      /* Get current clock */

      clock_gettime(CLOCK_REALTIME, &curr);

      /* Get difference between MCU RTC time and UTC time from GPS. */

      time_diff = ts.tv_sec - curr.tv_sec;

      if (time_diff >= 0)
        {
          if (time_diff <= GPS_MAX_ALLOWABLE_TIME_DIFFERENCE)
            {
              /* Difference is with-in limits. */

              return OK;
            }

          if (time_diff >= GPS_INVALID_TIME_LIMIT)
            {
              /* Difference is over safety limits. */

              eng_dbg("GPS time ahead of MCU RTC by %d secs. "
                      "Invalid GPS time?\n", time_diff);

              return OK;
            }

          eng_dbg("GPS time ahead of MCU RTC by %d secs. "
                  "Setting RTC to GPS time.\n", time_diff);
        }
      else
        {
          if (-time_diff <= GPS_MAX_ALLOWABLE_TIME_DIFFERENCE)
            {
              /* Difference is with-in limits. */

              return OK;
            }

          if (-time_diff >= GPS_INVALID_TIME_LIMIT)
            {
              /* Difference is over safety limits. */

              eng_dbg("GPS time ahead of MCU RTC by %d secs. "
                      "Invalid GPS time?\n", -time_diff);

              return OK;
            }

          eng_dbg("MCU RTC ahead of GPS time by %d secs. "
                  "Setting RTC to GPS time.\n", -time_diff);
        }
    }

  /* Set clock */

  clock_settime(CLOCK_REALTIME, &ts);

  eng_dbg("System time set: %04d.%02d.%02d %02d:%02d:%02d.\n",
          year, month, day, hour, min, sec);

  return OK;
}

static int gps_wakeup_timer_cb(const int timer_id, const struct timespec *date,
                               void * const priv)
{
  int ret;

  if (timer_id < 0)
    return OK;

  DEBUGASSERT(timer_id == g_gps.wakeup_timer);

  ts_core_timer_stop(g_gps.wakeup_timer);
  g_gps.wakeup_timer = -1;

  ret = ts_gps_set_aiding_params (false, "/media/GPS/info.txt", 1,
                                  false, 0, 0, 0, 0);

  DEBUGASSERT(ret == OK);

  eng_dbg("Request fix acquired state\n");

  if (ts_gps_request_state (GPS_STATE_FIX_ACQUIRED,
                            GPS_FIX_ACQUIRED_TIMEOUT) != OK)
    {
      eng_dbg("Request failed\n");
    }

  return OK;
}

static void poweroff_and_start_wake_timer(gps_state_t current_state,
                                          int32_t wake_in_secs)
{
  struct timespec ts = {};

  if (current_state != GPS_STATE_POWER_OFF)
    {
      dbg ("Shutting down GPS.\n");

      /* Power down GPS */

      ts_gps_request_state (GPS_STATE_POWER_OFF, 0);
    }

  /* Stop pending wake-up timer. */

  if (g_gps.wakeup_timer >= 0)
    {
      ts_core_timer_stop(g_gps.wakeup_timer);
      g_gps.wakeup_timer = -1;
    }

  /* Start wake timer */

  clock_gettime(CLOCK_MONOTONIC, &ts);
  ts.tv_sec += wake_in_secs;
  g_gps.wakeup_timer = ts_core_timer_setup_date(&ts,
                                                gps_wakeup_timer_cb,
                                                NULL);
  if (g_gps.wakeup_timer < TS_CORE_FIRST_TIMER_ID)
    {
      eng_dbg ("ts_core_timer_setup failed\n");
    }
}

static int32_t get_secs_to_next_expected(const struct timespec *curr_ts)
{
  struct client_s *client;
  int32_t smallest = INT_MAX;

  for (client = (struct client_s *) sq_peek(&g_gps.clients);
       client;
       client = (struct client_s *) sq_next(&client->entry))
    {
      struct ts_cause *cause = client->cause;
      int32_t interval = cause->conf.measurement.interval;
      struct timespec next_update;

      if (interval <= 0)
        interval = 10000;

      next_update = client->last_update;
      next_update.tv_sec += interval / 1000;
      next_update.tv_nsec += (interval % 1000) * (1000 * 1000);

      next_update.tv_sec -= curr_ts->tv_sec;
      next_update.tv_nsec -= curr_ts->tv_nsec;
      if (next_update.tv_nsec >= 1000 * 1000 * 1000)
        {
          next_update.tv_nsec -= 1000 * 1000 * 1000;
          next_update.tv_sec++;
        }
      else if (next_update.tv_nsec < 0)
        {
          next_update.tv_nsec += 1000 * 1000 * 1000;
          next_update.tv_sec--;
        }

      if (next_update.tv_sec < smallest)
        smallest = next_update.tv_sec;
    }

  return smallest;
}

static void
gps_cb (void const * const e, void * const priv)
{
  struct gps_event_s const * const event = e;
  struct client_s *client;
  char latlon_str[32];
  int32_t next_secs;

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
        struct gps_event_target_state_s const * const target = e;

        eng_dbg ("GPS_EVENT_TARGET_STATE_TIMEOUT, target:%d, current:%d. "
                 "Timeout:%ds.\n", target->target_state, target->current_state,
                 GPS_FIX_ACQUIRED_TIMEOUT);

        poweroff_and_start_wake_timer(target->current_state,
                                      GPS_FIX_RETRY_PERIOD);

        break;
      }

    case GPS_EVENT_TARGET_STATE_REACHED:
      {
        struct gps_event_target_state_s const * const target = e;

        eng_dbg ("GPS_EVENT_TARGET_STATE_REACHED:%d.\n", target->target_state);
        (void)(target);

        break;
      }

    case GPS_EVENT_TIME:
      {
        struct gps_event_time_s const * const gps = e;

        eng_dbg ("GPS_EVENT_TIME\n");

        /* Check if time & date is available */

        if (!gps->time->validity.time || !gps->time->validity.date ||
            !gps->time->validity.fully_resolved)
          {
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
        struct timespec curr_ts;
        struct timespec curr_real_ts;
        int64_t curr_msec;

        eng_dbg ("GPS_EVENT_LOCATION\n");

        if (gps->location->fix_type == GPS_FIX_NOT_AVAILABLE)
          {
            break;
          }

        clock_gettime(CLOCK_REALTIME, &curr_real_ts);
        clock_gettime(CLOCK_MONOTONIC, &curr_ts);
        curr_msec = timespec_to_msec(&curr_ts);


        /* Update location from event */

        g_gps.location.current.time = time (0);
        g_gps.location.current.valid = true;
        g_gps.location.current.latitude = gps->location->latitude / 10000000.0;
        g_gps.location.current.longitude = gps->location->longitude / 10000000.0;
        g_gps.location.current.altitude = gps->location->height / 1000;

        /* Convert speed from mm/s to m/s */

        g_gps.location.current.speed = gps->location->ground_speed / 1000.0;
        g_gps.location.current.heading = gps->location->heading / 10000.0;
        g_gps.location.current.accuracy = gps->location->horizontal_accuracy / 1000;

        for (client = (struct client_s *) sq_peek(&g_gps.clients);
             client;
             client = (struct client_s *) sq_next(&client->entry))
          {
            struct ts_cause *cause = client->cause;
            const char hyst_percent = 5;
            int32_t interval = cause->conf.measurement.interval;
            int64_t interval_with_hyst;
            int64_t last_msec = timespec_to_msec(&client->last_update);

            if (interval <= 0)
              interval = 10000;

            interval_with_hyst = interval;
            interval_with_hyst = interval_with_hyst * (100 - hyst_percent);
            /* Round up */
            interval_with_hyst = (interval_with_hyst / 100) +
                                 interval_with_hyst % 100;

            /* Check cause interval and ignore location if it would
             * be given to cause too fast. */

            if ((curr_msec - last_msec) <= interval_with_hyst)
              continue;

            /* Increase last_update as multiple of 'interval'. This is to keep
             * events synchronized. */

            /*eng_dbg("old_last_msec: %lld, curr_msec: %lld, diff: %lld, "
                    "interval: %d\n",
                    last_msec, curr_msec, curr_msec - last_msec, interval);*/

            last_msec += ((curr_msec - last_msec + interval / 2) / interval)
                          * interval;

            /*eng_dbg("new_last_msec: %lld, curr_msec: %lld, diff: %lld\n",
                    last_msec, curr_msec, -(curr_msec - last_msec));*/

            client->last_update.tv_sec = last_msec / 1000;
            client->last_update.tv_nsec = (last_msec % 1000) / (1000 * 1000);

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

            if (handle_cause_event (cause, &curr_real_ts))
              {
                break;
              }
          }

        /* Software power-save:
         *  - Check all senses and how long time till next location
         *    is expected (depending on 'last_update' and interval).
         *  - If next location is expected over 1 minute from now,
         *    turn off GPS and launch idle-timer with expiry in
         *    'expected_time - 20' seconds.
         */

        next_secs = get_secs_to_next_expected(&curr_ts);
        eng_dbg("Expecting next GPS location in %d seconds.\n", next_secs);

        if (next_secs >= GPS_ONOFF_POWERSAVE_MIN_INTERVAL_SECS)
          {
            next_secs -= GPS_ONOFF_POWERSAVE_WAKE_ADVANCE_SECS;

            eng_dbg ("GPS Software power-save, power-off GPS and "
                     "wake-up in %d seconds.\n", (int)next_secs);

            poweroff_and_start_wake_timer(ts_gps_get_state(), next_secs);
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
  struct client_s *client, *other;
  uint32_t navigation_rate;
  uint32_t event_mask;
  int ret;

  DEBUGASSERT(GPS_ONOFF_POWERSAVE_MIN_INTERVAL_SECS >
              GPS_ONOFF_POWERSAVE_WAKE_ADVANCE_SECS);
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
      if (!g_gps.initialized)
        {
          init_gps_struct (&g_gps);

          eng_dbg("Initializing u-blox GPS module\n");

          ret = ts_gps_initialize ();
          DEBUGASSERT(ret == OK);

          g_gps.initialized = true;
        }

      DEBUGASSERT(g_gps.wakeup_timer < 0);

      ret = ts_gps_set_aiding_params (false, "/media/GPS/info.txt", 1,
                                      false, 0, 0, 0, 0);
      DEBUGASSERT(ret == OK);

      navigation_rate = cause->conf.measurement.interval > 0 ?
                        cause->conf.measurement.interval : 10000;

      eng_dbg("Set navigation rate to: %u ms\n", navigation_rate);

      ret = ts_gps_config(GPS_CONFIG_NAVIGATION_RATE, &navigation_rate);
      if (ret < 0)
        {
          eng_dbg("ts_gps_config failed\n");
          goto errout_free;
        }

      eng_dbg("Register gps callback\n");

      event_mask = GPS_EVENT_TARGET_STATE_NOT_REACHED |
                   GPS_EVENT_TARGET_STATE_REACHED |
                   GPS_EVENT_TARGET_STATE_TIMEOUT |
                   GPS_EVENT_LOCATION | GPS_EVENT_TIME;

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
  else
    {
      /* Get first navigation rate. */

      navigation_rate = cause->conf.measurement.interval > 0 ?
                        cause->conf.measurement.interval : 10000;

      /* Get navigation rates for other clients and calculate
       * greatest common divisor that will be used as navigation rate. */

      other = (struct client_s *) sq_peek(&g_gps.clients);
      while (other)
        {
          uint32_t other_nav_rate;

          other_nav_rate = (other->cause &&
                             other->cause->conf.measurement.interval > 0) ?
                             other->cause->conf.measurement.interval : 10000;

          navigation_rate = gcd_u32(other_nav_rate, navigation_rate);

          other = (struct client_s *) sq_next(&other->entry);
        }

      eng_dbg("Set navigation rate to: %u ms\n", navigation_rate);

      ret = ts_gps_config(GPS_CONFIG_NAVIGATION_RATE, &navigation_rate);
      if (ret < 0)
        {
          eng_dbg("ts_gps_config failed\n");
        }
    }

  g_gps.refcount++;

  sq_addlast (&client->entry, &g_gps.clients);
  clock_gettime(CLOCK_MONOTONIC, &client->last_update);

  return OK;

errout_free:
  free (client);

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
      if (g_gps.wakeup_timer >= 0)
        {
          ts_core_timer_stop(g_gps.wakeup_timer);
          g_gps.wakeup_timer = -1;
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
