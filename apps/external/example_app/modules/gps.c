/****************************************************************************
 * modules/gps.c
 *
 * Copyright (C) 2015-2016 Haltian Ltd.
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
#include <string.h>
#include <time.h>
#include <math.h>

#include <assert.h>

#include <arch/board/board-reset.h>
#include <apps/thingsee/modules/ts_gps.h>
#include <apps/thingsee/ts_core.h>

#include <example_app_dbg.h>
#include <example_app_gps.h>

/* Timeout in seconds to reach fix acquired state. */

#define GPS_FIX_ACQUIRED_TIMEOUT            (90)

/* Timeout in seconds for initial fix attempt. */

#define GPS_INITIAL_FIX_ACQUIRED_TIMEOUT    (5 * 60)

/* How many seconds GPS and MCU clock are allowed to differ? Set MCU time
 * with GPS time if difference is greater. */

#define GPS_MAX_ALLOWABLE_TIME_DIFFERENCE   3

/* Limit for GPS and MCU clock difference. Set MCU time with GPS time MCU RTC
 * not already configured or if GPS/MCU difference is smaller than this setting.
 * (Prevent setting MCU clock with invalid/corrupted GPS clock). */

#define GPS_INVALID_TIME_LIMIT              (5 * 60)

/* Watchdog timeout for GPS. */

#define GPS_WATCHDOG_TIMER_MIN_SECS         (GPS_INITIAL_FIX_ACQUIRED_TIMEOUT + 1)
#define GPS_WATCHDOG_TIMER_MAX_SECS         (30 * 60)
#define GPS_WATCHDOG_COUNT_END              2

struct gps_s
{
  struct
  {
    bool gps_started:1;
    bool initialized:1;
  };

  struct
  {
    struct location_s current;
  } location;

  exapp_gps_callback_t callback;
  void *priv;

  int watchdog_timerid;
  int watchdog_count;
  uint32_t watchdog_timer_secs;
};

static struct gps_s g_gps =
{
  .initialized = 0,
  .watchdog_timerid = -1,
  .watchdog_count = 0,
};

static int recover_timerid = -1;

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

              exapp_dbg("GPS time ahead of MCU RTC by %d secs. "
                        "Invalid GPS time?\n", time_diff);

              return OK;
            }

          exapp_dbg("GPS time ahead of MCU RTC by %d secs. "
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

              exapp_dbg("GPS time ahead of MCU RTC by %d secs. "
                        "Invalid GPS time?\n", -time_diff);

              return OK;
            }

          exapp_dbg("MCU RTC ahead of GPS time by %d secs. "
                    "Setting RTC to GPS time.\n", -time_diff);
        }
    }

  /* Set clock */

  clock_settime(CLOCK_REALTIME, &ts);

  exapp_dbg("System time set: %04d.%02d.%02d %02d:%02d:%02d.\n",
            year, month, day, hour, min, sec);

  return OK;
}

static int fix_recover_timer_cb(const int timer_id,
    const struct timespec *date, void * const priv)
{
  /* Indicate timeout with NULL location */

  (g_gps.callback) ? g_gps.callback(NULL, priv) : (void)0;

  return OK;
}

static int gps_watchdog_cb(const int timer_id, const struct timespec *date,
                           void * const priv)
{
  struct timespec ts;

  g_gps.watchdog_count++;

  g_gps.watchdog_timerid = -1;

  clock_gettime(CLOCK_MONOTONIC, &ts);
  ts.tv_sec += g_gps.watchdog_timer_secs;
  g_gps.watchdog_timerid = ts_core_timer_setup_date(&ts, gps_watchdog_cb, NULL);
  DEBUGASSERT(g_gps.watchdog_timerid >= 0);

  exapp_dbg("count: %d\n", g_gps.watchdog_count);

  if (g_gps.watchdog_count >= GPS_WATCHDOG_COUNT_END)
    {
      /* Indicate timeout with NULL location */

      (g_gps.callback) ? g_gps.callback(NULL, priv) : (void)0;
    }

  return OK;
}

static void gps_cb(void const * const e, void * const priv)
{
  struct gps_event_s const * const event = e;

  DEBUGASSERT(event);

  /* Check if GPS was started by Thingsee engine */

  if (!g_gps.gps_started)
    {
      return;
    }

  /* Reset watchdog */

  g_gps.watchdog_count = 0;

  switch (event->id)
    {
    case GPS_EVENT_TARGET_STATE_NOT_REACHED:
      {
        struct gps_event_target_state_s const * const target = e;

        exapp_dbg("GPS_EVENT_TARGET_STATE_NOT_REACHED:%d, current:%d.\n",
               target->target_state, target->current_state);

        if (target->current_state != GPS_STATE_POWER_OFF)
          {
            exapp_dbg("Shutting down GPS.\n");

            /* Power down GPS */

            ts_gps_request_state(GPS_STATE_POWER_OFF, 0);
          }

        /* Indicate error with NULL location */

        g_gps.callback(NULL, priv);

        break;
      }

    case GPS_EVENT_TARGET_STATE_TIMEOUT:
      {
        struct gps_event_target_state_s const * const target = e;

        exapp_dbg("GPS_EVENT_TARGET_STATE_TIMEOUT\n");

        if (target->target_state == GPS_STATE_FIX_ACQUIRED)
          {
            exapp_dbg("target:%d, current:%d.\n", target->target_state,
              target->current_state);
          }

        if (target->current_state != GPS_STATE_POWER_OFF)
          {
            exapp_dbg("Shutting down GPS.\n");

            /* Power down GPS */

            ts_gps_request_state(GPS_STATE_POWER_OFF, 0);
          }

        /* Indicate timeout with NULL location */

        g_gps.callback(NULL, priv);

        break;
      }

    case GPS_EVENT_TARGET_STATE_REACHED:
      {
        struct gps_event_target_state_s const * const target = e;

        exapp_dbg("GPS_EVENT_TARGET_STATE_REACHED:%d.\n", target->target_state);

        break;
      }

    case GPS_EVENT_STATE_CHANGE:
      {
        struct gps_event_state_change_s const * const cevent = e;
        static gps_state_t gps_state;
        struct timespec ts;

        exapp_dbg("GPS_EVENT_STATE_CHANGE:%d.\n", cevent->state);

        if (cevent->state == GPS_STATE_SEARCHING_FIX &&
            gps_state == GPS_STATE_FIX_ACQUIRED)
          {
            /* Fix lost (fix acquired -> searching fix), start fix recover timeout */

            if (recover_timerid >= 0)
              {
                exapp_dbg("fix recover timer already running\n");
                break;
              }

            exapp_dbg("start fix recover timer\n");

            clock_gettime(CLOCK_MONOTONIC, &ts);
            ts.tv_sec += GPS_FIX_ACQUIRED_TIMEOUT;
            recover_timerid = ts_core_timer_setup_date(&ts, fix_recover_timer_cb, NULL);
            DEBUGASSERT(recover_timerid >= 0);
          }
        else if (cevent->state != GPS_STATE_SEARCHING_FIX)
          {
            if (recover_timerid >= 0)
              {
                exapp_dbg("stop fix recover timer\n");

                ts_core_timer_stop(recover_timerid);
                recover_timerid = -1;
              }
          }

        gps_state = cevent->state;

        break;
      }

    case GPS_EVENT_TIME:
      {
        struct gps_event_time_s const * const gps = e;

        //exapp_dbg("GPS_EVENT_TIME\n");

        /* Check if time & date is available */

        if (!gps->time->validity.time || !gps->time->validity.date ||
            !gps->time->validity.fully_resolved)
          {
            break;
          }

        /* Set system time */

        gps_set_time(gps->time->year, gps->time->month, gps->time->day,
                     gps->time->hour, gps->time->min, gps->time->sec);

        break;
      }

    case GPS_EVENT_LOCATION:
      {
        struct gps_event_location_s const * const gps = e;

        exapp_dbg("GPS_EVENT_LOCATION\n");

        if (gps->location->fix_type == GPS_FIX_NOT_AVAILABLE)
          {
            break;
          }

        /* Update location from event */

        g_gps.location.current.valid = true;
        g_gps.location.current.time = time(0);
        g_gps.location.current.latitude =
            gps->location->latitude / 10000000.0;
        g_gps.location.current.longitude =
            gps->location->longitude / 10000000.0;
        g_gps.location.current.altitude =
            round(gps->location->height / 1000.0);

        /* Convert speed from mm/s to m/s */

        g_gps.location.current.speed =
            round(gps->location->ground_speed / 1000.0);
        g_gps.location.current.heading =
            round(gps->location->heading / 100000.0);
        g_gps.location.current.accuracy =
            round(gps->location->horizontal_accuracy / 1000.0);

        g_gps.callback(&g_gps.location.current, priv);

        break;
      }

    default:
      break;
    }
}

static int gps_set_navigation_rate(const uint32_t rate)
{
  int ret;

  exapp_dbg("Set navigation rate to: %d ms\n", rate);

  ret = ts_gps_config(GPS_CONFIG_NAVIGATION_RATE, &rate);
  if (ret < 0)
    {
      exapp_dbg("ts_gps_config failed, gps state:%d\n", ts_gps_get_state());

      if (ts_gps_get_state() != GPS_STATE_SEARCHING_FIX &&
          ts_gps_get_state() != GPS_STATE_FIX_ACQUIRED)
        {
          return ERROR;
        }

      /* GPS already active, so already GPS has correct config.
       * Ignore error. */
    }

  return OK;
}

int exapp_gps_init(const struct gps_config_s *config,
                   exapp_gps_callback_t callback, void * priv,
                   bool initial_fix)
{
  int ret;
  uint32_t event_mask;
  uint32_t timeout;
  int gps_state;
  struct timespec ts;

  if (g_gps.initialized == false)
    {
      exapp_dbg("Initializing u-blox GPS module\n");

      ret = ts_gps_initialize();
      DEBUGASSERT(ret == OK);

      g_gps.initialized = true;
    }

  gps_state = ts_gps_get_state();
  if (gps_state != GPS_STATE_POWER_OFF)
    {
      exapp_dbg("BUG! GPS not powered off, %d\n", gps_state);
      /* Should be powered off. */
      DEBUGASSERT(gps_state != GPS_STATE_POWER_OFF);
    }

  if (config->rate > 0)
    {
      ret = gps_set_navigation_rate(config->rate);
      if (ret < 0)
        {
          exapp_dbg("gps_set_navigation_rate failed\n");
          return ERROR;
        }
    }

  exapp_dbg("Register gps callback\n");

  event_mask = GPS_EVENT_TARGET_STATE_NOT_REACHED
      | GPS_EVENT_TARGET_STATE_REACHED | GPS_EVENT_TARGET_STATE_TIMEOUT
      | GPS_EVENT_LOCATION | GPS_EVENT_STATE_CHANGE | GPS_EVENT_TIME;

  ret = ts_gps_callback_register(event_mask, gps_cb, NULL);
  DEBUGASSERT(ret == OK);

  exapp_dbg("Search fix\n");

  if (initial_fix)
    timeout = GPS_INITIAL_FIX_ACQUIRED_TIMEOUT;
  else
    timeout = GPS_FIX_ACQUIRED_TIMEOUT;

  ret = ts_gps_request_state(GPS_STATE_FIX_ACQUIRED, timeout);
  if (ret != OK)
    {
      exapp_dbg("ts_gps_request_state failed\n");
      return ERROR;
    }

  g_gps.gps_started = true;
  g_gps.callback = callback;
  g_gps.priv = priv;

  g_gps.watchdog_timer_secs = config->watchdog_secs;
  if (g_gps.watchdog_timer_secs < GPS_WATCHDOG_TIMER_MIN_SECS)
    g_gps.watchdog_timer_secs = GPS_WATCHDOG_TIMER_MIN_SECS;
  if (g_gps.watchdog_timer_secs > GPS_WATCHDOG_TIMER_MAX_SECS)
    g_gps.watchdog_timer_secs = GPS_WATCHDOG_TIMER_MAX_SECS;

  DEBUGASSERT(g_gps.watchdog_timerid < 0);

  clock_gettime(CLOCK_MONOTONIC, &ts);
  ts.tv_sec += g_gps.watchdog_timer_secs;
  g_gps.watchdog_timerid = ts_core_timer_setup_date(&ts, gps_watchdog_cb, NULL);
  DEBUGASSERT(g_gps.watchdog_timerid >= 0);

  return OK;
}

int exapp_gps_uninit(void)
{
  int ret;

  if (recover_timerid >= 0)
    {
      exapp_dbg("stop fix recover timer\n");

      ts_core_timer_stop(recover_timerid);
      recover_timerid = -1;
    }

  if (g_gps.watchdog_timerid >= 0)
    {
      exapp_dbg("stop watchdog timer\n");

      ts_core_timer_stop(g_gps.watchdog_timerid);
      g_gps.watchdog_timerid = -1;
    }

  if (ts_gps_get_state() != GPS_STATE_POWER_OFF)
    {
      exapp_dbg("Power off gps\n");

      ts_gps_request_state(GPS_STATE_POWER_OFF, 0);
      g_gps.gps_started = false;
    }

  exapp_dbg("Unregister gps callback\n");

  ret = ts_gps_callback_unregister(gps_cb);
  DEBUGASSERT(ret == OK);

  return OK;
}
