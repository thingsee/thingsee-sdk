/****************************************************************************
 * apps/ts_engine/engine/system_config.c
 *
 * Copyright (C) 2017 Haltian Ltd. All rights reserved.
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
 *   Jussi Kivilinna <jussi.kivilinna@haltian.com>
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <debug.h>
#include <assert.h>
#include <string.h>
#include <math.h>
#include <string.h>

#include <arch/board/board-reset.h>

#include "util.h"
#include "eng_dbg.h"
#include "system_config.h"

#ifndef CONFIG_ARCH_SIM
#include "../../thingsee/charger/bq24251_module.h"
#endif

/*
 * Parameters for SYSTEM.JSN:
 *
 * {
 *   "forcePowerOn":Boolean,        // Force device to turn-on when powered
 *   "timezoneOffset":Number,       // Hour offset of local timezone to GMT
 *   "charging":{
 *     "disabledDaily":{            // Daily control for charging. Defined
 *                                  // as clock range when charging is disallowed.
 *       "start": String ("HH:MM"), // Start of charging disabled clock range
 *       "end": String ("HH:MM")    // End of charging disabled clock range
 *     }
 *   }
 * }
 */

struct system_config_s
{
  bool force_power_on;
  float tz_offset_hours;

  struct
  {
    /* Controls for charging. */

    struct
    {
      /* Charging disabled will be disabled _daily_ starting 'start_minutes'
       * and ending at 'end_minutes'. Input format in configuration file is
       * "HH:MM". */

      unsigned int start;
      unsigned int end;
    } daily_off;
  } charging;
};

static struct system_config_s g_system_config;
static struct
{
  int charger_timer_id;
} g_priv = { .charger_timer_id = -1 };

static const struct system_config_s system_config_defaults =
  {
    .force_power_on = false,
    .tz_offset_hours = 0,
    .charging =
      {
        .daily_off =
          {
            .start = 0,
            .end = 0,
          },
      },
  };

static int charger_allowed_timer_cb(const int timer_id, void * const priv)
{
  g_priv.charger_timer_id = -1; /* one-shot */

#ifndef CONFIG_ARCH_SIM
#ifdef CONFIG_THINGSEE_CHARGER_MODULE
  bq24251_set_charging_allowed(system_config_charging_allowed());
#endif
#endif

  return 0;
}

static int parse_hh_mm(cJSON *obj, bool limit_range)
{
  int minutes = 0;

  if (cJSON_type(obj) == cJSON_String)
    {
      int hh = 0;
      unsigned int mm = 0;

      if (sscanf(cJSON_string(obj), "%d:%u", &hh, &mm) == 2)
        {
          minutes = 60 * hh + mm * ((hh >= 0) * 2 - 1);
        }
    }
  else
    {
      minutes = lround(cJSON_double(obj) * 60);
    }

  if (limit_range)
    {
      /* Limit range to 00:00 .. 23:59 */

      if (minutes < 0)
        minutes += (-minutes / (24 * 60) + 1) * (24 * 60);

      minutes %= (24 * 60);
    }

  return minutes;
}

void system_config_load(void)
{
  cJSON *value_obj = NULL;
  cJSON *charging_obj = NULL;
  cJSON *disabled_obj = NULL;
  cJSON *obj;

  /* Load defaults. */

  g_system_config = system_config_defaults;

  /* Parse JSON file. */

  obj = __ts_engine_sdcard_read_json(SDCARD_SYSTEM_CONFIG_FILENAME);
  if (!obj)
    {
      return;
    }

  /* Parse parameters from JSON. */

  value_obj = cJSON_GetObjectItem(obj, "forcePowerOn");
  if (value_obj)
    g_system_config.force_power_on = cJSON_boolean(value_obj);

  value_obj = cJSON_GetObjectItem(obj, "timezoneOffset");
  if (value_obj)
    g_system_config.tz_offset_hours = parse_hh_mm(value_obj, false) / 60.0f;

  charging_obj = cJSON_GetObjectItem(obj, "charging");
  if (charging_obj)
    disabled_obj = cJSON_GetObjectItem(charging_obj, "disabledDaily");

  if (disabled_obj)
    {
      value_obj = cJSON_GetObjectItem(disabled_obj, "start");
      if (value_obj)
        g_system_config.charging.daily_off.start = parse_hh_mm(value_obj, true);

      value_obj = cJSON_GetObjectItem(disabled_obj, "end");
      if (value_obj)
        g_system_config.charging.daily_off.end = parse_hh_mm(value_obj, true);
    }

  cJSON_Delete(obj);
}

bool system_config_force_power_on(void)
{
  return g_system_config.force_power_on;
}

bool system_config_charging_allowed(void)
{
  unsigned int start_secs =
      g_system_config.charging.daily_off.start * 60;
  unsigned int end_secs =
      g_system_config.charging.daily_off.end * 60;
  unsigned int seconds_to_change;
  unsigned int tm_dsec; /* second of day. */
  int tz_offset_secs;
  struct tm tm = {};
  bool allowed;
  time_t now;

  if (start_secs == end_secs)
    {
      return true;
    }

  now = time(0);
  if (!board_rtc_time_is_set(NULL) || ts_core_is_date_before_compile_date(now))
    {
      /* Device does not have correct time yet. */

      allowed = true;
      seconds_to_change = 60;
      goto register_timer;
    }

  /* Get current minute of day. */

  tz_offset_secs = g_system_config.tz_offset_hours * (60 * 60);
  if (tz_offset_secs < 0)
    now -= -tz_offset_secs;
  else
    now += tz_offset_secs;

  (void)gmtime_r(&now, &tm);
  tm_dsec = (tm.tm_hour * 60 + tm.tm_min) * 60 +
            (tm.tm_sec < 60 ? tm.tm_sec : 59);
  tm_dsec %= 24 * 60 * 60;

  eng_dbg("start_secs=%u, end_secs=%u, curr_sec=%u => %sallow charging\n",
          start_secs, end_secs, tm_dsec,
          (start_secs <= tm_dsec && tm_dsec < end_secs) ? "dis" : "");

  while (start_secs > end_secs)
    {
      end_secs += 24 * 60 * 60;
    }

  if (start_secs <= tm_dsec && tm_dsec < end_secs)
    {
      /* Within disallowed range. */

      allowed = false;

      /* Seconds to next state change. */

      seconds_to_change = end_secs - tm_dsec + 1;
    }
  else
    {
      /* Within allowed range. */

      allowed = true;

      /* Seconds to next state change. */

      while (start_secs < tm_dsec)
        {
          start_secs += 24 * 60 * 60;
        }

      seconds_to_change = start_secs - tm_dsec + 1;
    }

  eng_dbg("next check in %u seconds.\n", seconds_to_change);

register_timer:
  /* Register timer to handle next allow/disallow state change. */

  if (g_priv.charger_timer_id >= 0)
    {
      ts_core_timer_stop(g_priv.charger_timer_id);
      g_priv.charger_timer_id = -1;
    }

  g_priv.charger_timer_id = ts_core_timer_setup(TS_TIMER_TYPE_TIMEOUT,
                                                1000ULL * seconds_to_change,
                                                charger_allowed_timer_cb,
                                                NULL);
  DEBUGASSERT(g_priv.charger_timer_id >= 0);

  return allowed;
}
