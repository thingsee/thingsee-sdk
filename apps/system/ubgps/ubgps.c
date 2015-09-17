/****************************************************************************
 * apps/system/ubgps/ubgps.c
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
 *   Author: Sami Pelkonen <sami.pelkonen@haltian.com>
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

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <string.h>
#include <poll.h>
#include <debug.h>
#include <queue.h>

#include <arch/board/board-gps.h>
#include <apps/thingsee/modules/ts_gps.h>

#include "ubgps_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SYSTEM_UBGPS_DEBUG
 #define dbg_ubgps(...) dbg(__VA_ARGS__)
#else
  #define dbg_ubgps(...)
#endif

#define NMEA_BUFFER_LINE_LEN          128

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* GPS internal data structure */

static struct ubgps_s g_gps;

/* A-GPS hint data structure */

static struct gps_assist_hint_s g_gps_hint;

/* GPS state machine names */

static const char * gps_sm_name[__GPS_STATE_MAX] =
{
  [GPS_STATE_POWER_OFF]       = "GPS_STATE_POWER_OFF",
  [GPS_STATE_INITIALIZATION]  = "GPS_STATE_INITIALIZATION",
  [GPS_STATE_COLD_START]      = "GPS_STATE_COLD_START",
  [GPS_STATE_SEARCHING_FIX]   = "GPS_STATE_SEARCHING_FIX",
  [GPS_STATE_FIX_ACQUIRED]    = "GPS_STATE_FIX_ACQUIRED",
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ubgps_initialize
 *
 * Description:
 *   Initialize u-blox 7 GPS module daemon
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
struct ubgps_s *ubgps_initialize(void)
{
  struct ubgps_s * const gps = &g_gps;
  struct sm_event_s event;
  int ret;

  dbg_ubgps("\n");

  /* Check if GPS already powered */

  if (gps->fd > 0)
    {
      return gps;
    }

  /* Clear data */

  memset(gps, 0, sizeof(struct ubgps_s));
  gps->hint = &g_gps_hint;

  /* Open GPS serial device */

  gps->fd = board_gps_initialize();
  if (gps->fd < 0)
    {
      dbg("Failed to open GPS device: %d\n", errno);
      return NULL;
    }

  /* Set file blocking */

  ret = ubgps_set_nonblocking(gps, false);
  if (ret < 0)
    return NULL;

  /* Initialize aiding mutex */

  pthread_mutex_init(&g_aid_mutex, NULL);

  /* Initialize GPS state */

  gps->state.current_state = GPS_STATE_POWER_OFF;
  gps->state.new_state = gps->state.current_state;
  gps->state.target_state = gps->state.current_state;
  gps->state.nmea_protocol_enabled = false;
  gps->state.navigation_rate = DEFAULT_NAVIGATION_RATE;
  gps->state.target_state_pending = false;
  gps->state.target_timer_id = -1;

  /* Initialize UBX receiver */

  ret = ubx_initialize(gps, ubx_callback, gps);
  if (ret < 0)
    {
      dbg("ubx_initialize returned %d\n", ret);
      goto errout_close;
    }

  /* Setup NMEA line buffer */

  gps->nmea.line_size = NMEA_BUFFER_LINE_LEN;
  gps->nmea.current_len = 0;

  /* Allocate memory for NMEA line buffer */

  gps->nmea.line = malloc(gps->nmea.line_size);
  if (!gps->nmea.line)
    {
      dbg("NMEA line alloc failed\n");
      goto errout_ubx;
    }

  /* Initialize GPS callback queue */

  sq_init(&gps->callbacks);

#ifdef CONFIG_UBGPS_ASSIST_UPDATER
  /* Initialize A-GPS if A-GPS updater enabled. */

  ubgps_set_aiding_params(false, "", 1, false, 0, 0, 0, 0);
#endif

  /* Construct and provess state machine entry event */

  event.id = SM_EVENT_ENTRY;
  (void)ubgps_sm_process(gps, &event);

  return gps;

errout_ubx:

  /* Deinitialize UBX receiver */

  ubx_deinitialize(gps);

errout_close:

  /* Close GPS device */

  close(gps->fd);
  gps->fd = -1;

  pthread_mutex_destroy(&g_aid_mutex);

  return NULL;
}

/****************************************************************************
 * Name: ubgps_callback_register
 *
 * Description:
 *   Register callback function for GPS events and data retrieval
 *
 * Input Parameters:
 *   event_mask  - Mask of subscribed events generated with GPS_EVENT macro
 *   callback    - Callback function
 *   priv        - Pointer to private data
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ubgps_callback_register(uint32_t const event_mask, gps_callback_t callback, void * const priv)
{
  struct ubgps_s * const gps = &g_gps;
  struct gps_callback_entry_s * cb;
  uint32_t mask = 0;
  struct gps_callback_entry_s * cb_reuse = NULL;

  dbg_ubgps("mask:%04X, cb:%p\n", event_mask, callback);

  /* Check input data */

  if (!callback)
    return ERROR;

  /* Check if callback is already registered */

  cb = (struct gps_callback_entry_s *)sq_peek(&gps->callbacks);
  while (cb)
    {
      if (cb->callback == callback)
        {
          if (cb->event_mask != 0)
            {
              return ERROR;
            }

          /* Reuse unactive callback */

          cb_reuse = cb;
        }
      else
        {
          /* Add to callback events to mask */

          mask |= cb->event_mask;
        }

      /* Move to next callback */

      cb = (struct gps_callback_entry_s *)sq_next(&cb->entry);
    }

  /* Update event mask for all callbacks */

  gps->callback_event_mask = mask | event_mask;

  /* Setup new callback */

  if (cb_reuse)
    {
      sq_rem(&cb_reuse->entry, &gps->callbacks);
      cb = cb_reuse;
    }
  else
    {
      cb = malloc(sizeof(struct gps_callback_entry_s));
      if (cb == NULL)
        {
          return ERROR;
        }
    }

  cb->event_mask = event_mask;
  cb->callback = callback;
  cb->priv = priv;

  /* Add callback to queue */

  sq_addlast(&cb->entry, &gps->callbacks);

  return OK;
}

/****************************************************************************
 * Name: ubgps_callback_unregister
 *
 * Description:
 *   Unregister callback from GPS module
 *
 * Input Parameters:
 *   callback    - Callback function
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ubgps_callback_unregister(gps_callback_t callback)
{
  struct ubgps_s * const gps = &g_gps;
  struct gps_callback_entry_s * cb, * cbnext;
  uint32_t mask = 0;
  int status = ERROR;

  dbg_ubgps("%p\n", callback);

  /* Check input data */

  if (!callback)
    return ERROR;

  /* Search for callback in queue */

  cb = (struct gps_callback_entry_s *)sq_peek(&gps->callbacks);
  while (cb)
    {
      /* Save next callback entry */
      cbnext = (struct gps_callback_entry_s *)sq_next(&cb->entry);

      if (cb->callback == callback)
        {
          /* Mark callback as non-active. We cannot free callback yet, as
           * unregister might have been called from callback itself.
           */

          cb->event_mask = 0;

          /* Mark successful operation */

          status = OK;
        }
      else
        {
          /* Add to callback events to mask */

          mask |= cb->event_mask;
        }

      /* Move to next callback */

      cb = cbnext;
    }

  /* Update event mask for all callbacks */

  gps->callback_event_mask = mask;

  return status;
}

/****************************************************************************
 * Name: ubgps_get_state
 *
 * Description:
 *   Get current GPS state
 *
 * Returned Values:
 *   GPS state
 *
 ****************************************************************************/
gps_state_t ubgps_get_state(void)
{
  struct ubgps_s const * const gps = &g_gps;

  return gps->state.current_state;
}

/****************************************************************************
 * Name: ubgps_get_location
 *
 * Description:
 *   Get current GPS location
 *
 * Returned Values:
 *   Pointer to GPS location structure on success
 *   NULL in case of error
 *
 ****************************************************************************/
struct gps_location_s const * const ubgps_get_location(void)
{
  struct ubgps_s const * const gps = &g_gps;

  if (gps->state.current_state <= GPS_STATE_INITIALIZATION)
    return NULL;

  return &gps->location;
}

/****************************************************************************
 * Name: ubgps_config
 *
 * Description:
 *   Set GPS configuration parameters
 *
 * Input Parameters:
 *   config      - Configuration item
 *   value       - Pointer to item value
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ubgps_config(gps_config_t const config, void const * const value)
{
  struct ubgps_s * const gps = &g_gps;

  dbg_ubgps("%u\n", config);

  if (!value)
    return ERROR;

  switch (config)
    {
      case GPS_CONFIG_NMEA_DATA:
        {
          gps->state.nmea_protocol_enabled = !!*((bool *)value);

          break;
        }

      case GPS_CONFIG_NAVIGATION_RATE:
        {
          int status;

          gps->state.navigation_rate = *((uint32_t *)value);

          /* Update navigation rate immediately if initialization phase
             is already completed. */

          if (gps->state.current_state == GPS_STATE_SEARCHING_FIX ||
              gps->state.current_state == GPS_STATE_FIX_ACQUIRED)
            {
#ifdef CONFIG_UBGPS_PSM_MODE
              if (gps->state.navigation_rate >= CONFIG_UBGPS_PSM_MODE_THRESHOLD)
                {
                  /* Use default navigation rate for SW controlled PSM */

                  status = ubgps_send_cfg_rate(gps, DEFAULT_NAVIGATION_RATE);
                }
              else
                {
                  status = ubgps_send_cfg_rate(gps, gps->state.navigation_rate);
                }
#else
              status = ubgps_send_cfg_rate(gps, gps->state.navigation_rate);
#endif /* CONFIG_UBGPS_PSM_MODE */

              if (status != OK)
                {
                  dbg("navigation rate update failed\n");
                  return ERROR;
                }
            }

          break;
        }

      default:
        return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: ubgps_request_state
 *
 * Description:
 *   Request GPS module state
 *
 * Input Parameters:
 *   state       - Target state
 *   timeout_ms  - Timeout in seconds to reach target state
 *                 (0 value means no timeout)
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ubgps_request_state(gps_state_t const state, uint32_t const timeout)
{
  struct ubgps_s * const gps = &g_gps;
  struct sm_event_target_state_s event;

  dbg_ubgps("%u, timeout: %u\n", state, timeout);

  if (state < 0 || state > __GPS_STATE_MAX)
    return ERROR;

  /* Mark target state transition as pending */

  gps->state.target_state_pending = true;

  /* Construct target state event */

  event.super.id = SM_EVENT_TARGET_STATE;
  event.target_state = state;
  event.timeout = timeout;

  /* Process event */

  return ubgps_sm_process(gps, (struct sm_event_s *)&event);
}

/****************************************************************************
 * Name: ubgps_get_statename
 *
 * Description:
 *   Get GPS state name
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
char const * const ubgps_get_statename(gps_state_t const gps_state)
{
  if (gps_state < 0 || gps_state >= __GPS_STATE_MAX)
    return NULL;

  return gps_sm_name[gps_state];
}

/*****************************************************************************
 * Name: ubgps_deepsleep_callback
 *
 * Description:
 *   Deep-sleep hook for ubgps
 *****************************************************************************/

bool ubgps_deepsleep_callback(void * const priv)
{
  struct ubgps_s * const gps = &g_gps;

  /* Allow deep sleep if GPS state is power_off and state transition is not
     pending */

  return (ubgps_get_state() == GPS_STATE_POWER_OFF &&
         !gps->state.target_state_pending);
}

/****************************************************************************
 * Name: ubgps_set_aiding_params
 *
 * Description:
 *   Set parameters for assisted GPS
 *
 * Input Parameters:
 *   use_time    - Use system time for GPS initialization
 *   alp_file    - AlmanacPlus filename for AssistNow Offline (NULL if not used)
 *   alp_file_id - File ID for AlmanacPlus file
 *   use_loc     - Use location
 *   latitude    - Latitude
 *   longitude   - Longitude
 *   altitude    - Altitude in meters
 *   accuracy    - Horizontal accuracy in meters
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ubgps_set_aiding_params(bool const use_time,
                             char const * const alp_file,
                             uint16_t const alp_file_id,
                             bool const use_loc,
                             double const latitude,
                             double const longitude,
                             int32_t const altitude,
                             uint32_t const accuracy)
{
  struct ubgps_s * const gps = &g_gps;

  dbg_ubgps("\n");

  (void)use_time;

  /* Free previously set assistance data */

  if (gps->assist)
    {
      if (gps->assist->alp_file)
        free(gps->assist->alp_file);

      free(gps->assist);
      gps->assist = NULL;
    }

  gps->assist = zalloc(sizeof(*gps->assist));
  if (!gps->assist)
    return ERROR;

  gps->assist->alp_file = NULL;
  gps->assist->alp_file_id = 0;

#ifdef CONFIG_UBGPS_ASSIST_UPDATER
  /* Ignore input ALP file, use A-GPS updater provided file instead. */

  (void)alp_file;

  if (ubgps_check_alp_file_validity(ubgps_aid_get_alp_filename()))
    {
      /* AlmanacPlus file available */

      gps->assist->alp_file = strdup(ubgps_aid_get_alp_filename());
      gps->assist->alp_file_id = 1;
    }
#else
  if (alp_file)
    {
      /* Check that AlmanacPlus file is present and valid */

      if (ubgps_check_alp_file_validity(alp_file))
        {
          /* AlmanacPlus file available */

          gps->assist->alp_file = strdup(alp_file);
          gps->assist->alp_file_id = alp_file_id;
        }
    }
#endif

  if (use_loc)
    {
      if (!g_gps_hint.have_location)
        {
          /* Use only if GPS module does not internally already have
           * location. */

          (void)ubgps_give_location_hint(latitude, longitude, altitude,
                                         accuracy);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: ubgps_give_location_hint
 *
 * Description:
 *   Give location hint for A-GPS
 *
 * Input Parameters:
 *   latitude    - Latitude
 *   longitude   - Longitude
 *   altitude    - Altitude in meters
 *   accuracy    - Horizontal accuracy in meters
 *
 * Returned Value:
 *   Status
 *
 ****************************************************************************/

int ubgps_give_location_hint(double const latitude,
                             double const longitude,
                             int32_t const altitude,
                             uint32_t const accuracy)
{
  struct timespec currtime = {};

  if (accuracy > HINT_LOCATION_MINIMUM_NEW_ACCURACY)
    {
      errno = EINVAL;
      return ERROR;
    }

  (void)clock_gettime(CLOCK_MONOTONIC, &currtime);

  if (g_gps_hint.have_location)
    {
      uint64_t current_accuracy;
      uint32_t secs_since = currtime.tv_sec - g_gps_hint.location_time.tv_sec;

      /* Adjust location accuracy by time*speed. */

      current_accuracy = g_gps_hint.accuracy;
      current_accuracy += ((uint64_t)HINT_LOCATION_ACCURACY_DEGRADE_SPEED_MPS *
                           secs_since);

      /* Use old location if its accuracy is better than new. Accuracy of
       * old location data will degrade over time and eventually be overridden
       * with new location. */

      if (current_accuracy < accuracy)
        {
          errno = EALREADY;
          return ERROR;
        }
    }


  g_gps_hint.location_time = currtime;
  g_gps_hint.have_location = true;
  g_gps_hint.latitude = latitude * 1e7;
  g_gps_hint.longitude = longitude * 1e7;
  g_gps_hint.altitude = altitude;
  g_gps_hint.accuracy = accuracy;

  dbg_ubgps("New location hint:\n");
  dbg_ubgps(" lat     : %d\n", g_gps_hint.latitude);
  dbg_ubgps(" long    : %d\n", g_gps_hint.longitude);
  dbg_ubgps(" alt     : %d meters\n", altitude);
  dbg_ubgps(" accuracy: %u meters\n", g_gps_hint.accuracy);

  return OK;
}
