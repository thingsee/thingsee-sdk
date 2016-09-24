/****************************************************************************
 * apps/thingsee/gps/ts_gps.c
 *
 * Authors:
 *   Sami Pelkonen <sami.pelkonen@haltian.com>
 *   Harri Luhtala <harri.luhtala@haltian.com>
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
#include <debug.h>
#include <queue.h>

#include <arch/board/board-gps.h>
#include <apps/thingsee/ts_core.h>
#include <apps/thingsee/modules/ts_gps.h>
#include <apps/system/ubgps.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ts_gps_timer_callback(const int timer_id, void * const priv);
static int ts_gps_poll_callback(const struct pollfd * const inpfd,
                                void * const priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct ubgps_s *ts_gps;

struct {
  int fd;
  int timerid;
} ts_core_gps = {
  .fd = -1,
  .timerid = -1,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/*****************************************************************************
 * Name: ts_gps_core_reconfigure
 *
 * Description:
 *   Reconfigure ubgps poll/timer events.
 *****************************************************************************/

static int ts_gps_core_reconfigure(void)
{
  struct pollfd pfd = {};
  int timeout_ms;
  int ret;

  /* Get new poll setup. */

  ret = ubgps_poll_setup(ts_gps, &pfd, &timeout_ms);
  if (ret < 0)
    {
      return ret;
    }

  /* Reconfigure poll events. */

  if (pfd.fd >= 0 && ts_core_gps.fd == pfd.fd)
    {
      ret = ts_core_fd_set_poll_events(pfd.fd, pfd.events);
      DEBUGASSERT(ret >= 0);
    }
  else
    {
      if (ts_core_gps.fd >= 0)
        ts_core_fd_unregister(ts_core_gps.fd);
      if (pfd.fd >= 0)
        {
          ret = ts_core_fd_register(pfd.fd, pfd.events, ts_gps_poll_callback,
                                    NULL);
          DEBUGASSERT(ret >= 0);
        }

      ts_core_gps.fd = pfd.fd;
    }

  /* Reconfigure timers. */

  if (ts_core_gps.timerid >= 0)
    ts_core_timer_stop(ts_core_gps.timerid);
  if (timeout_ms >= 0)
    {
      ret = ts_core_timer_setup(TS_TIMER_TYPE_TIMEOUT, timeout_ms,
                                ts_gps_timer_callback, NULL);
      DEBUGASSERT(ret >= 0);
      ts_core_gps.timerid = ret;
    }
  else
    {
      ts_core_gps.timerid = -1;
    }

  return OK;
}

/*****************************************************************************
 * Name: ts_gps_timer_callback
 *
 * Description:
 *   Timer event handling for GPS.
 *****************************************************************************/

static int ts_gps_timer_callback(const int timer_id, void * const priv)
{
  if (ts_core_gps.timerid < 0)
    return OK;

  DEBUGASSERT(ts_core_gps.timerid == timer_id);

  /* This is one-shot timer and have been uninitialized by core. */

  ts_core_gps.timerid = -1;

  /* Process timer. */

  (void)ubgps_poll_timedout(ts_gps);

  /* Reconfigure poll/timer events. */

  return ts_gps_core_reconfigure();
}

/*****************************************************************************
 * Name: ts_gps_poll_callback
 *
 * Description:
 *   Poll event handling for GPS.
 *****************************************************************************/

static int ts_gps_poll_callback(const struct pollfd * const inpfd,
                                void * const priv)
{
  struct pollfd pfd;

  if (ts_core_gps.fd < 0)
    return OK;

  DEBUGASSERT(ts_core_gps.fd == inpfd->fd);

  /* Process event. */

  pfd = *inpfd;
  (void)ubgps_poll_event(ts_gps, &pfd);

  /* Reconfigure poll/timer events. */

  return ts_gps_core_reconfigure();
}

/*****************************************************************************
 * Name: ts_gps_new_timer_callback
 *
 * Description:
 *   New timer event handling for GPS. Called from ubgps when new timer is
 *   registered and upper level needs to reconfigure poll timeout.
 *****************************************************************************/

static void ts_gps_new_timer_callback(void const * const e, void * const priv)
{
  int ret;

  ret = ts_gps_core_reconfigure();
  if (ret < 0)
    {
      dbg("ts_gps_core_reconfigure failed\n");
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ts_gps_initialize
 *
 * Description:
 *   Initialization for GPS module
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ts_gps_initialize(void)
{
  int ret;

  if (ts_gps)
    {
      /* already initialized */

      return OK;
    }

  ts_gps = ubgps_initialize();
  if (!ts_gps)
    {
      dbg("init failed\n");
      return ERROR;
    }

  ret = ubgps_callback_register(GPS_EVENT_NEW_TIMER,
                                ts_gps_new_timer_callback, NULL);
  if (ret < 0)
    {
      dbg("ubgps_callback_register failed\n");
      return ret;
    }

  ret = ts_gps_core_reconfigure();
  if (ret < 0)
    {
      dbg("ts_gps_core_reconfigure failed\n");
      return ret;
    }

  return ts_core_deepsleep_hook_add(ubgps_deepsleep_callback, NULL);
}

/****************************************************************************
 * Name: ts_gps_callback_register
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
int ts_gps_callback_register(uint32_t const event_mask, gps_callback_t callback, void * const priv)
{
  return ubgps_callback_register(event_mask, callback, priv);
}

/****************************************************************************
 * Name: ts_gps_callback_unregister
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
int ts_gps_callback_unregister(gps_callback_t callback)
{
  return ubgps_callback_unregister(callback);
}

/****************************************************************************
 * Name: ts_gps_get_state
 *
 * Description:
 *   Get current GPS state
 *
 * Returned Values:
 *   GPS state
 *
 ****************************************************************************/
gps_state_t ts_gps_get_state(void)
{
  return ubgps_get_state();
}

/****************************************************************************
 * Name: ts_gps_reset_odometer
 *
 * Description:
 *   Resets the traveled distance computed by the odometer
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
#ifdef CONFIG_UBGPS_ODOMETER_EVENT
int ts_gps_reset_odometer(void)
{
  return ubgps_reset_odometer();
}
#endif

/****************************************************************************
 * Name: ts_gps_get_location
 *
 * Description:
 *   Get current GPS location
 *
 * Returned Values:
 *   Pointer to GPS location structure on success
 *   NULL in case of error
 *
 ****************************************************************************/
struct gps_location_s const * const ts_gps_get_location(void)
{
  return ubgps_get_location();
}

/****************************************************************************
 * Name: ts_gps_config
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
int ts_gps_config(gps_config_t const config, void const * const value)
{
  return ubgps_config(config, value);
}

/****************************************************************************
 * Name: ts_gps_request_state
 *
 * Description:
 *   Request GPS module state
 *
 * Input Parameters:
 *   state       - Target state
 *   timeout     - Timeout in seconds to reach target state
 *                 (0 value means no timeout)
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ts_gps_request_state(gps_state_t const state, uint32_t const timeout)
{
  return ubgps_request_state(state, timeout);
}

/****************************************************************************
 * Name: ts_gps_get_statename
 *
 * Description:
 *   Get GPS state name
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
char const * const ts_gps_get_statename(gps_state_t const gps_state)
{
  return ubgps_get_statename(gps_state);
}

/****************************************************************************
 * Name: ts_gps_set_aiding_params
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
int ts_gps_set_aiding_params(bool const use_time,
                             char const * const alp_file,
                             uint16_t const alp_file_id,
                             bool const use_loc,
                             double const latitude,
                             double const longitude,
                             int32_t const altitude,
                             uint32_t const accuracy)
{
  return ubgps_set_aiding_params(use_time, alp_file, alp_file_id,
                                 use_loc, latitude, longitude,
                                 altitude, accuracy);
}
