/****************************************************************************
 * apps/include/system/ubgps.h
 *
 *   Copyright (C) 2014 Haltian Ltd. All rights reserved.
 *   Authors:
 *     Sami Pelkonen <sami.pelkonen@haltian.com>
 *     Harri Luhtala <harri.luhtala@haltian.com>
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

#ifndef __APPS_INCLUDE_SYSTEM_UBGPS_H
#define __APPS_INCLUDE_SYSTEM_UBGPS_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPS states */

enum gps_state_e
{
  /* GPS module powered off */

  GPS_STATE_POWER_OFF = 0,

  /* GPS module initialization */

  GPS_STATE_INITIALIZATION,

  /* GPS module cold start state */

  GPS_STATE_COLD_START,

  /* GPS module powered on and searching for GPS fix */

  GPS_STATE_SEARCHING_FIX,

  /* GPS module powered on and GPS fix acquired */

  GPS_STATE_FIX_ACQUIRED,

  /* Maximum GPS state */

  __GPS_STATE_MAX,
};
typedef enum gps_state_e gps_state_t;


/* GPS public events */

enum gps_event_id_e
{
  /* State change event (gps_event_state_change_s) */

  GPS_EVENT_STATE_CHANGE =              (1 << 0),

  /* Target state reached event (gps_event_target_state_s) */

  GPS_EVENT_TARGET_STATE_REACHED =      (1 << 1),

  /* Error has occurred before reaching target state (gps_event_target_state_s) */

  GPS_EVENT_TARGET_STATE_NOT_REACHED =  (1 << 2),

  /* Timeout occurred before target state was reached (gps_event_target_state_s) */

  GPS_EVENT_TARGET_STATE_TIMEOUT =      (1 << 3),

  /* NMEA data event (gps_event_nmea_data_s) */

  GPS_EVENT_NMEA_DATA =                 (1 << 4),

  /* GPS time data event (gps_event_time_s) */

  GPS_EVENT_TIME =                      (1 << 5),

  /* GPS location data event (gps_event_location_s) */

  GPS_EVENT_LOCATION =                  (1 << 6),
};
typedef enum gps_event_id_e gps_event_id_t;


/* GPS fix type */

enum gps_fix_e
{
  GPS_FIX_NOT_AVAILABLE = 0,
  GPS_FIX_DEAD_RECOGNING,
  GPS_FIX_2D,
  GPS_FIX_3D,
  GPS_FIX_GNSS_DEAD_RECOGNING,
  GPS_FIX_TIME_ONLY,
  __GPS_FIX_MAX,
};
typedef enum gps_fix_e gps_fix_t;


/* GPS configuration items */

enum gps_config_e
{
  /* Report NMEA protocol data. Data type is bool */

  GPS_CONFIG_NMEA_DATA = 0,

  /* Navigation rate in ms. Data type is uint16 */

  GPS_CONFIG_NAVIGATION_RATE,

  /* Maximum configuration item value */

  __GPS_CONFIG_MAX,
};
typedef enum gps_config_e gps_config_t;

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* GPS time validity */

struct gps_time_validity_s
{
  /* Valid UTC date */

  bool date : 1;

  /* Valid UTC Time of Day */

  bool time : 1;

  /* UTC Time of Day has been fully resolved (no seconds uncertainty) */

  bool fully_resolved : 1;
};


/* GPS time in UTC */

struct gps_time_s
{
  /* Time validity */

  struct gps_time_validity_s validity;

  /* Year */

  int year;

  /* Month */

  int month;

  /* Day */

  int day;

  /* Hour */

  int hour;

  /* Minutes */

  int min;

  /* Seconds */

  int sec;
};


/* GPS location */

struct gps_location_s
{
  /* GPS fix type */

  gps_fix_t fix_type;

  /* Number of satellites used in navigation solution */

  uint8_t num_of_used_satellites;

  /* Longitude in 1e7 scale */

  int32_t longitude;

  /* Latitude in 1e7 scale */

  int32_t latitude;

  /* Horizontal accuracy estimate in mm */

  uint32_t horizontal_accuracy;

  /* Height above mean sea level in mm */

  int32_t height;

  /* Vertical accuracy estimate in mm */

  uint32_t vertical_accuracy;

  /* Ground speed mm/s */

  int32_t ground_speed;

  /* Ground speed accuracy estimate in mm/s */

  uint32_t ground_speed_accuracy;

  /* Heading in degrees / 1e5 scale */

  int32_t heading;

  /* Heading accuracy estimate in degress / 1e5 scale */

  uint32_t heading_accuracy;
};


/* GPS public event base */

struct gps_event_s
{
  /* GPS event ID */

  gps_event_id_t id;
};

/*
 * GPS public events
 */


/* GPS state change event */

struct gps_event_state_change_s
{
  /* Event base class */

  struct gps_event_s super;

  /* GPS state */

  gps_state_t state;
};


/* GPS target state event */

struct gps_event_target_state_s
{
  /* Event base class */

  struct gps_event_s super;

  /* Target state */

  gps_state_t target_state;

  /* Current state */

  gps_state_t current_state;
};


/* GPS NMEA event */

struct gps_event_nmea_data_s
{
  /* Event base class */

  struct gps_event_s super;

  /* NULL terminated NMEA line data */

  char * line;
};


/* GPS time event */

struct gps_event_time_s
{
  /* Event base class */

  struct gps_event_s super;

  /* Time */

  struct gps_time_s * time;
};


/* GPS location event */

struct gps_event_location_s
{
  /* Event base class */

  struct gps_event_s super;

  /* Time */

  struct gps_time_s * time;

  /* Location */

  struct gps_location_s * location;
};

/****************************************************************************
 * Public Function Prototypes
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
struct ubgps_s *ubgps_initialize(void);

/****************************************************************************
 * Name: ts_gps_callback_t
 *
 * Description:
 *   GPS callback function prototype
 *
 * Input Parameters:
 *   e           - Pointer to event
 *   priv        - Pointer to private data
 *
 ****************************************************************************/
typedef void (*gps_callback_t)(void const * const e,
                               void * const priv);

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
int ubgps_callback_register(uint32_t const event_mask, gps_callback_t callback, void * const priv);

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
int ubgps_callback_unregister(gps_callback_t callback);

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
gps_state_t ubgps_get_state(void);

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
struct gps_location_s const * const ubgps_get_location(void);

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
int ubgps_config(gps_config_t const config, void const * const value);

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
int ubgps_request_state(gps_state_t const state, uint32_t const timeout_ms);

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
char const * const ubgps_get_statename(gps_state_t const gps_state);

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
int ubgps_set_aiding_params(bool use_time,
                             char const * const alp_file,
                             uint16_t alp_file_id,
                             bool use_loc,
                             double latitude,
                             double longitude,
                             int32_t altitude,
                             uint32_t accuracy);

/****************************************************************************
 * Name: ubgps_setup_poll
 *
 * Description:
 *   Setup pollfd structure and timeout value for ubgps library
 *
 * Input Parameters:
 *   None.
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/

int ubgps_poll_setup(struct ubgps_s *gps, struct pollfd *pfd,
                       int *timeout);

/****************************************************************************
 * Name: ubgps_poll_event
 *
 * Description:
 *   Indicate poll event for ubgps library. Library will handle current
 *   pending I/O and internal state changes.
 *
 * Input Parameters:
 *   pfd: Poll structure for ubgps, setup with ubgps_poll_setup
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/

int ubgps_poll_event(struct ubgps_s *gps, struct pollfd *pfd);

/****************************************************************************
 * Name: ubgps_poll_timedout
 *
 * Description:
 *   Indicate that poll timed out. This allows library to handle internal
 *   timed state changes.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/

int ubgps_poll_timedout(struct ubgps_s *gps);

/****************************************************************************
 * Name: ubgps_deepsleep_callback
 *
 * Description:
 *   Deep-sleep hook for ubgps
 *
 ****************************************************************************/

bool ubgps_deepsleep_callback(void * const priv);

#endif /* __APPS_INCLUDE_SYSTEM_UBGPS_H */
