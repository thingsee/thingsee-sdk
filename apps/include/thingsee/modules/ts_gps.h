/****************************************************************************
 * apps/include/thingsee/modules/ts_gps.h
 *
 *   Copyright (C) 2014 Haltian Ltd. All rights reserved.
 *   Authors: Sami Pelkonen <sami.pelkonen@haltian.com>
 *            Harri Luhtala <harri.luhtala@haltian.com>
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

#ifndef __APPS_INCLUDE_THINGSEE_TS_GPS_H
#define __APPS_INCLUDE_THINGSEE_TS_GPS_H

#include <apps/system/ubgps.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern struct ubgps_s *ts_gps;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

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
 * Name: ts_gps_initialize
 *
 * Description:
 *   Initialize thingsee GPS module daemon
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ts_gps_initialize(void);

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
int ts_gps_callback_register(uint32_t const event_mask, gps_callback_t callback, void * const priv);

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
int ts_gps_callback_unregister(gps_callback_t callback);

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
gps_state_t ts_gps_get_state(void);

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
int ts_gps_reset_odometer(void);

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
struct gps_location_s const * const ts_gps_get_location(void);

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
int ts_gps_config(gps_config_t const config, void const * const value);

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
int ts_gps_request_state(gps_state_t const state, uint32_t const timeout);

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
char const * const ts_gps_get_statename(gps_state_t const gps_state);

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
                             uint32_t const accuracy);

#endif /* __APPS_INCLUDE_THINGSEE_TS_GPS_H */
