/****************************************************************************
 * apps/include/thingsee/modules/ts_bluetooth.h
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
 *   Author: Harri Luhtala <harri.luhtala@haltian.com>
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

#ifndef __APPS_INCLUDE_THINGSEE_TS_BLUETOOTH_H
#define __APPS_INCLUDE_THINGSEE_TS_BLUETOOTH_H

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern struct btle_s *ts_bluetooth;

enum bluetooth_state_e
{
  /* Waiting to be started */
  BTLE_STATE_INIT = 0,

  /* Started but not advertising */
  BTLE_STATE_STARTED,

  /* Currently Advertising */
  BTLE_STATE_ADVERTISING,

  /* Currently using non-connectable Advertising */
  BTLE_STATE_ADVERTISING_NONCONN,

  /* Device is started but not advertising,
     is in waiting period before advertising again */
  BTLE_STATE_WAITING,

  /* Device just timed out from a connection but is not yet advertising,
     is in waiting period before advertising again */
  BTLE_STATE_WAITING_AFTER_TIMEOUT,

  /* In a connection */
  BTLE_STATE_CONNECTED,

  /* In a connection + advertising */
  BTLE_STATE_CONNECTED_ADV,

  /* Error occurred - invalid state */
  BTLE_STATE_ERROR,

  __BTLE_STATE_MAX
};
typedef enum bluetooth_state_e bluetooth_state_t;

/* Bluetooth public events */

enum bluetooth_event_id_e
{
  /* BT connection state changed  */

  BT_EVENT_CONNECTION_STATE = 0x01,

  /* Device profile received */

  BT_EVENT_DEVICE_PROFILE =   0x02,

  /* Cloud profile received */

  BT_EVENT_CLOUD_PROFILE =    0x04,

  /* Time event received */

  BT_EVENT_TIME =             0x08,
};
typedef enum bluetooth_event_id_e bluetooth_event_id_t;


struct bluetooth_time_s
{
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

/* Public event base */

struct bluetooth_event_s
{
  /* BTLE event ID */

  bluetooth_event_id_t id;
};

/* Connection state event */

struct bluetooth_event_conn_state_s
{
  /* Event */

  struct bluetooth_event_s event;

  /* Current state */

  bluetooth_state_t state;
};

/* Profile event */

struct bluetooth_event_profile_s
{
  /* Event */

  struct bluetooth_event_s event;

  /* profile size in bytes */

  uint16_t size;

  /* Pointer for profile data */

  void* profile;
};

/* Time event */

struct bluetooth_event_time_s
{
  /* Event */

  struct bluetooth_event_s event;

  /* profile size in bytes */

  struct bluetooth_time_s;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ts_bluetooth_callback_t
 *
 * Description:
 *   BT callback function prototype
 *
 * Input Parameters:
 *   e           - Pointer to event
 *   priv        - Pointer to private data
 *
 ****************************************************************************/
typedef void (*bluetooth_callback_t)(void const * const e,
                                     void * const priv);

/****************************************************************************
 * Name: ts_bluetooth_initialize
 *
 * Description:
 *   Initialize thingsee BT module
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ts_bluetooth_initialize(void);

/****************************************************************************
 * Name: ts_bluetooth_uninitialize
 *
 * Description:
 *   Uninitialize thingsee BT module
 *
 * Returned Values:
 *
 ****************************************************************************/
void ts_bluetooth_uninitialize(void);

/****************************************************************************
 * Name: ts_bluetooth_callback_register
 *
 * Description:
 *   Register callback function for BT events and data retrieval
 *
 * Input Parameters:
 *   event_mask  - Mask of subscribed events generated with BT_EVENT macro
 *   callback    - Callback function
 *   priv        - Pointer to private data
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ts_bluetooth_callback_register(uint32_t const event_mask,
                                   bluetooth_callback_t callback,
                                   void * const priv);

/****************************************************************************
 * Name: ts_bluetooth_callback_unregister
 *
 * Description:
 *   Unregister callback from BT module
 *
 * Input Parameters:
 *   callback    - Callback function
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ts_bluetooth_callback_unregister(bluetooth_callback_t callback);

/****************************************************************************
 * Name: ts_bluetooth_get_state
 *
 * Description:
 *   Get current BT state
 *
 * Returned Values:
 *   BT state
 *
 ****************************************************************************/
int ts_bluetooth_get_state(void);

/****************************************************************************
 * Name: ts_bluetooth_get_statename
 *
 * Description:
 *   Get BT state name
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
char const * const ts_bluetooth_get_statename(bluetooth_state_t const bt_state);

#endif /* __APPS_INCLUDE_THINGSEE_TS_BLUETOOTH_H */
