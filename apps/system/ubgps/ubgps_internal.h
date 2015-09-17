/****************************************************************************
 * apps/system/ubgps/ubgps_internal.h
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

#ifndef __THINGSEE_GPS_INTERNAL_H
#define __THINGSEE_GPS_INTERNAL_H

#include <queue.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <pthread.h>

#include "ubgps_events.h"
#include "ubx.h"

/****************************************************************************
 * Public Defines
 ****************************************************************************/

/* Default navigation rate [ms] */

#define DEFAULT_NAVIGATION_RATE       1000

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Mutex to protect access for aiding data */

extern pthread_mutex_t g_aid_mutex;

/* Timer callback function type. */

typedef int (* ubgps_timer_fn_t)(int timer_id, void * const arg);

/* GPS module state */

struct gps_module_state_s
{
  /* GPS powered */

  bool powered;

  /* NMEA protocol enabled */

  bool nmea_protocol_enabled;

  /* Navigation rate [ms] for GPS */

  uint32_t navigation_rate;

  /* Current GPS state */

  gps_state_t current_state;

  /* New GPS state */

  gps_state_t new_state;

  /* Target GPS state */

  gps_state_t target_state;

  /* Target state transition pending */

  int target_state_pending;

  /* Target state timer id */

  int target_timer_id;

  /* Initialization timer ID */

  int init_timer_id;

  /* Aiding data timer ID */

  int aid_timer_id;

  /* Power save mode timer ID */

  int psm_timer_id;

  /* Location filter timer id */

  int location_timer_id;

  /* Initialization phase */

  int init_phase;

  /* Initialization retry counter */

  int init_count;

  /* UBX receiver */

  struct ubx_receiver_s ubx_receiver;
};


/* NMEA line storage for logging */

struct nmea_data_s
{
  /* Line buffer for NMEA data */

  char * line;

  /* Size of NMEA data line buffer */

  size_t line_size;

  /* Current data len in line buffer */

  size_t current_len;
};


/* GPS assistance data structure */

struct gps_assistance_s {
  /* Use system time */

  bool use_time;

  /* Allocated AlmanacPlus filename for AssistNow Offline */

  char * alp_file;

  /* File ID for AlmanacPlus file */

  uint16_t alp_file_id;

  /* Use location */

  bool use_loc;

  /* Latitude */

  uint32_t latitude;

  /* Longitude */

  uint32_t longitude;

  /* Altitude in centimeters */

  int32_t altitude;

  /* Accuracy in centimeters */

  uint32_t accuracy;

  /* Aiding server address */

  struct sockaddr_in alp_srv_addr;

  /* Update time of current aiding data in seconds */

  time_t update_time;
};

/* GPS module internal structure */

struct ubgps_s {
  /* GPS serial device file descriptor */

  int fd;

  /* GPS serial device is non-blocking */

  bool is_nonblocking;

  /* Timers */

  sq_queue_t timers;
  uint16_t timer_id_cnt;

  /* GPS module state */

  struct gps_module_state_s state;

  /* Mask of subscribed events for callbacks */

  uint32_t callback_event_mask;

  /* Registered GPS callbacks */

  sq_queue_t callbacks;

  /* Current GPS time */

  struct gps_time_s time;

  /* Current GPS location */

  struct gps_location_s location;

  /* Filtered GPS location */

  struct gps_location_s filt_location;

  /* NMEA line buffer */

  struct nmea_data_s nmea;

  /* GPS assistance data */

  struct gps_assistance_s * assist;
};


/* GPS callback queue entry */

struct gps_callback_entry_s
{
  /* Queue entry */

  sq_entry_t entry;

  /* Subscribed events */

  uint32_t event_mask;

  /* GPS callback function */

  gps_callback_t callback;

  /* Private data */

  void * priv;
};


/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ubgps_sm_process
 *
 * Description:
 *   Inject event to GPS state machine
 *
 * Input Parameters:
 *   gps         - GPS object
 *   event       - Pointer to processed event
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ubgps_sm_process(struct ubgps_s * const gps,
                     struct sm_event_s const * const event);

/****************************************************************************
 * Name: ubx_callback
 *
 * Description:
 *   Listen to UBX callbacks
 *
 * Input Parameters:
 *   receiver    - UBX object
 *   priv        - Private data
 *   msg         - UBX message
 *   timeout     - Timeout status
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ubx_callback(struct ubx_receiver_s const * const receiver,
                        void * priv,
                        struct ubx_msg_s const * const msg,
                        bool const timeout);

/****************************************************************************
 * Name: nmea_receiver
 *
 * Description:
 *   NMEA data handler
 *
 * Input Parameters:
 *   gps         - GPS object
 *   data        - NMEA data (one character)
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int nmea_receiver(struct ubgps_s * const gps, uint8_t const data);

/****************************************************************************
 * Name: gps_receiver
 *
 * Description:
 *   Handle input from GPS module
 *
 * Input Parameters:
 *   pfd         - Poll structure
 *   priv        - Private data
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ubgps_receiver(const struct pollfd * const pfd, void * const priv);

/****************************************************************************
 * Name: ubgps_publish_event
 *
 * Description:
 *   Publish event to registered callbacks
 *
 * Input Parameters:
 *   gps         - GPS object
 *   event       - Pointer to published event
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
void ubgps_publish_event(struct ubgps_s * const gps,
                         struct gps_event_s const * const event);

/****************************************************************************
 * Name: ubgps_timeout
 *
 * Description:
 *   GPS timeout handling
 *
 * Input Parameters:
 *   timer_id    - Timer ID
 *   priv        - Private data
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ubgps_timeout(const int timer_id, void * const priv);

/****************************************************************************
 * Name: ubgps_set_new_state
 *
 * Description:
 *   Update GPS state machine state and invoke callbacks
 *
 * Input Parameters:
 *   gps         - GPS object
 *   new_state   - New state
 *
 ****************************************************************************/
void ubgps_set_new_state(struct ubgps_s * const gps, gps_state_t new_state);

/****************************************************************************
 * Name: ubgps_report_target_state
 *
 * Description:
 *   Report target state reached status
 *
 * Input Parameters:
 *   gps         - GPS object
 *   timeout     - Timeout status
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
void ubgps_report_target_state(struct ubgps_s * const gps, bool const timeout);

/****************************************************************************
 * Name: ubgps_setup_timeout
 *
 * Description:
 *   Setup target state transition timeout
 *
 * Input Parameters:
 *   gps         - GPS object
 *   timeout     - Timeout in seconds
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ubgps_setup_timeout(struct ubgps_s * const gps, uint32_t const timeout);

/****************************************************************************
 * Name: ubgps_send_cfg_prt
 *
 * Description:
 *   Send UBX CFG-PRT message (Port configuration)
 *
 * Input Parameters:
 *   gps         - GPS object
 *   baud        - Baud rate
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ubgps_send_cfg_prt(struct ubgps_s * const gps, uint32_t const baud);

/****************************************************************************
 * Name: ubgps_send_cfg_ant
 *
 * Description:
 *   Send UBX CFG-ANT message (Antenna configuration)
 *
 * Input Parameters:
 *   gps         - GPS object
 *   flags       - Control flags
 *   pins        - Pins to be controlled
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ubgps_send_cfg_ant(struct ubgps_s * const gps, uint16_t const flags,
                       uint16_t const pins);

/****************************************************************************
 * Name: ubgps_send_cfg_msg
 *
 * Description:
 *   Send UBX CFG-MSG message (Set message rate)
 *
 * Input Parameters:
 *   gps         - GPS object
 *   class_id    - UBX message class id
 *   msg_id      - UBX message id
 *   rate        - Message rate (see UBX CFG-MSG)
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ubgps_send_cfg_msg(struct ubgps_s * const gps, uint8_t class_id,
                       uint8_t msg_id, uint8_t rate);

/****************************************************************************
 * Name: ubgps_send_cfg_rate
 *
 * Description:
 *   Send UBX CFG-RATE message (Set navigation rate)
 *
 * Input Parameters:
 *   gps         - GPS object
 *   rate_ms     - Message rate (see UBX CFG-RATE)
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ubgps_send_cfg_rate(struct ubgps_s * const gps, uint16_t rate_ms);

/****************************************************************************
 * Name: ubgps_send_cfg_rst
 *
 * Description:
 *   Send UBX CFG-RST message (Reset Receiver)
 *
 * Input Parameters:
 *   gps         - GPS object
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ubgps_send_cfg_rst(struct ubgps_s * const gps);

/****************************************************************************
 * Name: ubgps_send_cfg_pm2
 *
 * Description:
 *   Send UBX CFG-PM2 message (Set power management config)
 *
 * Input Parameters:
 *   gps         - GPS object
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ubgps_send_cfg_pm2(struct ubgps_s * const gps, uint32_t flags,
                       uint32_t update_period, uint32_t search_period);

/****************************************************************************
 * Name: ubgps_send_cfg_sbas
 *
 * Description:
 *   Send UBX CFG-SBAS message (Set SBAS config)
 *
 * Input Parameters:
 *   gps         - GPS object
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ubgps_send_cfg_sbas(struct ubgps_s * const gps, bool enable);

/****************************************************************************
 * Name: ubgps_send_cfg_rxm
 *
 * Description:
 *   Send UBX CFG-RXM message (Set RXM config)
 *
 * Input Parameters:
 *   gps         - GPS object
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ubgps_send_cfg_rxm(struct ubgps_s * const gps, uint8_t mode);

/****************************************************************************
 * Name: ubgps_send_aid_ini
 *
 * Description:
 *   Send UBX AID-INI message (Aiding position, time, frequency, clock drift)
 *
 * Input Parameters:
 *   gps         - GPS object
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ubgps_send_aid_ini(struct ubgps_s * const gps);

/****************************************************************************
 * Name: ubgps_send_aid_alp_poll
 *
 * Description:
 *   Send UBX AID-ALP poll message
 *
 * Input Parameters:
 *   gps         - GPS object
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ubgps_send_aid_alp_poll(struct ubgps_s * const gps);

/****************************************************************************
 * Name: ubgps_handle_aid_alp
 *
 * Description:
 *   Handle AID-ALP UBX message
 *
 * Input Parameters:
 *   gps         - GPS object
 *   msg         - UBX AID-ALP message
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ubgps_handle_aid_alp(struct ubgps_s * const gps,
                         struct ubx_msg_s const * const msg);

/****************************************************************************
 * Name: ubgps_parse_nav_pvt
 *
 * Description:
 *   Parse UBX NAV-PVT message
 *
 * Input Parameters:
 *   gps         - GPS object
 *   msg         - UBX NAV-PVT message
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ubgps_parse_nav_pvt(struct ubgps_s * const gps,
                        struct ubx_msg_s const * const msg);

/****************************************************************************
 * Name: ubgps_handle_nav_pvt
 *
 * Description:
 *   Handle NAV-PVT UBX message
 *
 * Input Parameters:
 *   gps         - GPS object
 *   msg         - UBX NAV-PVT message
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ubgps_handle_nav_pvt(struct ubgps_s * const gps,
                         struct ubx_msg_s const * const msg);

/****************************************************************************
 * Name: ubgps_handle_aid_alpsrv
 *
 * Description:
 *   Handle AID-ALPSRV UBX message
 *
 * Input Parameters:
 *   gps         - GPS object
 *   msg         - UBX AID-ALPSRV message
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ubgps_handle_aid_alpsrv(struct ubgps_s * const gps,
                            struct ubx_msg_s const * const msg);

/****************************************************************************
 * Name: ubgps_set_nonblocking
 *
 * Description:
 *   Set GPS file descriptor non-blocking or blocking
 *
 * Input Parameters:
 *   gps         - GPS object
 *   set         - Set non-blocking if true; set blocking if false
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ubgps_set_nonblocking(struct ubgps_s * const gps, bool const set);

/****************************************************************************
 * Name: __ubgps_set_timer
 *
 * Description:
 *   Setup timer with callback.
 *
 ****************************************************************************/

int __ubgps_set_timer(struct ubgps_s *gps,
                      uint32_t timeout_msec, ubgps_timer_fn_t timer_cb,
                      void *cb_priv);

/****************************************************************************
 * Name: __ubgps_remove_timer
 *
 * Description:
 *   Remove timer.
 *
 ****************************************************************************/

void __ubgps_remove_timer(struct ubgps_s * const gps, uint16_t id);

/****************************************************************************
 * Name: ubgps_callback_unregister
 *
 * Description:
 *   Free unactive callbacks from GPS module
 *
 ****************************************************************************/

void __ubgps_gc_callbacks(struct ubgps_s * const gps);

/****************************************************************************
 * Name: ubgps_check_alp_file_validity
 *
 * Description:
 *   Check that ALP file contains valid aiding data.
 *
 ****************************************************************************/

bool ubgps_check_alp_file_validity(const char *filepath);

/****************************************************************************
 * Name: ubgps_aid_updater_start
 *
 * Description:
 *   Start assisted data updater
 *
 ****************************************************************************/

int ubgps_aid_updater_start(struct gps_assistance_s * const assist);

/****************************************************************************
 * Name: ubgps_aid_stop
 *
 * Description:
 *   Stop assisted data updater
 *
 ****************************************************************************/

int ubgps_aid_updater_stop(struct gps_assistance_s * const assist);

/****************************************************************************
 * Name: ubgps_psm_timer_cb
 *
 * Description:
 *   handler for PSM timeout callback
 *
 * Input Parameters:
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ubgps_psm_timer_cb(const int timer_id, const struct timespec *date,
                       void * const priv);

#endif /* __THINGSEE_GPS_INTERNAL_H */
