/****************************************************************************
 * apps/system/ubgps/ubgps_internal.c
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
#include <debug.h>
#include <queue.h>
#include <poll.h>
#include <nuttx/random.h>

#include "ubgps_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SYSTEM_UBGPS_VERBOSE_DEBUG
 #define dbg_int(...) dbg(__VA_ARGS__)
#else
  #define dbg_int(...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct queued_event_s
{
  sq_entry_t node;
  struct sm_event_target_state_s event;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

 /* Mutex to protect access for aiding data */

pthread_mutex_t g_aid_mutex;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void nav_flags_to_str(char *buf, size_t buflen, uint8_t flags)
{
  const char *valid = (flags & NAV_FLAG_FIX_OK) ? "FixOK" : "FixNOK";
  const char *headvalid = (flags & NAV_FLAG_HEADING_VALID) ? ",HeadOK" : "";
  const char *psm_state;

  switch (flags & NAV_FLAG_PSM_MASK)
    {
    case 0:
      psm_state = "";
      break;
    case NAV_FLAG_PSM_EN:
      psm_state = ",pEN";
      break;
    case NAV_FLAG_PSM_ACQ:
      psm_state = ",pACQ";
      break;
    case NAV_FLAG_PSM_TRK:
      psm_state = ",pTRK";
      break;
    case NAV_FLAG_PSM_POT:
      psm_state = ",pPOT";
      break;
    case NAV_FLAG_PSM_INA:
      psm_state = ",pINA";
      break;
    default:
      psm_state = ",p???";
      break;
    }

  snprintf(buf, buflen, "%s%s%s", valid, headvalid, psm_state);
}

static void utctime_to_gpstime(const struct timespec *ts, uint16_t *week,
                               uint32_t *wmsec)
{
  const time_t gps_utc_leap_sec_diff_2017 = 18;
  const time_t gps_epoch_secs = 315964800;
  const time_t seconds_per_week = 7 * 24 * 60 * 60;
  time_t gps_secs;

  gps_secs = ts->tv_sec - gps_epoch_secs + gps_utc_leap_sec_diff_2017;

  *week = gps_secs / seconds_per_week;
  gps_secs %= seconds_per_week;

  *wmsec = gps_secs * 1000 + ts->tv_nsec / (1000 * 1000);
}

/****************************************************************************
 * Name: ubgps_filter_location
 *
 * Description:
 *   Filter location events and publish until horizontal accuracy is below
 *   defined threshold or location filter timeout occurs.
 *
 * Input Parameters:
 *   gps         - GPS object
 *   levent      - Location event
 *
 * Returned Values:
 *
 ****************************************************************************/
static void ubgps_filter_location(struct ubgps_s * const gps,
  struct gps_event_location_s const * const levent)
{
#ifdef CONFIG_UBGPS_ACCURACY_FILTER_DURATION
  const int accuracy_filter_duration = CONFIG_UBGPS_ACCURACY_FILTER_DURATION;
#else
  const int accuracy_filter_duration = 10;
#endif
#ifdef CONFIG_UBGPS_ACCURACY_FILTER_THRESHOLD
  const int accuracy_filter_threshold = CONFIG_UBGPS_ACCURACY_FILTER_THRESHOLD;
#else
  const int accuracy_filter_threshold = 100;
#endif

  DEBUGASSERT(gps && levent);

  if (accuracy_filter_duration == 0 ||
      (levent->location->horizontal_accuracy / 1000 <
       accuracy_filter_threshold))
    {
      /* Filter disabled or accuracy below threshold */

      ubgps_publish_event(gps, (struct gps_event_s *)levent);

      if (gps->state.location_timer_id >= 0)
        {
          __ubgps_remove_timer(gps, gps->state.location_timer_id);

          gps->state.location_timer_id = -1;
        }

      gps->filt_location.horizontal_accuracy = 0;

      return;
    }

  if (gps->filt_location.horizontal_accuracy == 0)
    {
      /* The first location event that is not below threshold,
         start timeout timer to collect location events. */

      gps->state.location_timer_id = __ubgps_set_timer(gps,
        accuracy_filter_duration * 1000, ubgps_timeout, gps);
    }

  if (gps->filt_location.horizontal_accuracy == 0 ||
      (levent->location->horizontal_accuracy <
      gps->filt_location.horizontal_accuracy))
    {
      /* More accurate location than previous one, update location data. */

      memcpy(&gps->filt_location, levent->location, sizeof(*levent->location));
      return;
    }
}

/****************************************************************************
 * Name: ubgps_process_event_timer
 ****************************************************************************/
static int ubgps_process_event_timer(const int timer_id, void * const priv)
{
  struct ubgps_s * const gps = priv;

  ubgps_process_state_change_queue(gps);

  return OK;
}

/****************************************************************************
 * Public Functions
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
int ubgps_sm_process(struct ubgps_s * const gps, struct sm_event_s const * const event)
{
  gps_state_t before_state;
  gps_state_t after_state;
  gps_state_t state;
  int status;

  DEBUGASSERT(gps && event);

  /* Proceed pending state changes */

  ubgps_process_state_change_queue(gps);

  ++gps->state.in_sm_process;

  if (gps->state.in_state_change)
    {
      dbg("WARNING! processing event while in state change!\n");
    }

  /* Inject event to state machine */

  state = gps->state.current_state;
  before_state = state;
  status = ubgps_sm(gps, state)->func(gps, event);
  after_state = gps->state.current_state;

  if (before_state != after_state)
    {
      dbg("WARNING! ubgps_sm_process() was re-entered from gps_sm[state](),"
          "and current_state changed (before %s, after %s).\n",
          ubgps_get_statename(before_state), ubgps_get_statename(after_state));
      state = after_state;
    }

  /* Check status */

  if (status != OK)
    {
      dbg("%s: event id: %d, status %s\n", ubgps_get_statename(state),
          event->id, status == OK ? "OK" : "ERROR");
    }

  /* Check for state change */

  if (gps->state.new_state != state)
    {
      struct sm_event_s sevent;
      struct gps_event_state_change_s cevent;

      gps->state.in_state_change = true;

      /* Construct and send exit event to old state */

      sevent.id = SM_EVENT_EXIT;
      (void)ubgps_sm(gps, state)->func(gps, &sevent);

      /* Change to new state */

      state = gps->state.new_state;
      gps->state.current_state = state;

      /* Construct and send entry event to new state */

      sevent.id = SM_EVENT_ENTRY;
      (void)ubgps_sm(gps, state)->func(gps, &sevent);

      /* Publish state change event */

      cevent.super.id = GPS_EVENT_STATE_CHANGE;
      cevent.state = state;
      ubgps_publish_event(gps, (struct gps_event_s *)&cevent);

      gps->state.in_state_change = false;
    }

  gps->state.in_sm_process--;

  /* Proceed pending state changes */

  ubgps_process_state_change_queue(gps);

  return status;
}

/****************************************************************************
 * Name: ubgps_process_state_change_queue
 *
 * Description:
 *   Process deferred state changes.
 *
 * Input Parameters:
 *   gps         - GPS object
 *
 ****************************************************************************/
void ubgps_process_state_change_queue(struct ubgps_s * const gps)
{
  struct queued_event_s *pqevent;

  if (gps->state.in_queue_process)
    {
      return;
    }
  if (gps->state.in_sm_process > 0)
    {
      return;
    }

  gps->state.in_queue_process = true;

  /* Process pending state changes. */

  pqevent = (void *)sq_remfirst(&gps->event_target_state_queue);
  while (pqevent)
    {
      struct sm_event_target_state_s nevent = pqevent->event;

      free(pqevent);

      dbg_int("Process deferred state change request "
              "(target_state: %d, timeout: %d),\n",
              nevent.target_state, nevent.timeout);

      /* Mark target state transition as pending */

      gps->state.target_state_pending = true;

      /* Process event */

      ubgps_sm_process(gps, (struct sm_event_s *)&nevent);

      /* Next event */

      pqevent = (void *)sq_remfirst(&gps->event_target_state_queue);
    }

  gps->state.in_queue_process = false;
}

/****************************************************************************
 * Name: ubgps_queue_state_change
 *
 * Description:
 *   Queue state change deferred processing
 *
 * Input Parameters:
 *   gps         - GPS object
 *   event       - Pointer to processed event
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ubgps_queue_state_change(struct ubgps_s * const gps,
                             struct sm_event_target_state_s *event)
{
  struct queued_event_s *pqevent;
  int timer_id;

  pqevent = calloc(1, sizeof(*pqevent));
  if (!pqevent)
    return ERROR;

  pqevent->event = *event;

  /* Defer event processing to prevent ubgps_sm_process re-entry. */

  timer_id = __ubgps_set_timer(gps, 0, ubgps_process_event_timer, NULL);
  if (timer_id < 0)
    {
      free(pqevent);
      return ERROR;
    }

  sq_addlast(&pqevent->node, &gps->event_target_state_queue);
  return OK;
}

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
                        bool const timeout)
{
  struct ubgps_s * const gps = (struct ubgps_s *)priv;

  DEBUGASSERT(receiver && gps && msg);

  /* Check for timeout or ACK/NAK event */

  if (timeout ||
      (msg->class_id == UBX_CLASS_ACK && (msg->msg_id == UBX_ACK_ACK || msg->msg_id == UBX_ACK_NAK)))
    {
      struct sm_event_ubx_status_s ubx_status;
      uint8_t class_id;
      uint8_t msg_id;

      if (timeout)
        {
          /* Get message ID's from received message */

          class_id = msg->class_id;
          msg_id = msg->msg_id;
        }
      else
        {
          /* Check that UBX ACK-ACK/NAK message has correct length */

          if (msg->length != UBX_ACK_ACK_LEN)
            return ERROR;

          /* Get message ID's from received message */

          class_id = UBX_GET_U1(msg, 0);
          msg_id = UBX_GET_U1(msg, 1);
        }

      /* Construct UBX status event */

      ubx_status.super.id = SM_EVENT_UBX_STATUS;
      ubx_status.class_id = class_id;
      ubx_status.msg_id = msg_id;

      if (timeout)
        {
          ubx_status.status = UBX_STATUS_TIMEOUT;
        }
      else if (msg->msg_id == UBX_ACK_ACK)
        {
          ubx_status.status = UBX_STATUS_ACK;
        }
      else
        {
          ubx_status.status = UBX_STATUS_NAK;
        }

      /* Process event */

      ubgps_sm_process(gps, (struct sm_event_s *)&ubx_status);
    }
  else
    {
      struct sm_event_ubx_msg_s ubx_msg;

      /* Construct UBX message event */

      ubx_msg.super.id = SM_EVENT_UBX_MESSAGE;
      ubx_msg.msg = msg;

      /* Process event */

      ubgps_sm_process(gps, (struct sm_event_s *)&ubx_msg);
    }

  return OK;
}

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
int nmea_receiver(struct ubgps_s * const gps, uint8_t const data)
{
  bool nmea_send = false;

  /* Check if NMEA line buffer is allocated */

  if (!gps->nmea.line)
    return OK;


  if (data == '\r' || data == '\n')
    {
      if (gps->nmea.current_len)
        {
          /* Add string NULL termination */

          if (gps->nmea.line[(gps->nmea.current_len - 1)])
            gps->nmea.line[gps->nmea.current_len++] = '\0';

          nmea_send = true;
        }
    }
  else
    {
      /* Add NMEA data to line buffer */

      gps->nmea.line[gps->nmea.current_len++] = data;

      /* Check if buffer is full */

      if (gps->nmea.current_len == (gps->nmea.line_size - 1))
        {
          gps->nmea.line[gps->nmea.current_len++] = '\0';

          nmea_send = true;
        }
    }

  /* Send NMEA data */

  if (nmea_send && gps->nmea.current_len > 1)
    {
      struct gps_event_nmea_data_s nmea;

      /* Construct and publish NMEA event */

      nmea.super.id = GPS_EVENT_NMEA_DATA;
      nmea.line = gps->nmea.line;
      ubgps_publish_event(gps, (struct gps_event_s *)&nmea);

      /* Reset NMEA line length */

      gps->nmea.current_len = 0;
    }

  return OK;
}

/****************************************************************************
 * Name: ubgps_receiver
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
int ubgps_receiver(const struct pollfd * const pfd, void * const priv)
{
  struct ubgps_s * const gps = (struct ubgps_s *)priv;
  uint8_t input_buffer[32];
  ssize_t input_bytes;
  uint8_t * input;
  int ret;

  while (1)
    {
      /* Set file non-blocking */

      ret = ubgps_set_nonblocking(gps, true);
      if (ret < 0)
        return ERROR;

      /* Read bytes from GPS module */

      input_bytes = read(pfd->fd, input_buffer, sizeof(input_buffer));
      if (input_bytes <= 0)
        break;

      /* Set file blocking */

      ret = ubgps_set_nonblocking(gps, false);
      if (ret < 0)
        return ERROR;

      /* Send bytes to UBX message receiver */

      input = input_buffer;
      while (input_bytes--)
        {
          uint8_t data = *input++;

          /* Pass data to UBX receiver */

          if (ubx_msg_receive(gps, data) == ERROR)
            {
              /* Pass data to NMEA receiver */

              nmea_receiver(gps, data);
            }
        }

      /* Run garbage collector. */

      __ubgps_gc_callbacks(gps);
    }

  /* Set file blocking */

  ret = ubgps_set_nonblocking(gps, false);
  if (ret < 0)
    return ERROR;

  return OK;
}

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
void ubgps_publish_event(struct ubgps_s * const gps, struct gps_event_s const * const event)
{
  struct gps_callback_entry_s const * cb;

  DEBUGASSERT(gps && event);

  /* Check if any callback is interested of this event */

  if (!(gps->callback_event_mask & event->id))
    return;

  cb = (struct gps_callback_entry_s *)sq_peek(&gps->callbacks);
  while (cb)
    {
      if ((cb->event_mask & event->id) && cb->callback)
        cb->callback(event, cb->priv);

      cb = (struct gps_callback_entry_s *)sq_next(&cb->entry);
    }
}

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
int ubgps_timeout(const int timer_id, void * const priv)
{
  struct ubgps_s * const gps = (struct ubgps_s *)priv;
  struct sm_event_timeout_s timeout;

  DEBUGASSERT(gps);

  /* Construct timeout event */

  timeout.super.id = SM_EVENT_TIMEOUT;
  timeout.timer_id = timer_id;

  /* Process event */

  ubgps_sm_process(gps, (struct sm_event_s *)&timeout);

  return OK;
}

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
void ubgps_set_new_state(struct ubgps_s * const gps, gps_state_t new_state)
{
  DEBUGASSERT(gps && new_state >= GPS_STATE_POWER_OFF && new_state < __GPS_STATE_MAX);

  /* Set new state machine state */

  gps->state.new_state = new_state;
}

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
void ubgps_report_target_state(struct ubgps_s * const gps, bool const timeout)
{
  struct gps_event_target_state_s event;

  DEBUGASSERT(gps);

  /* Check if target state transition is in progress */

  if (!gps->state.target_state_pending)
    return;

  /* Setup event fields */

  event.current_state = gps->state.current_state;
  event.target_state = gps->state.target_state;

  if (timeout)
    {
      event.super.id = GPS_EVENT_TARGET_STATE_TIMEOUT;
    }
  else if (gps->state.current_state == gps->state.target_state)
    {
      event.super.id = GPS_EVENT_TARGET_STATE_REACHED;
    }
  else
    {
      event.super.id = GPS_EVENT_TARGET_STATE_NOT_REACHED;
    }

  if (gps->state.target_timer_id >= 0)
    {
      if (!timeout)
        {
          /* Stop target state timeout */
          __ubgps_remove_timer(gps, gps->state.target_timer_id);
        }

      /* Clear timeout timer ID */

      gps->state.target_timer_id = -1;
    }

  /* Clear target state transition pending flag */

  gps->state.target_state_pending = false;

  /* Publish event to listeners */

  ubgps_publish_event(gps, (struct gps_event_s *)&event);
}

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
int ubgps_setup_timeout(struct ubgps_s * const gps, uint32_t const timeout)
{
  DEBUGASSERT(gps);

  /* Check for running timeout timer */

  if (gps->state.target_timer_id >= 0)
    {
      __ubgps_remove_timer(gps, gps->state.target_timer_id);

      gps->state.target_timer_id = -1;
    }

  /* Check for timeout */

  if (timeout > 0)
    {
      gps->state.target_timer_id = __ubgps_set_timer(gps,
                                                     timeout * 1000,
                                                     ubgps_timeout,
                                                     gps);

      if (gps->state.target_timer_id < 0)
        return ERROR;
    }

  return OK;
}

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
int ubgps_send_cfg_prt(struct ubgps_s * const gps, uint32_t const baud)
{
  struct ubx_msg_s * msg;
  int status = OK;

  DEBUGASSERT(gps);

  /* Check if UBX receiver is busy */

  if (ubx_busy(&gps->state.ubx_receiver))
    return ERROR;

  /* Allocate and setup CFG-PRT message */

  msg = UBX_MSG_ALLOC(CFG, PRT);
  if (!msg)
    return ERROR;

  UBX_SET_U1(msg, 0, 0x01); /* Port identifier: UART */
  UBX_SET_U1(msg, 1, 0x00); /* Reserved */
  UBX_SET_X2(msg, 2, 0x0000); /* TX ready PIN configuration */
  UBX_SET_X4(msg, 4, 0x000008c0); /* UART mode: 8bit, no parity */
  UBX_SET_U4(msg, 8, baud); /* Baud rate */
  UBX_SET_X2(msg, 12, 0x0001); /* Input protocols: UBX */

  /* Output protocols: UBX + optional NMEA */

  UBX_SET_X2(msg, 14, (1 << 0) | (gps->state.nmea_protocol_enabled ? 1 << 1 : 0));
  UBX_SET_X2(msg, 16, 0x0000); /* Flags */
  UBX_SET_U2(msg, 18, 0x0000); /* Reserved */

  status = ubx_msg_send(gps, gps->fd, msg);

  UBX_MSG_FREE(msg);

  return status;
}

/****************************************************************************
 * Name: ubgps_send_cfg_ant
 *
 * Description:
 *   Send UBX CFG-ANT message (Antenna configuration)
 *
 * Input Parameters:
 *   gps         - GPS object
 *   flags       - Control flags
 *   pins        - Pins to be configured
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ubgps_send_cfg_ant(struct ubgps_s * const gps, uint16_t const flags,
                       uint16_t const pins)
{
  struct ubx_msg_s * msg;
  int status = OK;

  DEBUGASSERT(gps);

  /* Check if UBX receiver is busy */

  if (ubx_busy(&gps->state.ubx_receiver))
    return ERROR;

  /* Allocate and setup CFG-ANT message */

  msg = UBX_MSG_ALLOC(CFG, ANT);
  if (!msg)
    return ERROR;

  UBX_SET_X2(msg, 0, flags);  /* Control flags */
  UBX_SET_X2(msg, 2, pins);   /* Pins to be configured */

  status = ubx_msg_send(gps, gps->fd, msg);

  UBX_MSG_FREE(msg);

  return status;
}

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
int ubgps_send_cfg_msg(struct ubgps_s * const gps, uint8_t class_id, uint8_t msg_id, uint8_t rate)
{
  struct ubx_msg_s * msg;
  int status = OK;

  DEBUGASSERT(gps);

  /* Check if UBX receiver is busy */

  if (ubx_busy(&gps->state.ubx_receiver))
    return ERROR;

  /* Allocate and setup CFG-MSG message */

  msg = UBX_MSG_ALLOC(CFG, MSG);
  if (!msg)
    return ERROR;

  UBX_SET_U1(msg, 0, class_id);
  UBX_SET_U1(msg, 1, msg_id);
  UBX_SET_U1(msg, 2, rate);

  status = ubx_msg_send(gps, gps->fd, msg);

  UBX_MSG_FREE(msg);

  return status;
}

/****************************************************************************
 * Name: ubgps_send_cfg_rate
 *
 * Description:
 *   Send UBX CFG-RATE message (Set navigation rate)
 *
 * Input Parameters:
 *   gps         - GPS object
 *   rate_ms     - Navigation rate (see UBX CFG-RATE)
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ubgps_send_cfg_rate(struct ubgps_s * const gps, uint16_t rate_ms)
{
  struct ubx_msg_s * msg;
  int status = OK;

  DEBUGASSERT(gps);

  /* Check if UBX receiver is busy */

  if (ubx_busy(&gps->state.ubx_receiver))
    return ERROR;

  /* Allocate and setup CFG-RATE message */
  msg = UBX_MSG_ALLOC(CFG, RATE);
  if (!msg)
    return ERROR;

  UBX_SET_U2(msg, 0, rate_ms);
  UBX_SET_U2(msg, 2, 1); /* navRate, always 1 */
  UBX_SET_U2(msg, 4, 0); /* timeRef, 0 - UTC */

  status = ubx_msg_send(gps, gps->fd, msg);

  UBX_MSG_FREE(msg);

  return status;
}

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
int ubgps_send_cfg_sbas(struct ubgps_s * const gps, bool enable)
{
  struct ubx_msg_s * msg;
  int status = OK;

  DEBUGASSERT(gps);

  /* Check if UBX receiver is busy */

  if (ubx_busy(&gps->state.ubx_receiver))
    return ERROR;

  /* Allocate and setup CFG-SBAS message */

  msg = UBX_MSG_ALLOC(CFG, SBAS);
  if (!msg)
    return ERROR;

  UBX_SET_X1(msg, 0, enable);               /* Mode */
  UBX_SET_X1(msg, 1, SBAS_USAGE_RANGE | SBAS_USAGE_DIFFCORR);
  UBX_SET_U1(msg, 2, 0);                    /* maxSBAS, obsolete */
  UBX_SET_X1(msg, 3, SBAS_SCAN_MODE_AUTO);  /* scanmode2 */
  UBX_SET_X4(msg, 4, SBAS_SCAN_MODE_AUTO);  /* scanmode1 */

  status = ubx_msg_send(gps, gps->fd, msg);

  UBX_MSG_FREE(msg);

  return status;
}

/****************************************************************************
 * Name: ubgps_send_cfg_nav5
 *
 * Description:
 *   Send UBX CFG-NAV5 message (navigation engine settings)
 *
 * Input Parameters:
 *   gps         - GPS object
 *   mask        - parameters to be applied
 *   dyn_model   - Dynamic platform model
 *   fix_mode    - Fix mode
 *   hold_thr    - static hold velocity threshold
 *   hold_dist   - static hold distance threshold (to quit static hold)
 *   pos_acc     - position accuracy mask
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ubgps_send_cfg_nav5(struct ubgps_s * const gps, uint16_t mask,
    uint8_t dyn_model, uint8_t fix_mode, uint8_t hold_thr, uint16_t hold_dist,
    uint16_t pos_acc)
{
  struct ubx_msg_s * msg;
  int status = OK;

  DEBUGASSERT(gps);

  /* Check if UBX receiver is busy */

  if (ubx_busy(&gps->state.ubx_receiver))
    return ERROR;

  /* Allocate and setup CFG-NAV5 message */

  msg = UBX_MSG_ALLOC(CFG, NAV5);
  if (!msg)
    return ERROR;

  UBX_SET_X2(msg, 0, mask);       /* Mask - parameters to be applied */
  UBX_SET_U1(msg, 2, dyn_model);  /* Dynamic platform model */
  UBX_SET_U1(msg, 3, fix_mode);   /* fixMode */
  UBX_SET_I4(msg, 4, 0);          /* fixedAlt, N/A in auto mode */
  UBX_SET_U4(msg, 8, 0);          /* fixedAltVar, N/A in auto mode */
  UBX_SET_I1(msg, 12, 5);         /* minElev */
  UBX_SET_U2(msg, 14, 250);       /* pDop [1e-1] */
  UBX_SET_U2(msg, 16, 250);       /* tDop [1e-1] */
  UBX_SET_U2(msg, 18, pos_acc);   /* pAcc [m] */
  UBX_SET_U2(msg, 20, 300);       /* tAcc */
  UBX_SET_U2(msg, 22, hold_thr);  /* staticHoldThresh [cm/s] */
  UBX_SET_U1(msg, 23, 60);        /* dgpsTimeout */
  UBX_SET_U1(msg, 24, 0);         /* cnoThreshNumSVs */
  UBX_SET_U1(msg, 25, 0);         /* cnoThreshold */
  UBX_SET_U2(msg, 28, hold_dist); /* staticHoldMaxDist [m] */
  UBX_SET_U1(msg, 30, 0);         /* utcStandard */

  status = ubx_msg_send(gps, gps->fd, msg);

  UBX_MSG_FREE(msg);

  return status;
}

/****************************************************************************
 * Name: ubgps_send_cfg_pm2
 *
 * Description:
 *   Send UBX CFG-PM2 message (Set power management config)
 *
 * Input Parameters:
 *   gps           - GPS object
 *   flags         - power save mode flags
 *   update_period - time between position fix attempts
 *   search_period - time between acquisition attempts
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ubgps_send_cfg_pm2(struct ubgps_s * const gps, uint32_t flags,
                       uint32_t update_period, uint32_t search_period)
{
  struct ubx_msg_s * msg;
  int status = OK;

  DEBUGASSERT(gps);

  /* Check if UBX receiver is busy */

  if (ubx_busy(&gps->state.ubx_receiver))
    return ERROR;

  /* Allocate and setup CFG-PM2 message */

  msg = UBX_MSG_ALLOC(CFG, PM2);
  if (!msg)
    return ERROR;

  UBX_SET_U1(msg, 0,  1);             /* message version */
  UBX_SET_U1(msg, 2,  0);             /* maxStartupStateDur [s] */
  UBX_SET_X4(msg, 4,  flags);         /* PSM flags */

  /* Update period is limited in cyclic mode between 1000-10000ms. Update
     period configures update rate for power optimized tracking (POT) state in
     cyclic mode. Update rate of tracking state is configured with CFG-RATE. */

  if (update_period < PM_CYCLIC_MIN_RATE)
    update_period = PM_CYCLIC_MIN_RATE;
  else if (update_period > PM_CYCLIC_MAX_RATE)
    update_period = PM_CYCLIC_MAX_RATE;

  UBX_SET_U4(msg, 8,  update_period);

  /* Search_period and acquisition timeout parameters are obsolete if
     doNotEnterPowerOff is defined in flags */

  UBX_SET_U4(msg, 12, search_period); /* search period [ms] */
  UBX_SET_U4(msg, 16, 0);             /* grid offset [ms], ignored in cyclic mode */
  UBX_SET_U2(msg, 20, 0);             /* on time [s], 0: enter POT state as soon as possible */
  UBX_SET_U2(msg, 22, 0);             /* acquisition timeout [s] */

  status = ubx_msg_send(gps, gps->fd, msg);

  UBX_MSG_FREE(msg);

  return status;
}

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
int ubgps_send_cfg_rxm(struct ubgps_s * const gps, uint8_t mode)
{
  struct ubx_msg_s * msg;
  int status = OK;

  DEBUGASSERT(gps);

  /* Check if UBX receiver is busy */

  if (ubx_busy(&gps->state.ubx_receiver))
    return ERROR;

  /* Allocate and setup CFG-RXM message */

  msg = UBX_MSG_ALLOC(CFG, RXM);
  if (!msg)
    return ERROR;

  UBX_SET_U1(msg, 0,  8);     /* reserved, always set to 8 */
  UBX_SET_U1(msg, 1,  mode);  /* low power mode */

  status = ubx_msg_send(gps, gps->fd, msg);

  UBX_MSG_FREE(msg);

  return status;
}

/****************************************************************************
 * Name: ubgps_send_cfg_cfg
 *
 * Description:
 *   Send UBX CFG-CFG message (clear, save and load configurations)
 *
 * Input Parameters:
 *   gps         - GPS object
 *   action      - clear, save, load
 *   mask        - configuration sub-sections
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ubgps_send_cfg_cfg(struct ubgps_s * const gps, uint8_t action, uint32_t mask)
{
  struct ubx_msg_s * msg;
  int status = OK;

  DEBUGASSERT(gps);

  /* Check if UBX receiver is busy */

  if (ubx_busy(&gps->state.ubx_receiver))
    return ERROR;

  /* Allocate and setup CFG-CFG message */

  msg = UBX_MSG_ALLOC(CFG, CFG);
  if (!msg)
    return ERROR;

  switch (action)
    {
      case CFG_ACT_CLEAR:
        UBX_SET_X4(msg, 0, mask);
        break;

      case CFG_ACT_SAVE:
        UBX_SET_X4(msg, 4, mask);
        break;

      case CFG_ACT_LOAD:
        UBX_SET_X4(msg, 8, mask);
        break;

      default:
        dbg_int("unknown action\n");
        status = ERROR;
        goto error;
    }

  status = ubx_msg_send(gps, gps->fd, msg);

error:
  UBX_MSG_FREE(msg);

  return status;
}

/****************************************************************************
 * Name: ubgps_send_cfg_rst
 *
 * Description:
 *   Send UBX CFG-RST message (Reset Receiver)
 *
 * Input Parameters:
 *   gps         - GPS object
 *   cold        - true for cold reset, otherwise hot
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ubgps_send_cfg_rst(struct ubgps_s * const gps, bool cold)
{
  struct ubx_msg_s * msg;
  int status = OK;

  DEBUGASSERT(gps);

  /* Check if UBX receiver is busy */

  if (ubx_busy(&gps->state.ubx_receiver))
    return ERROR;

  /* Allocate and setup CFG-RST message */

  msg = UBX_MSG_ALLOC(CFG, RST);
  if (!msg)
    return ERROR;

  UBX_SET_U2(msg, 0, cold ? 0xffff : 0x0000);
  UBX_SET_U2(msg, 2, 0x0002);

  status = ubx_msg_send(gps, gps->fd, msg);

  UBX_MSG_FREE(msg);

  return status;
}

/****************************************************************************
 * Name: ubgps_send_aid_alp_poll
 *
 * Description:
 *   Send UBX AID-ALP message to poll almanac data status
 *
 * Input Parameters:
 *   gps         - GPS object
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ubgps_send_aid_alp_poll(struct ubgps_s * const gps)
{
  struct ubx_msg_s * msg;
  int status = OK;

  DEBUGASSERT(gps);

  /* Check if UBX receiver is busy */

  if (ubx_busy(&gps->state.ubx_receiver))
    return ERROR;

  /* Allocate and setup AID-ALP message */

  msg = UBX_MSG_ALLOC(AID, ALP_POLL);
  if (!msg)
    return ERROR;

  status = ubx_msg_send(gps, gps->fd, msg);

  UBX_MSG_FREE(msg);

  return status;
}

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
int ubgps_send_aid_ini(struct ubgps_s * const gps)
{
  struct ubx_msg_s * msg;
  uint32_t latitude = 0;
  uint32_t longitude = 0;
  int32_t altitude = 0;
  uint32_t accuracy = 0;
  uint16_t gps_week = 0;
  uint32_t gps_week_msec = 0;
  uint32_t flags = 0;
  int status;

  DEBUGASSERT(gps);

  /* Check if UBX receiver is busy */

  if (ubx_busy(&gps->state.ubx_receiver))
    return ERROR;

  if (board_rtc_time_is_set(NULL))
    {
      struct timespec currtime;

      dbg_int("Using system time for GPS.\n");

      /* Construct time and date for GPS */

      clock_gettime(CLOCK_REALTIME, &currtime);
      utctime_to_gpstime(&currtime, &gps_week, &gps_week_msec);

      /* Time is valid and Time is in WEEK/MSEC format */

      flags |= (1 << 1);
      flags &= ~(1 << 10);
    }

  if (gps->assist && gps->assist->alp_file)
    dbg_int("Using AssistNow Offline data from '%s', file ID %d.\n",
        gps->assist->alp_file, gps->assist->alp_file_id);

  if (gps->hint && gps->hint->have_location)
    {
      struct timespec currtime = {};
      uint64_t current_accuracy;

      (void)clock_gettime(CLOCK_MONOTONIC, &currtime);

      /* Adjust location accuracy by time*speed. */

      current_accuracy = gps->hint->accuracy;
      current_accuracy +=
          ((uint64_t)HINT_LOCATION_ACCURACY_DEGRADE_SPEED_MPS *
           (currtime.tv_sec - gps->hint->location_time.tv_sec));

      if (current_accuracy > HINT_LOCATION_MAX_ACCURACY)
        {
          dbg_int("Location hint accuracy too poor: %lld meters (max allowed: %d meters)\n",
                  current_accuracy, HINT_LOCATION_MAX_ACCURACY);
        }
      else
        {
          /* Position is given in lat / long / alt */

          flags |= (1 << 5);

          /* Position is valid. */

          flags |= (1 << 0);
        }

      current_accuracy *= 100; /* m => cm */
      if (current_accuracy > UINT32_MAX)
        current_accuracy = UINT32_MAX;

      altitude = gps->hint->altitude * 100; /* m => cm */
      if (altitude > INT32_MAX)
        altitude = INT32_MAX;
      else if (altitude < INT32_MIN)
        altitude = INT32_MIN;

      latitude = gps->hint->latitude;
      longitude = gps->hint->longitude;
      accuracy = current_accuracy;

      if (flags & (1 << 0))
        {
          dbg_int("Using last known position:\n");
          dbg_int(" lat     : %d\n", gps->hint->latitude);
          dbg_int(" long    : %d\n", gps->hint->longitude);
          dbg_int(" alt     : %d meters\n", altitude / 100);
          dbg_int(" acc_old : %u meters\n", gps->hint->accuracy);
          dbg_int(" acc_curr: %u meters\n", (uint32_t)current_accuracy / 100);
        }
    }

  /* Allocate and setup AID-INI message */

  msg = UBX_MSG_ALLOC(AID, INI);
  if (!msg)
    return ERROR;

  UBX_SET_I4(msg, 0, latitude);   /* Latitude */
  UBX_SET_I4(msg, 4, longitude);  /* Longitude */
  UBX_SET_I4(msg, 8, altitude);   /* Altitude in centimeters */
  UBX_SET_U4(msg, 12, accuracy);  /* Position accuracy in centimeters */
  UBX_SET_X2(msg, 16, 0);         /* Time mark configuration */
  UBX_SET_U2(msg, 18, gps_week);  /* GPS week */
  UBX_SET_U4(msg, 20, gps_week_msec); /* Time of week (msec) */
  UBX_SET_I4(msg, 24, 0);         /* Fractional part of time of week */
  UBX_SET_U4(msg, 28, 10000);     /* Milliseconds part of time accuracy */
  UBX_SET_U4(msg, 32, 0);         /* Nanoseconds part of time accuracy */
  UBX_SET_I4(msg, 36, 0);         /* Clock drift or frequency */
  UBX_SET_U4(msg, 40, 0);         /* Accuracy of clock drift or frequency */

  /* Time is valid, Altitude is invalid and Time is in UTC format */

  UBX_SET_X4(msg, 44, flags);

  status = ubx_msg_send(gps, gps->fd, msg);

  UBX_MSG_FREE(msg);

  return status;
}

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
                        struct ubx_msg_s const * const msg, uint8_t *nav_flags)
{
  char flags_str[32];
  uint8_t valid = UBX_GET_X1(msg, 11);
  uint8_t fix_type = UBX_GET_X1(msg, 20);
  uint8_t flags = UBX_GET_X1(msg, 21);

  /* Initialize time data */

  memset(&gps->time, 0, sizeof(gps->time));

  /* Get time / date validity */

  gps->time.validity.date = !!(valid & (1 << 0));
  gps->time.validity.time = !!(valid & (1 << 1));
  gps->time.validity.old_fully_resolved = !!(valid & (1 << 2));

  if (fix_type > GPS_FIX_NOT_AVAILABLE && fix_type < __GPS_FIX_MAX &&
      (flags & NAV_FLAG_FIX_OK))
    {
      gps->time.validity.fully_resolved = gps->time.validity.old_fully_resolved;
    }
  else
    {
      /* Do not consider GPS time resolved unless acquired fix. Otherwise time
       * maybe A-GPS or MCU RTC based and potentially invalid. */

      gps->time.validity.date = false;
      gps->time.validity.time = false;
      gps->time.validity.fully_resolved = false;
    }

  if (gps->time.validity.date || gps->time.validity.fully_resolved)
    {
      /* Get date */

      gps->time.year = UBX_GET_U2(msg, 4);
      gps->time.month = UBX_GET_U1(msg, 6);
      gps->time.day = UBX_GET_U1(msg, 7);
    }

  if (gps->time.validity.time || gps->time.validity.fully_resolved)
    {
      gps->time.hour = UBX_GET_U1(msg, 8);
      gps->time.min = UBX_GET_U1(msg, 9);
      gps->time.sec = UBX_GET_U1(msg, 10);
    }

#ifdef CONFIG_SYSTEM_UBGPS_VERBOSE_DEBUG
  if (gps->time.validity.fully_resolved)
    {
      struct timespec ts_mcu;
      struct timespec ts_gps;
      struct tm gps_t = {};

      /* Prepate time structure */

      gps_t.tm_year = gps->time.year - 1900;
      gps_t.tm_mon = gps->time.month - 1;
      gps_t.tm_mday = gps->time.day;
      gps_t.tm_hour = gps->time.hour;
      gps_t.tm_min = gps->time.min;
      gps_t.tm_sec = gps->time.sec;

      ts_gps.tv_sec = mktime (&gps_t);
      ts_gps.tv_nsec = 0;

      (void)clock_gettime(CLOCK_REALTIME, &ts_mcu);
      (void)ts_gps.tv_sec;

      /* Check difference between MCU RTC (should be UTC) and GPS time. */

      dbg_int("GPS time: %d, MCU time: %d, MCU ahead of GPS time by: %d secs\n",
              ts_gps.tv_sec, ts_mcu.tv_sec, ts_mcu.tv_sec - ts_gps.tv_sec);
    }
#endif

  /* Get location data if there's a fix */

  /* Initialize location data */

  memset(&gps->location, 0, sizeof(gps->location));

  /* Get fix type */

  gps->location.fix_type = fix_type;

  /* Get number of satellites used in navigation solution */

  gps->location.num_of_used_satellites = UBX_GET_U1(msg, 23);

  /* Get longitude [1e-7 deg], latitude [1e-7 deg] and horizontal accuracy [mm] */

  gps->location.longitude = UBX_GET_I4(msg, 24);
  gps->location.latitude = UBX_GET_I4(msg, 28);
  gps->location.horizontal_accuracy = UBX_GET_U4(msg, 40);

  /* Get height [mm] and vertical accuracy [mm] */

  gps->location.height = UBX_GET_I4(msg, 36);
  gps->location.vertical_accuracy = UBX_GET_U4(msg, 44);

  /* Get ground speed [mm/s] and accuracy [mm/s] */

  gps->location.ground_speed = UBX_GET_I4(msg, 60);
  gps->location.ground_speed_accuracy = UBX_GET_U4(msg, 68);

  /* Get heading [1e-5 deg] and heading accuracy estimate [1e-5 deg] */

  gps->location.heading = UBX_GET_I4(msg, 64);
  gps->location.heading_accuracy = UBX_GET_U4(msg, 72);

  nav_flags_to_str(flags_str, sizeof(flags_str), flags);

  dbg_int("fix:%d, flags:0x%02X (%s), sat:%u, lat:%d, lon:%d, acc:%um, height:%dm,\n"
          "acc:%um, speed:%dcm/s, acc:%ucm/s, heading:%ddeg, acc:%udeg\n",
           gps->location.fix_type,
           flags,
           flags_str,
           gps->location.num_of_used_satellites,
           gps->location.latitude,
           gps->location.longitude,
           gps->location.horizontal_accuracy/1000,
           gps->location.height/1000,
           gps->location.vertical_accuracy/1000,
           gps->location.ground_speed/10,
           gps->location.ground_speed_accuracy/10,
           gps->location.heading/100000,
           gps->location.heading_accuracy/100000);

#ifdef CONFIG_UBGPS_DISALLOW_ACQ_FIX
  if ((fix_type > GPS_FIX_NOT_AVAILABLE && fix_type < __GPS_FIX_MAX) &&
      (flags & NAV_FLAG_PSM_MASK) == NAV_FLAG_PSM_ACQ &&
      (flags & NAV_FLAG_FIX_OK))
    {
      flags &= ~NAV_FLAG_FIX_OK;

      dbg_int("%s => %s (blocking %s because in %s)\n",
              "FixOK", "FixNOK", "FixOK", "pACQ");
    }
#endif

  if (fix_type > GPS_FIX_NOT_AVAILABLE && fix_type < __GPS_FIX_MAX &&
      (flags & NAV_FLAG_FIX_OK))
    {
      uint32_t tmp;

      /* Update location hint for A-GPS. */

      (void)ubgps_give_location_hint((double)gps->location.latitude / 1e7,
                                     (double)gps->location.longitude / 1e7,
                                     gps->location.height / 1000,
                                     gps->location.horizontal_accuracy / 1000);

      /* Collect least significant bits and feed to entropy pool. */

      tmp = gps->location.longitude << 0;
      tmp += gps->location.latitude << 3;
      tmp += gps->location.horizontal_accuracy << 6;
      tmp += gps->location.height << 9;
      tmp += gps->location.vertical_accuracy << 12;
      tmp += gps->location.num_of_used_satellites << 15;
      tmp += gps->location.ground_speed << 18;
      tmp += gps->location.ground_speed_accuracy << 21;
      tmp += gps->location.heading << 24;
      tmp += gps->location.heading_accuracy << 27;
      add_sensor_randomness(tmp);
    }
  else
    {
      gps->location.fix_type = GPS_FIX_NOT_AVAILABLE;
    }

  if (nav_flags)
    {
      *nav_flags = flags;
    }

  return OK;
}

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
int ubgps_handle_nav_pvt(struct ubgps_s * const gps, struct ubx_msg_s const * const msg)
{
  struct gps_event_time_s tevent;
  struct gps_event_location_s levent;
  uint8_t nav_flags = 0;
#if defined(BOARD_HAS_GPS_PM_SET_NEXT_MESSAGE_TIME) && \
    !defined(CONFIG_UBGPS_DISABLE_BOARD_POWERSAVE_CONTROL)
  struct timespec next_msg_ts;
  unsigned int next_msg_msec;

  (void)clock_gettime(CLOCK_MONOTONIC, &next_msg_ts);
#endif

  DEBUGASSERT(gps && msg);

  /* Parse UBX NAV-PVT message */

  ubgps_parse_nav_pvt(gps, msg, &nav_flags);

#if defined(BOARD_HAS_GPS_PM_SET_NEXT_MESSAGE_TIME) && \
    !defined(CONFIG_UBGPS_DISABLE_BOARD_POWERSAVE_CONTROL)

  if (gps->state.current_state == GPS_STATE_FIX_ACQUIRED ||
      gps->state.current_state == GPS_STATE_SEARCHING_FIX)
    {
      if (gps->state.power_mode != RXM_POWER_SAVE)
        {
          /* In continuous mode, message rate is fixed. */

          next_msg_msec = gps->state.navigation_rate;
        }
      else
        {
          /* In cyclic mode, message rate depends on tracking mode. */

          if ((nav_flags & NAV_FLAG_PSM_MASK) == NAV_FLAG_PSM_POT ||
              (nav_flags & NAV_FLAG_PSM_MASK) == NAV_FLAG_PSM_TRK)
            {
              /* In tracking and power optimized tracking, messages received at
               * cyclic-mode update rate. */

              next_msg_msec = gps->state.navigation_rate;
            }
          else
            {
              /* In other modes (ACQ), message rate is fixed 2 hz. */

              next_msg_msec = 1000 / 2;
            }
        }

      /* Give board level power-management an estimate when next NAV message is
       * expected to be received. */

      (void)clock_gettime(CLOCK_MONOTONIC, &next_msg_ts);
      next_msg_ts.tv_sec += next_msg_msec / MSEC_PER_SEC;
      next_msg_ts.tv_nsec += (next_msg_msec % MSEC_PER_SEC) * NSEC_PER_MSEC;
      while (next_msg_ts.tv_nsec >= NSEC_PER_SEC)
        {
          next_msg_ts.tv_sec++;
          next_msg_ts.tv_nsec -= NSEC_PER_SEC;
        }

      board_gps_pm_set_next_message_time(&next_msg_ts);

      dbg_int("next message in: %d msec\n", next_msg_msec);
    }
#endif

  /* Check if GPS fix state has changed */

  if (gps->location.fix_type == GPS_FIX_NOT_AVAILABLE &&
      gps->state.current_state == GPS_STATE_FIX_ACQUIRED)
    {
      /* Change state to searching GPS fix */

      ubgps_set_new_state(gps, GPS_STATE_SEARCHING_FIX);
    }
  else if (gps->location.fix_type > GPS_FIX_NOT_AVAILABLE &&
      gps->state.current_state < GPS_STATE_FIX_ACQUIRED)
    {
      /* Change state to GPS fix acquired */

      ubgps_set_new_state(gps, GPS_STATE_FIX_ACQUIRED);
    }

  /* Publish time event only when date and/or time is known */

  if (gps->time.validity.date ||
      gps->time.validity.time ||
      gps->time.validity.fully_resolved)
    {
      /* Construct and publish time event */

      tevent.super.id = GPS_EVENT_TIME;
      tevent.time = &gps->time;
      ubgps_publish_event(gps, (struct gps_event_s *)&tevent);
    }

  /* Publish location event only when location is known */

  if (gps->location.fix_type > GPS_FIX_NOT_AVAILABLE)
    {
      /* Construct and publish location event */

      levent.super.id = GPS_EVENT_LOCATION;
      levent.time = &gps->time;
      levent.location = &gps->location;

      ubgps_filter_location(gps, &levent);
    }

  return OK;
}

/****************************************************************************
 * Name: ubgps_handle_aid_alp
 *
 * Description:
 *   Handle AID-ALP UBX message
 *
 * Input Parameters:
 *   gps         - GPS object
 *   msg         - UBX AID-ALPSRV message
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ubgps_handle_aid_alp(struct ubgps_s * const gps, struct ubx_msg_s const * const msg)
{
  struct sm_event_aid_s event;
  long int age;

  DEBUGASSERT(gps && msg);

  age = UBX_GET_I4(msg, 8);

  dbg_int("pred_start:%lus, pred_dur:%lus, age:%ldh (%lds), pred_wk:%u, sat:%u\n",
      UBX_GET_U4(msg, 0), UBX_GET_U4(msg, 4), age/3600, age,
      UBX_GET_U2(msg, 14), UBX_GET_U1(msg, 20));

  event.super.id = SM_EVENT_AID_STATUS;
  event.age = age;
  ubgps_sm_process(gps, (struct sm_event_s *)&event);

  return OK;
}

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
int ubgps_handle_aid_alpsrv(struct ubgps_s * const gps, struct ubx_msg_s const * const msg)
{
  uint8_t requested_data_type;
  size_t data_offset;
  size_t data_size;
  ssize_t data_read;
  struct ubx_msg_s * resp = NULL;
  int alpfd = -1;
  int status = ERROR;
  int ret;

  DEBUGASSERT(gps && msg);

  if (!gps->assist)
    return OK;

  /* Check that AssistNow Offline data is available */

  ret = pthread_mutex_trylock(&g_aid_mutex);
  if (ret != 0)
    {
      dbg_int("mutex_trylock failed: %d\n", ret);
      return OK;
    }

  if (!gps->assist->alp_file || !gps->assist->alp_file_id)
    {
      pthread_mutex_unlock(&g_aid_mutex);
      return OK;
    }

  if (!ubgps_check_alp_file_validity(gps->assist->alp_file))
    {
      free(gps->assist->alp_file);
      gps->assist->alp_file = NULL;
      pthread_mutex_unlock(&g_aid_mutex);
      return OK;
    }

  requested_data_type = UBX_GET_U1(msg, 1);
  data_offset = (uint32_t)UBX_GET_U2(msg, 2) * 2;
  data_size = (uint32_t)UBX_GET_U2(msg, 4) * 2;

  dbg_int("file_id:%u, type:%u, offset:%u, size:%u\n",
      gps->assist->alp_file_id, requested_data_type, data_offset, data_size);

  /* Check if AID-ALPSRV is a download request */

  if (requested_data_type != 0xff)
    {
      /* Allocate response */

      resp = ubx_msg_allocate(UBX_CLASS_AID,
                              UBX_AID_ALPSRV,
                              UBX_AID_ALPSRV_LEN + data_size);
      if (!resp)
        goto errout;

      /* Copy original data to response */

      memcpy(&resp->payload[0], &msg->payload[0], UBX_AID_ALPSRV_LEN);

      /* Set ALP file ID */

      UBX_SET_U2(resp, 6, gps->assist->alp_file_id);
    }

  /* Open ALP file */

  alpfd = open(gps->assist->alp_file, O_RDWR);
  if (alpfd < 0)
    {
      int error = get_errno();

      dbg("ALP file '%s' open failed: %d.\n", gps->assist->alp_file, error);
      goto errout;
    }

  /* Seek to requested offset */

  status = lseek(alpfd, data_offset, SEEK_SET);
  if (status < 0)
    {
      int error = get_errno();

      dbg("Failed to seek ALP file to offset %d: %d.\n", data_offset,
          error);

      goto errout;
    }

  if (resp)
    {
      /* Read data from AlmanacPlus file */

      data_read = read(alpfd, &resp->payload[UBX_AID_ALPSRV_LEN], data_size);
      if (data_read < 0)
        {
          int error = get_errno();

          dbg("ALP file read failed (offset:%d, size:%d): %d.\n", data_offset,
              data_size, error);

          goto errout;
        }

      /* Set read data size */

      UBX_SET_U2(resp, 8, data_read);

      /* Send response */

      status = ubx_msg_send(gps, gps->fd, resp);
    }
  else
    {
      uint16_t file_id = UBX_GET_U2(msg, 6);

      /* Update data in AlmanacPlus file */

      if (file_id == gps->assist->alp_file_id)
        {
          status = write(alpfd, &msg->payload[8], data_size);
          if (status != data_size)
          {
            status = ERROR;
            dbg("ALP write failed: %d\n", get_errno());
          }
          else
          {
            status = OK;
          }
        }
    }

errout:

  if (resp)
    UBX_MSG_FREE(resp);

  if (alpfd >= 0)
    close(alpfd);

  pthread_mutex_unlock(&g_aid_mutex);

  return status;
}

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
int ubgps_set_nonblocking(struct ubgps_s * const gps, bool const set)
{
  int flags, ret;

  DEBUGASSERT(gps->fd >= 0);

  if (set == gps->is_nonblocking)
    return OK;

  flags = fcntl(gps->fd, F_GETFL, 0);
  if (flags == ERROR)
    return ERROR;

  if (set)
    flags |= O_NONBLOCK;
  else
    flags &= ~O_NONBLOCK;

  ret = fcntl(gps->fd, F_SETFL, flags);
  if (ret == ERROR)
    return ERROR;

  gps->is_nonblocking = set;

  return OK;
}

/****************************************************************************
 * Name: __ubgps_callback_unregister
 *
 * Description:
 *   Free unactive callbacks from GPS module
 *
 * Input Parameters:
 *   gps         - GPS object
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/

void __ubgps_gc_callbacks(struct ubgps_s * const gps)
{
  struct gps_callback_entry_s * cb, * cbnext;

  /* Search for callback in queue */

  cb = (struct gps_callback_entry_s *)sq_peek(&gps->callbacks);
  while (cb)
    {
      /* Save next callback entry */

      cbnext = (struct gps_callback_entry_s *)sq_next(&cb->entry);

      if (cb->event_mask == 0)
        {
          /* Free unactive callback. */

          sq_rem(&cb->entry, &gps->callbacks);
          free(cb);
        }

      cb = cbnext;
    }
}

/****************************************************************************
 * Name: __ubgps_full_write
 ****************************************************************************/

size_t __ubgps_full_write(int gps_fd, const void *buf, size_t writelen)
{
  const uint8_t *writebuf = buf;
  size_t nwritten;
  size_t total = 0;

  do
    {
      nwritten = write(gps_fd, writebuf, writelen);
      if (nwritten == ERROR)
        {
          int error = get_errno();
          if (error != EAGAIN)
            {
              return ERROR;
            }
          nwritten = 0;
        }
      writebuf += nwritten;
      writelen -= nwritten;
      total += nwritten;
    }
  while (writelen > 0);

  return total;
}

/****************************************************************************
 * Name: ubgps_check_alp_file_validity
 *
 * Description:
 *   Check that ALP file contains valid aiding data.
 *
 ****************************************************************************/

bool ubgps_check_alp_file_validity(const char *filepath)
{
  bool ret = false;
  uint8_t buf[2];
  ssize_t rlen;
  int fd;

  fd = open(filepath, O_RDONLY);
  if (fd < 0)
    {
      dbg("could not open file: \"%s\" (errno=%d)\n", filepath, get_errno());
      goto out;
    }

  /* Get header */

  rlen = read(fd, buf, 2);
  if (rlen < 2)
    {
      dbg("could not read header, rlen=%d, errno=%d\n", rlen, get_errno());
      goto out;
    }

  /* Expected header is 'µB' */

  if (!(buf[0] == 0xB5 && buf[1] == 0x62))
    {
      dbg("invalid header, %02X:%02X\n", buf[0], buf[1]);
      goto out;
    }

#if 0
  /* Get tail */

  rlen = lseek(fd, -2, SEEK_END);
  if (rlen == -1)
    {
      dbg("could not seek to tail, rlen=%d, errno=%d\n", rlen, get_errno());
      goto out;
    }

  rlen = read(fd, buf, 2);
  if (rlen < 2)
    {
      dbg("could not read tail, rlen=%d, errno=%d\n", rlen, get_errno());
      goto out;
    }

  /* Expected tail is 'XX' */

  if (!(buf[0] == 'X' && buf[1] == 'X'))
    {
      dbg("invalid tail, %02X:%02X\n", buf[0], buf[1]);
      goto out;
    }
#endif

  /* ALP file appears to be valid. */

  ret = true;

out:
  if (fd >= 0)
    {
      close(fd);
    }
  return ret;
}
