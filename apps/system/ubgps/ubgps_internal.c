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
#include <apps/thingsee/ts_core.h>

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


#ifdef CONFIG_UBGPS_PSM_MODE
/****************************************************************************
 * Name: ubgps_filter_location
 *
 * Description:
 *   Filter location events in power save mode. Location events are published
 *   until horizontal accuracy is below defined threshold or location filter
 *   timeout occurs.
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
  DEBUGASSERT(gps && levent);

  if (gps->state.navigation_rate < CONFIG_UBGPS_PSM_MODE_THRESHOLD*1000)
    {
      /* Navigation rate less than PSM threshold. */

      ubgps_publish_event(gps, (struct gps_event_s *)levent);
      return;
    }

  if (CONFIG_UBGPS_PSM_ACCURACY_FILTER_DURATION == 0 ||
      (levent->location->horizontal_accuracy / 1000 <
      CONFIG_UBGPS_PSM_ACCURACY_FILTER_THRESHOLD))
    {
      struct sm_event_psm_event_s psm;

      /* Filter disabled or accuracy below threshold */

      ubgps_publish_event(gps, (struct gps_event_s *)levent);

      if (gps->state.location_timer_id >= 0)
        {
          __ubgps_remove_timer(gps, gps->state.location_timer_id);

          gps->state.location_timer_id = -1;
        }

      gps->filt_location.horizontal_accuracy = 0;

      /* Construct power save mode event */

      psm.super.id = SM_EVENT_PSM_STATE;
      psm.enable = true;
      ubgps_sm_process(gps, (struct sm_event_s *)&psm);

      return;
    }

  if (gps->filt_location.horizontal_accuracy == 0)
    {
      /* The first location event that is not below threshold,
         start timeout timer to collect location events. */

      gps->state.location_timer_id = __ubgps_set_timer(gps,
        CONFIG_UBGPS_PSM_ACCURACY_FILTER_DURATION*1000, ubgps_timeout, gps);
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
#endif /* CONFIG_UBGPS_PSM_MODE */

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
  gps_state_t state;
  int status;

  DEBUGASSERT(gps && event);

  /* Inject event to state machine */

  state = gps->state.current_state;
  status = gps_sm[state](gps, event);

  /* Check status */

  if (status != OK)
    {
      dbg("%s: event id: %d, status %s\n", ts_gps_get_statename(state),
          event->id, status == OK ? "OK" : "ERROR");
    }

  /* Check for state change */

  if (gps->state.new_state != state)
    {
      struct sm_event_s sevent;
      struct gps_event_state_change_s cevent;

      /* Construct and send exit event to old state */

      sevent.id = SM_EVENT_EXIT;
      (void)gps_sm[state](gps, &sevent);

      /* Change to new state */

      state = gps->state.new_state;
      gps->state.current_state = state;

      /* Construct and send entry event to new state */

      sevent.id = SM_EVENT_ENTRY;
      (void)gps_sm[state](gps, &sevent);

      /* Publish state change event */

      cevent.super.id = GPS_EVENT_STATE_CHANGE;
      cevent.state = state;
      ubgps_publish_event(gps, (struct gps_event_s *)&cevent);
    }

  return status;
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
  UBX_SET_X4(msg, 4,  flags);         /* PSM flags */

  /* Update period is limited in cyclic mode between 1000-10000ms. Update
     period configures update rate for power optimized tracking (POT) state in
     cyclic mode. Update rate of tracking state is configured with CFG-RATE. */

  if (update_period < 1000)
    update_period = 1000;
  else if (update_period > 10000)
    update_period = 10000;

  UBX_SET_U4(msg, 8,  update_period);

  /* Search_period and acquisition timeout parameters are obsolete if
     doNotEnterPowerOff is defined in flags */

  UBX_SET_U4(msg, 12, search_period); /* search period [ms] */
  UBX_SET_U4(msg, 16, 0);             /* grid offset [ms] */
  UBX_SET_U2(msg, 20, 1);             /* on time [s] */
  UBX_SET_U2(msg, 22, 1);             /* acquisition timeout [s] */

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
int ubgps_send_cfg_rst(struct ubgps_s * const gps)
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

  UBX_SET_U2(msg, 0, 0xffff);
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
  struct tm t;
  uint32_t latitude = 0;
  uint32_t longitude = 0;
  int32_t altitude = 0;
  uint32_t accuracy = 0;
  uint16_t yymm = 0;
  uint32_t ddhhmmss = 0;
  uint32_t flags = 0;
  int status;

  DEBUGASSERT(gps);

  /* Check if UBX receiver is busy */

  if (ubx_busy(&gps->state.ubx_receiver))
    return ERROR;

  if (board_rtc_time_is_set(NULL))
    {
      time_t currtime;

      dbg_int("Using system time for GPS.\n");

      /* Construct time and date for GPS */

      currtime = time(0);
      gmtime_r(&currtime, &t);
      yymm = ((t.tm_year - 100) << 8) + (t.tm_mon + 1);
      ddhhmmss = ((uint32_t)t.tm_mday << 24) +
                  ((uint32_t)t.tm_hour << 16) +
                  ((uint32_t)t.tm_min << 8) +
                  t.tm_sec;

      /* Time is valid and Time is in UTC format */

      flags |= (1 << 1) | (1 << 10);
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

      dbg_int("Using last known position:\n");
      dbg_int(" lat     : %d\n", gps->hint->latitude);
      dbg_int(" long    : %d\n", gps->hint->longitude);
      dbg_int(" alt     : %d meters\n", altitude / 100);
      dbg_int(" acc_old : %u meters\n", gps->hint->accuracy);
      dbg_int(" acc_curr: %u meters\n", (uint32_t)current_accuracy / 100);

      /* Position is given in lat / long / alt */

      flags |= (1 << 5);

      /* Position is valid. */

      flags |= (1 << 0);
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
  UBX_SET_U2(msg, 18, yymm);      /* Year / month */
  UBX_SET_U4(msg, 20, ddhhmmss);  /* Day / hour / minute / seconds (ddhhmmss) */
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
int ubgps_parse_nav_pvt(struct ubgps_s * const gps, struct ubx_msg_s const * const msg)
{
  uint8_t valid = UBX_GET_X1(msg, 11);
  uint8_t fix_type = UBX_GET_X1(msg, 20);
  uint8_t flags = UBX_GET_X1(msg, 21);

  /* Initialize time data */

  memset(&gps->time, 0, sizeof(gps->time));

  /* Get time / date validity */

  gps->time.validity.date = (valid & (1 << 0) ? true : false);
  gps->time.validity.time = (valid & (1 << 1) ? true : false);
  gps->time.validity.fully_resolved = (valid & (1 << 2) ? true : false);

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

  /* Initialize location data */

  memset(&gps->location, 0, sizeof(gps->location));

  /* Get fix type */

  if (fix_type < __GPS_FIX_MAX && (flags & NAV_FLAG_FIX_OK))
    gps->location.fix_type = fix_type;
  else
    gps->location.fix_type = GPS_FIX_NOT_AVAILABLE;

  /* Get location data if there's a fix */

  if (gps->location.fix_type > GPS_FIX_NOT_AVAILABLE)
    {
      /* Get number of satellites used in navigation solution */

      gps->location.num_of_used_satellites = UBX_GET_U1(msg, 23);

      /* Get longitude, latitude and horizontal accuracy */

      gps->location.longitude = UBX_GET_I4(msg, 24);
      gps->location.latitude = UBX_GET_I4(msg, 28);
      gps->location.horizontal_accuracy = UBX_GET_U4(msg, 40);

      /* Get height and vertical accuracy */

      gps->location.height = UBX_GET_I4(msg, 36);
      gps->location.vertical_accuracy = UBX_GET_U4(msg, 44);

      /* Get ground speed and accuracy */

      gps->location.ground_speed = UBX_GET_I4(msg, 60);
      gps->location.ground_speed_accuracy = UBX_GET_U4(msg, 68);

      /* Get heading and heading accuracy estimate */

      gps->location.heading = UBX_GET_I4(msg, 64);
      gps->location.heading_accuracy = UBX_GET_U4(msg, 72);

      dbg_int("fix:%d, flags:0x%02X, sat:%u, lat:%d, lon:%d, acc:%um, height:%dm,\n"
              "acc:%um, speed:%dm/s, acc:%um/s, heading:%ddeg, acc:%udeg\n",
               gps->location.fix_type,
               flags,
               gps->location.num_of_used_satellites,
               gps->location.latitude,
               gps->location.longitude,
               gps->location.horizontal_accuracy/1000,
               gps->location.height/1000,
               gps->location.vertical_accuracy/1000,
               gps->location.ground_speed/1000,
               gps->location.ground_speed_accuracy/1000,
               gps->location.heading/100000,
               gps->location.heading_accuracy/100000);

      /* Update location hint for A-GPS. */

      (void)ubgps_give_location_hint((double)gps->location.latitude / 1e7,
                                     (double)gps->location.longitude / 1e7,
                                     gps->location.height / 1000,
                                     gps->location.horizontal_accuracy / 1000);
    }

  return OK;
}

/****************************************************************************
 * Name: ubgps_psm_timer_cb
 *
 * Description:
 *   timeout handler for PSM callback
 *
 * Input Parameters:
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ubgps_psm_timer_cb(const int timer_id, const struct timespec *date,
                       void * const priv)
{
  struct ubgps_s * const gps = (struct ubgps_s *)priv;

  /* Off-time expired, acquire fix */

  if (timer_id == gps->state.psm_timer_id)
    {
      struct sm_event_psm_event_s psm;

      psm.super.id = SM_EVENT_PSM_STATE;
      psm.enable = false;
      ubgps_sm_process(gps, (struct sm_event_s *)&psm);

      ts_core_timer_stop(gps->state.psm_timer_id);
      gps->state.psm_timer_id = -1;
    }
  else
    {
      dbg("invalid timer id\n");
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

  DEBUGASSERT(gps && msg);

  /* Parse UBX NAV-PVT message */

  ubgps_parse_nav_pvt(gps, msg);

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

#ifdef CONFIG_UBGPS_PSM_MODE
      ubgps_filter_location(gps, &levent);
#else
      ubgps_publish_event(gps, (struct gps_event_s *)&levent);
#endif

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
  if (ret < 0)
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
