/****************************************************************************
 * apps/system/conman/conman_ubmodem.c
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
 *   Author: Pekka Ervasti <pekka.ervasti@haltian.com>
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
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <poll.h>
#include <arpa/inet.h>

#include <arch/board/board-modem.h>
#include <arch/board/board-device.h>
#include <apps/system/ubmodem.h>
#include <apps/netutils/dnsclient.h>
#include <nuttx/random.h>

#include "conman_dbg.h"
#include "conman_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef MIN
#  define MIN(a,b) (a < b ? a : b)
#endif
#ifndef MAX
#  define MAX(a,b) (a > b ? a : b)
#endif

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

struct sms_queue_entry_s
{
  sq_entry_t node;
  struct conman_msg_send_sms_s *sms;
  const char *message;
  const char *receiver;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int process_sms_queue(struct conman_s *conman);
static void send_sms_cb(struct ubmodem_s *modem, bool sms_sent, void *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void send_sms_cb(struct ubmodem_s *modem, bool sms_sent, void *priv)
{
  struct conman_s *conman = priv;

  if (sms_sent)
    conman_dbg("Successfully sent SMS\n");
  else
    conman_dbg("Failed to send SMS\n");

  (void)process_sms_queue(conman);
}

static int process_sms_queue(struct conman_s *conman)
{
  struct sms_queue_entry_s *item;
  int ret;

  if (conman->ub.status != CONMAN_STATUS_ESTABLISHED)
    return -ENOTCONN;

  /* Get next SMS to send from queue. */

  item = (void *)sq_remfirst(&conman->ub.sms_queue);
  if (!item)
    {
      /* Queue empty. */

      if (conman->ub.sms_sending_active)
        {
          conman_dbg("SMS sending inactive.\n");

          conman->ub.sms_sending_active = false;
        }

      if (conman->ub.sms_connection_requested)
        {
          conman_dbg("Release SMS connection.\n");

          /* We can now close the SMS connection. */

          (void)__conman_ctl_destroy_connection(conman, conman->ub.sms_connid);
          conman->ub.sms_connid = CONMAN_CONNID_CLEAR;

          conman->ub.sms_connection_requested = false;
        }

      return OK;
    }

  ret = ubmodem_send_sms(conman->ub.modem, item->receiver, item->message,
                         send_sms_cb, conman);
  free(item->sms);
  free(item);
  if (ret < 0)
    {
      conman_dbg("ubmodem_send_sms failed!\n");
      send_sms_cb(conman->ub.modem, false, conman);
      return ERROR;
    }

  conman_dbg("SMS sending active.\n");
  conman->ub.sms_sending_active = true;

  return OK;
}

static void event_ip_address(struct ubmodem_s *modem,
    enum ubmodem_event_flags_e event, const void *event_data, size_t datalen,
    void *priv)
{
  const struct ubmodem_event_ip_address_s *ipcfg = event_data;
  struct conman_s *conman = priv;
  uint32_t addr = ntohl(ipcfg->ipaddr.s_addr);
  uint32_t dns1 = ntohl(ipcfg->dns1.s_addr);
  uint32_t dns2 = ntohl(ipcfg->dns2.s_addr);
  const struct in_addr *dns;

  dbg("GPRS IP address: %d.%d.%d.%d\n", (addr >> 24) & 0xff,
      (addr >> 16) & 0xff, (addr >> 8) & 0xff, (addr >> 0) & 0xff);
  dbg("Primary DNS server: %d.%d.%d.%d\n",
      (dns1 >> 24) & 0xff, (dns1 >> 16) & 0xff, (dns1 >> 8) & 0xff,
      (dns1 >> 0) & 0xff);
  dbg("Secondary DNS server: %d.%d.%d.%d\n",
      (dns2 >> 24) & 0xff, (dns2 >> 16) & 0xff, (dns2 >> 8) & 0xff,
      (dns2 >> 0) & 0xff);

  if (dns1 != 0 && dns2 != 0)
    {
      uint8_t b;
      speckrandom_buf(&b, 1);
      dns = (b & 1) ? &ipcfg->dns1 : &ipcfg->dns2;
    }
  else
    {
      dns = (dns1 != 0) ? &ipcfg->dns1 : &ipcfg->dns2;

      /* TODO: What if both DNS addresses are invalid? */
    }

  dns_setserver(dns);

  conman->ub.ipaddr = ipcfg->ipaddr;
}

static void event_failed_target_level(struct ubmodem_s *modem,
    enum ubmodem_event_flags_e event, const void *event_data, size_t datalen,
    void *priv)
{
  const struct ubmodem_event_transition_failed_s *data = event_data;
  struct conman_s *conman = priv;

  dbg("current level: %d, target level: %d, reason: %s\n",
      data->current_level, data->target_level, data->reason);

  conman_dbg("failed target level, powering off (status: %d => %d)\n",
             conman->ub.status, CONMAN_STATUS_DESTROYING);

  conman->ub.status = CONMAN_STATUS_DESTROYING;
  conman->ub.target_level = UBMODEM_LEVEL_POWERED_OFF;
  ubmodem_request_level(modem, UBMODEM_LEVEL_POWERED_OFF);
}

static void ubmodem_info_callback(void *caller_data, const char *data, int datalen, bool status, ubmodem_info_type_t type)
{
  struct conman_s *conman = caller_data;

  switch (type)
  {
  case UB_INFO_IMEI:
    conman->ub.imei_requested = false;
    if (!status)
      {
        memset(conman->ub.imei, 0, sizeof(conman->ub.imei));
        conman_dbg("Received failed status for IMEI\n");
      }
    else
      {
        snprintf(conman->ub.imei, sizeof(conman->ub.imei), "%s", data);
        conman_dbg("IMEI received: %s\n", conman->ub.imei);
      }
    break;
  case UB_INFO_IMSI:
    conman->ub.imsi_requested = false;
    if (!status)
      {
        memset(conman->ub.imsi, 0, sizeof(conman->ub.imsi));
        conman_dbg("Received failed status for IMSI\n");
      }
    else
      {
        snprintf(conman->ub.imsi, sizeof(conman->ub.imsi), "%s", data);
        conman_dbg("IMSI received: %s\n", conman->ub.imsi);
      }
    break;
  case UB_INFO_UDOPN:
    conman->ub.udopn_requested = false;
    if (!status)
      {
        memset(conman->ub.udopn, 0, sizeof(conman->ub.udopn));
        conman_dbg("Received failed status for UDOPN\n");
      }
    else
      {
        snprintf(conman->ub.udopn, sizeof(conman->ub.udopn), "%s", data);
        conman_dbg("UDOPN received: %s\n", conman->ub.udopn);
      }
    break;
  default:
    break;
  }

  if (conman->ub.imei_requested || conman->ub.imsi_requested ||
      conman->ub.udopn_requested)
    {
      return;
    }

  conman_dbg("reached target level (status: %d => %d)\n",
             conman->ub.status, CONMAN_STATUS_ESTABLISHED);

  conman->ub.status = CONMAN_STATUS_ESTABLISHED;
  process_sms_queue(conman);
}

static void event_target_level(struct ubmodem_s *modem,
    enum ubmodem_event_flags_e event, const void *event_data, size_t datalen,
    void *priv)
{
  const struct ubmodem_event_target_level_reached_s *data = event_data;
  struct conman_s *conman = priv;

  dbg("target level reached: %d\n", data->new_level);

  if (!conman->ub.info_requested &&
      conman->ub.status != CONMAN_STATUS_DESTROYING &&
      data->new_level != UBMODEM_LEVEL_POWERED_OFF)
    {
      conman->ub.info_requested = true;
      if (!conman->ub.imei_requested)
        {
          conman_dbg("Requesting IMEI\n");
          if (ubmodem_get_info(conman->ub.modem, ubmodem_info_callback, priv, UB_INFO_IMEI) == OK)
            conman->ub.imei_requested = true;
          else
            conman_dbg("IMEI request failed!\n");
        }

      if (!conman->ub.imsi_requested)
        {
          conman_dbg("Requesting IMSI\n");
          if (ubmodem_get_info(conman->ub.modem, ubmodem_info_callback, priv, UB_INFO_IMSI) == OK)
            conman->ub.imsi_requested = true;
          else
            conman_dbg("IMSI request failed!\n");
        }

      if (!conman->ub.udopn_requested)
        {
          conman_dbg("Requesting UDOPN\n");
          if (ubmodem_get_info(conman->ub.modem, ubmodem_info_callback, priv, UB_INFO_UDOPN) == OK)
            conman->ub.udopn_requested = true;
          else
            conman_dbg("UDOPN request failed!\n");
        }
    }

  switch (conman->ub.status)
    {
    case CONMAN_STATUS_FAILED:
    case CONMAN_STATUS_OFF:
      {
        if (data->new_level != UBMODEM_LEVEL_POWERED_OFF)
          {
            conman_dbg("not at target level, retry... (status: %d)\n",
                       conman->ub.status);

            conman->ub.status = CONMAN_STATUS_DESTROYING;
            conman->ub.target_level = UBMODEM_LEVEL_POWERED_OFF;
            ubmodem_request_level(modem, conman->ub.target_level);
          }
        else
          {
            conman_dbg("reached target level (status: %d => %d)\n",
                       conman->ub.status, CONMAN_STATUS_OFF);

            conman->ub.status = CONMAN_STATUS_OFF;
            conman->ub.info_requested = false;
          }
      }
      break;

    case CONMAN_STATUS_ESTABLISHED:
    case CONMAN_STATUS_ESTABLISHING:
      {
        if (data->new_level != conman->ub.target_level)
          {
            conman_dbg("not at target level, retry... (status: %d)\n",
                       conman->ub.status);

            conman->ub.status = CONMAN_STATUS_ESTABLISHING;
            ubmodem_request_level(modem, conman->ub.target_level);
          }
        else if (!conman->ub.info_requested || conman->ub.imei_requested ||
                 conman->ub.imsi_requested || conman->ub.udopn_requested)
          {
            conman_dbg("info not yet requested, retry... (status: %d)\n",
                       conman->ub.status);

            conman->ub.status = CONMAN_STATUS_ESTABLISHING;
          }
        else
          {
            conman_dbg("reached target level (status: %d => %d)\n",
                       conman->ub.status, CONMAN_STATUS_ESTABLISHED);

            conman->ub.status = CONMAN_STATUS_ESTABLISHED;
            process_sms_queue(conman);
          }
      }
      break;

    case CONMAN_STATUS_DESTROYING:
      {
        if (data->new_level != UBMODEM_LEVEL_POWERED_OFF)
          {
            conman_dbg("not at target level, retry... (status: %d)\n",
                       conman->ub.status);

            conman->ub.status = CONMAN_STATUS_DESTROYING;
            conman->ub.target_level = UBMODEM_LEVEL_POWERED_OFF;
            ubmodem_request_level(modem, conman->ub.target_level);
          }
        else
          {
            conman_dbg("reached target level (status: %d => %d)\n",
                       conman->ub.status, CONMAN_STATUS_OFF);

            conman->ub.status = CONMAN_STATUS_OFF;
            conman->ub.info_requested = false;
          }
      }
      break;
    }
}

static void event_call_status_change(struct ubmodem_s *modem,
                                     enum ubmodem_event_flags_e event,
                                     const void *event_data, size_t datalen,
                                     void *priv)
{
  struct conman_s *conman = priv;
  const struct ubmodem_voice_call_ringing_s *ring_info;

  switch (event)
    {
      case UBMODEM_EVENT_FLAG_CALL_RINGING:
        {
          struct conman_event_call_incoming_info call_info = {};

          conman_dbg("Incoming call, RINGING!\n");

          DEBUGASSERT(datalen == sizeof(*ring_info));
          ring_info = event_data;

          snprintf(call_info.number, sizeof(call_info.number), "%s",
                   ring_info->number);
          call_info.numbertype = ring_info->number_type;

          __conman_send_boardcast_event(conman, CONMAN_EVENT_CALL_INCOMING,
                                        &call_info, sizeof(call_info));
        }
        break;

      case UBMODEM_EVENT_FLAG_CALL_ACTIVE:
        conman_dbg("Call ACTIVE!\n");

        __conman_send_boardcast_event(conman, CONMAN_EVENT_CALL_ACTIVE,
                                      NULL, 0);
        break;

      case UBMODEM_EVENT_FLAG_CALL_DISCONNECTED:
        conman_dbg("Call DISCONNECTED!\n");

        __conman_send_boardcast_event(conman, CONMAN_EVENT_CALL_DISCONNECTED,
                                      NULL, 0);
        break;

      default:
        DEBUGASSERT(false);
        break;
    }
}

#ifdef CONFIG_SYSTEM_CONMAN_DEBUG
static void event_new_level(struct ubmodem_s *modem,
  enum ubmodem_event_flags_e event, const void *event_data, size_t datalen,
  void *priv)
{
  const struct ubmodem_event_new_level_s *data = event_data;

  conman_dbg("old level: %d => new level: %d\n", data->old_level,
      data->new_level);
}

static void event_state_change(struct ubmodem_s *modem,
  enum ubmodem_event_flags_e event, const void *event_data, size_t datalen,
  void *priv)
{
  /* Library internal state change, listener for debugging. */

  conman_dbg("old state: %d => new state: %d\n",
      ((const int *) event_data)[1], ((const int *) event_data)[0]);
}

static void event_cmd_to_modem(struct ubmodem_s *modem,
  enum ubmodem_event_flags_e event, const void *event_data, size_t datalen,
  void *priv)
{
  const char *str = event_data;

  conman_dbg("'%s'\n", str);
}

static void event_resp_from_modem(struct ubmodem_s *modem,
  enum ubmodem_event_flags_e event, const void *event_data, size_t datalen,
  void *priv)
{
  const char *str = event_data;

  conman_dbg("'%s'\n", str);
}

static void event_trace_usrsock(struct ubmodem_s *modem,
  enum ubmodem_event_flags_e event, const void *event_data, size_t datalen,
  void *priv)
{
  const int *tmp = event_data;

  if (datalen != sizeof(int) * 4)
    return;

  switch (tmp[0])
    {
      case UBMODEM_TRACE_USRSOCK_REQ:
        conman_dbg("new usrsock request, type: %d, xid: %d\n",
                   tmp[1], tmp[2]);
        break;
      case UBMODEM_TRACE_USRSOCK_RESP:
        conman_dbg("response, xid: %d, result: %d, inprogress: %d\n",
                   tmp[1], tmp[2], tmp[3]);
        break;
      case UBMODEM_TRACE_USRSOCK_DATARESP:
        conman_dbg("data-response, xid: %d, result: %d, valuelen: %d\n",
                   tmp[1], tmp[2], tmp[3]);
        break;
      case UBMODEM_TRACE_USRSOCK_EVENT:
        conman_dbg("event, usockid: %d, events: %x\n",
                   tmp[1], tmp[2]);
        break;
      default:
        conman_dbg("unknown trace type: %d\n", tmp[0]);
        break;
    }
}

#ifdef CONFIG_SYSTEM_CONMAN_VERBOSE
static void event_data_to_modem(struct ubmodem_s *modem,
  enum ubmodem_event_flags_e event, const void *event_data, size_t datalen,
  void *priv)
{
  const char *buf = event_data;
  size_t i;

  /* Data listener for debugging. */

  printf("%s(): %s: [", __func__, "    to modem");

  for (i = 0; i < datalen; i++)
    {
      char cur = buf[i];

      printf(isprint(cur) ? "%c" : "\\x%02X", cur);
    }

  printf("]\n");
}

static void event_data_from_modem(struct ubmodem_s *modem,
  enum ubmodem_event_flags_e event, const void *event_data, size_t datalen,
  void *priv)
{
  const char *buf = event_data;
  size_t i;

  /* Data listener for debugging. */

  printf("%s(): %s: [", __func__, "from modem");

  for (i = 0; i < datalen; i++)
    {
      char cur = buf[i];

      printf(isprint(cur) ? "%c" : "\\x%02X", cur);
    }

  printf("]\n");
}
#endif /* CONFIG_SYSTEM_CONMAN_VERBOSE */
#endif /* CONFIG_SYSTEM_CONMAN_DEBUG */

static int ubmodem_hw_init(void *priv, bool *is_vcc_off)
{
  return board_modem_initialize(is_vcc_off);
}

static int ubmodem_hw_deinit(void *priv, int serial_fd)
{
  return board_modem_deinitialize(serial_fd);
}

static bool ubmodem_hw_vcc_set(void *priv, bool on)
{
  return board_modem_vcc_set(on);
}

static uint32_t ubmodem_hw_poweron_pin_set(void *priv, bool on)
{
  return board_modem_poweron_pin_set(on);
}

static uint32_t ubmodem_hw_reset_pin_set(void *priv, bool on)
{
  return board_modem_reset_pin_set(on);
}

static const struct ubmodem_hw_ops_s modem_hw_ops =
  {
      .initialize = ubmodem_hw_init,
      .deinitialize = ubmodem_hw_deinit,
      .vcc_set = ubmodem_hw_vcc_set,
      .reset_pin_set = ubmodem_hw_reset_pin_set,
      .poweron_pin_set = ubmodem_hw_poweron_pin_set
  };

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __conman_ubmodem_initialize
 *
 * Description:
 *   Initializes the modem library and registers callbacks for events.
 *
 * Input Parameters:
 *   conman  : connection manager handle
 *
 * Returned Value:
 *   OK    : no errors
 *   ERROR : failure
 *
 ****************************************************************************/

int __conman_ubmodem_initialize(struct conman_s *conman, int *maxfds)
{
  if (conman->ub.modem)
    return ERROR; /* Already initialized. */

  sq_init(&conman->ub.sms_queue);
  conman->ub.sms_connid = CONMAN_CONNID_CLEAR;

  conman->ub.status = CONMAN_STATUS_OFF;
  conman->ub.target_level = UBMODEM_LEVEL_POWERED_OFF;

  conman->ub.modem = ubmodem_initialize(&modem_hw_ops, NULL);
  if (!conman->ub.modem)
    return ERROR;

  ubmodem_set_config_callback(conman->ub.modem, __conman_config_modem_config_cb,
      conman);

  ubmodem_register_event_listener(conman->ub.modem,
      UBMODEM_EVENT_FLAG_IP_ADDRESS, event_ip_address, conman);

  ubmodem_register_event_listener(conman->ub.modem,
      UBMODEM_EVENT_FLAG_FAILED_LEVEL_TRANSITION, event_failed_target_level,
      conman);

  ubmodem_register_event_listener(conman->ub.modem,
      UBMODEM_EVENT_FLAG_TARGET_LEVEL_REACHED, event_target_level, conman);

  ubmodem_register_event_listener(conman->ub.modem,
      UBMODEM_EVENT_FLAG_CALL_RINGING |
      UBMODEM_EVENT_FLAG_CALL_ACTIVE |
      UBMODEM_EVENT_FLAG_CALL_DISCONNECTED, event_call_status_change, conman);

#ifdef CONFIG_SYSTEM_CONMAN_DEBUG
  ubmodem_register_event_listener(conman->ub.modem,
      UBMODEM_EVENT_FLAG_NEW_LEVEL, event_new_level, conman);

  ubmodem_register_event_listener(conman->ub.modem,
      UBMODEM_EVENT_FLAG_TRACE_USRSOCK, event_trace_usrsock, conman);

  ubmodem_register_event_listener(conman->ub.modem,
      UBMODEM_EVENT_FLAG_TRACE_CMD_TO_MODEM, event_cmd_to_modem, conman);

  ubmodem_register_event_listener(conman->ub.modem,
      UBMODEM_EVENT_FLAG_TRACE_RESP_FROM_MODEM, event_resp_from_modem, conman);

  ubmodem_register_event_listener(conman->ub.modem,
      UBMODEM_EVENT_FLAG_TRACE_STATE_CHANGE, event_state_change, conman);

#ifdef CONFIG_SYSTEM_CONMAN_VERBOSE
  ubmodem_register_event_listener(conman->ub.modem,
      UBMODEM_EVENT_FLAG_TRACE_DATA_TO_MODEM, event_data_to_modem, conman);

  ubmodem_register_event_listener(conman->ub.modem,
      UBMODEM_EVENT_FLAG_TRACE_DATA_FROM_MODEM, event_data_from_modem, conman);
#endif
#endif

  ubmodem_request_level(conman->ub.modem, UBMODEM_LEVEL_POWERED_OFF);

  *maxfds = ubmodem_poll_max_fds(conman->ub.modem);

  return OK;
}

/****************************************************************************
 * Name: __conman_ubmodem_request_connection
 *
 * Description:
 *   Requests connection of type (request from the client) from the modem.
 *   Wifi will be added later.
 *
 * Input Parameters:
 *   conman  : connection manager handle
 *   type    : type of connections the client is requesting for
 *
 * Returned Value:
 *   OK    : no errors
 *   ERROR : failure
 *
 ****************************************************************************/

int __conman_ubmodem_request_connection(struct conman_s *conman,
    enum conman_connection_type_e type)
{
  enum ubmodem_func_level_e target_level;

  if (conman->connections.amount.cellular == 0)
    {
      conman_dbg("no cellular configs available\n");
      return ERROR;
    }

  switch (type)
    {
    case CONMAN_DATA:
    case CONMAN_2G:
      target_level = UBMODEM_LEVEL_GPRS;
    break;

    case CONMAN_SMS:
      target_level = UBMODEM_LEVEL_NETWORK;
    break;

    case CONMAN_NONE:
      target_level = UBMODEM_LEVEL_POWERED_OFF;
    break;

    default:
      ASSERT(true);
      return ERROR;
    break;
    }

  if (target_level == UBMODEM_LEVEL_POWERED_OFF)
    {
      /* Check refcount. */

      if (conman->connections.current.cellular_refcnt == 0)
        {
          conman_dbg("BUG BUG, cellular refcount == 0\n");

          if (conman->ub.target_level != UBMODEM_LEVEL_POWERED_OFF)
            {
              conman_dbg(" BUG BUG BUG BUG, and target_level != POWERED_OFF\n");
            }
          else
            {
              return OK;
            }
        }
      else
        {
          /* Reduce refcount. */

          conman->connections.current.cellular_refcnt--;

          conman_dbg("power off, refcount %d => %d\n",
                     conman->connections.current.cellular_refcnt + 1,
                     conman->connections.current.cellular_refcnt);
        }

      if (conman->connections.current.cellular_refcnt > 0)
        {
          /* Cellular still in-use by other instance, do not destroy
           * connection. */

          conman_dbg("do not power off, refcount == %d\n",
                     conman->connections.current.cellular_refcnt);

          return OK;
        }

      /* Already powered-off? */

      if (conman->ub.target_level == UBMODEM_LEVEL_POWERED_OFF)
        {
          conman_dbg("power off, already powered off (status: %d)\n",
                     conman->ub.status);

          if (conman->ub.status != CONMAN_STATUS_OFF)
            {
              conman_dbg("BUG? ub.status != OFF, but level == POWERED_OFF\n");
            }

          return OK;
        }

      /* Start destroying. */

      conman_dbg("power off, destroying...\n");

      conman->ub.status = CONMAN_STATUS_DESTROYING;
      conman->ub.target_level = UBMODEM_LEVEL_POWERED_OFF;
      ubmodem_request_level(conman->ub.modem, target_level);

      return OK;
    }
  else
    {
      /* Update refcount. */

      conman->connections.current.cellular_refcnt++;

      conman_dbg("conn request, refcount %d => %d\n",
                 conman->connections.current.cellular_refcnt - 1,
                 conman->connections.current.cellular_refcnt);

      if (conman->connections.current.cellular_refcnt > 1)
        {
          /* Cellular already active. */

          if (conman->ub.target_level < target_level)
            {
              /* But at lower level than requested. Upgrade connection
               * to higher level.
               */

              conman_dbg("conn request, upgrading level %d => %d\n",
                         conman->ub.target_level, target_level);

              conman->ub.status = CONMAN_STATUS_ESTABLISHING;
              conman->ub.target_level = target_level;
              ubmodem_request_level(conman->ub.modem, target_level);

              return OK;
            }
          else
            {
              /* Current level is same or higher. */

              conman_dbg("conn request, current level %d, wanted %d\n",
                         conman->ub.target_level, target_level);

              return OK;
            }
        }
      else
        {
          conman_dbg("conn request, level %d => %d\n",
                     conman->ub.target_level, target_level);

          /* Cellular should be powered-off. */

          if (conman->ub.target_level != UBMODEM_LEVEL_POWERED_OFF)
            {
              conman_dbg("BUG? refcount == 1, but level != POWERED_OFF\n");
            }

          conman->ub.status = CONMAN_STATUS_ESTABLISHING;

          conman->ub.target_level = target_level;
          ubmodem_request_level(conman->ub.modem, target_level);

          return OK;
        }
    }
}

/****************************************************************************
 * Name: __conman_ubmodem_get_status_connection
 *
 * Description:
 *  Get status for the current modem connection.
 *
 * Input Parameters:
 *   conman  : connection manager handle
 *   status  : pointer to status structure
 *
 * Returned Value:
 *   OK    : no errors
 *   ERROR : failure
 *
 ****************************************************************************/

int __conman_ubmodem_get_status_connection(struct conman_s *conman,
                                           struct conman_status_s *status)
{
  status->conn_type = CONMAN_2G;
  status->status = conman->ub.status;
  status->info_type = CONMAN_INFO_CELLULAR;

  memset(&status->info.cellu, 0, sizeof(status->info.cellu));

  if (status->status == CONMAN_STATUS_ESTABLISHED)
    {
      if (conman->ub.imei_requested || conman->ub.imsi_requested || conman->ub.udopn_requested)
        status->status = CONMAN_STATUS_ESTABLISHING;

      if (conman->ub.target_level == UBMODEM_LEVEL_GPRS)
        {
          status->info.cellu.ipaddr = conman->ub.ipaddr;
        }

      if (status->status == CONMAN_STATUS_ESTABLISHED)
        {
          snprintf(status->info.cellu.imei, sizeof(status->info.cellu.imei), "%s", conman->ub.imei);
          snprintf(status->info.cellu.imsi, sizeof(status->info.cellu.imsi), "%s", conman->ub.imsi);
          snprintf(status->info.cellu.oper_name, sizeof(status->info.cellu.oper_name), "%s", conman->ub.udopn);
        }

      __conman_send_resp(conman, CONMAN_MSG_GET_CONNECTION_STATUS,
                         CONMAN_RESP_OK, status, sizeof(*status));

      return EINPROGRESS;
    }

  return OK;
}

/****************************************************************************
 * Name: __conman_ubmodem_is_destroying
 *
 * Description:
 *  Check if destroying previous connection
 *
 * Input Parameters:
 *   conman  : connection manager handle
 *
 * Returned Value:
 *   true if still destroying
 *
 ****************************************************************************/

bool __conman_ubmodem_is_destroying(struct conman_s *conman)
{
  return (conman->ub.status == CONMAN_STATUS_DESTROYING);
}

/****************************************************************************
 * Name: __conman_ubmodem_send_sms
 *
 * Description:
 *  Send SMS with 2G modem.
 *
 * Input Parameters:
 *   conman  : connection manager handle
 *   sms     : message structure, with receiver phone number and SMS message
 *             body
 *
 * Returned Value:
 *   OK if successfully queued for sending.
 *   Negated error code if case of error.
 ****************************************************************************/

int __conman_ubmodem_send_sms(struct conman_s *conman,
                              struct conman_msg_send_sms_s *sms)
{
  const char *receiver = &sms->sms_data[0];
  const char *message = &sms->sms_data[sms->receiver_len];
  struct sms_queue_entry_s *item;
  bool queue_empty;
  int ret;

  /* Confirm that receiver and message buffers contain null-terminated strings,
   * without extra nulls inside.
   */
  DEBUGASSERT(strnlen(receiver, sms->receiver_len) == sms->receiver_len - 1);
  DEBUGASSERT(strnlen(message, sms->message_len) == sms->message_len - 1);

  conman_dbg("receiver = '%s', message = '%s'\n", receiver, message);

  item = calloc(1, sizeof(*item));
  if (!item)
    {
      free(sms);
      errno = -ENOMEM;
      return ERROR;
    }

  item->sms = sms;
  item->receiver = receiver;
  item->message = message;

  /* Add to SMS queue. */

  queue_empty = sq_peek(&conman->ub.sms_queue) == NULL;

  sq_addlast(&item->node, &conman->ub.sms_queue);
  conman_dbg("message queued...\n");

  if (!queue_empty)
    {
      /* Sanity checks. */

      if (conman->ub.status != CONMAN_STATUS_ESTABLISHED)
        {
          if (!conman->ub.sms_connection_requested)
            {
              conman_dbg("SMS send state error, SMS queue was not empty, "
                         "connection not ESTABLISHED (but %d) and not SMS "
                         "connection requested.\n", conman->ub.status);
            }
        }
      else
        {
          if (!conman->ub.sms_sending_active)
            {
              conman_dbg("SMS send state error, SMS queue was not empty, "
                         "connection ESTABLISHED and SMS sending not "
                         "active.\n");
            }
        }
    }

  if (!conman->ub.sms_connection_requested)
    {
      conman_dbg("Request SMS connection.\n");
      DEBUGASSERT(conman->ub.sms_connid == CONMAN_CONNID_CLEAR);

      /* Queue message and request connection. */

      ret = __conman_ctl_create_connection(conman, CONMAN_SMS,
                                           &conman->ub.sms_connid);
      if (ret < 0)
        {
          conman_dbg("__conman_ctl_create_connection(CONMAN_SMS) failed!\n");
          /* Do not free 'sms', already in queue. */
          return OK;
        }

      conman->ub.sms_connection_requested = true;
      conman_dbg("SMS connection requested\n");
    }

  if (conman->ub.status == CONMAN_STATUS_ESTABLISHED &&
      !conman->ub.sms_sending_active)
    {
      /* Already connected. Start processing SMS queue. */

      ret = process_sms_queue(conman);
      if (ret < 0)
        {
          conman_dbg("process_sms_queue failed: %d!\n", ret);
          /* Do not free 'sms', already in queue. */
          return OK;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: __conman_ubmodem_call_answer
 *
 * Description:
 *   Answer pending voice-call with 2G modem.
 *
 * Input Parameters:
 *   conman  : connection manager handle
 *   mute    : Speaker/mic muting configuration
 *
 * Returned Value:
 *   OK if successfully queued for sending.
 *   Negated error code if case of error.
 ****************************************************************************/

int __conman_ubmodem_call_answer(struct conman_s *conman,
                                 const struct conman_msg_call_answer_s *mute)
{
  if (conman->ub.status != CONMAN_STATUS_ESTABLISHED)
    {
      return ERROR;
    }

#ifdef CONFIG_UBMODEM_VOICE
  ubmodem_voice_answer(conman->ub.modem, mute->mic_mute, mute->speaker_mute);
  return OK;
#else
  return ERROR;
#endif
}

/****************************************************************************
 * Name: __conman_ubmodem_call_hangup
 *
 * Description:
 *   Hangup pending or active voice-call with 2G modem.
 *
 * Input Parameters:
 *   conman  : connection manager handle
 *   mute    : Speaker/mic muting configuration
 *
 * Returned Value:
 *   OK if successfully queued for sending.
 *   Negated error code if case of error.
 ****************************************************************************/

int __conman_ubmodem_call_hangup(struct conman_s *conman)
{
  if (conman->ub.status != CONMAN_STATUS_ESTABLISHED)
    {
      return ERROR;
    }

#ifdef CONFIG_UBMODEM_VOICE
  ubmodem_voice_hangup(conman->ub.modem);
  return OK;
#else
  return ERROR;
#endif
}

/****************************************************************************
 * Name: __conman_ubmodem_call_audioctl
 *
 * Description:
 *   Control audio output/input for modem.
 *
 * Input Parameters:
 *   conman  : connection manager handle
 *   ctl     : audio controls
 *
 * Returned Value:
 *   OK if successfully queued for sending.
 *   Negated error code if case of error.
 ****************************************************************************/

int __conman_ubmodem_call_audioctl(struct conman_s *conman,
                                   const struct conman_msg_call_audioctl_s *ctl)
{
  if (conman->ub.status != CONMAN_STATUS_ESTABLISHED)
    {
      return ERROR;
    }

#ifdef CONFIG_UBMODEM_VOICE
  ubmodem_audio_setup(conman->ub.modem, ctl->audio_out_on, false);
  return OK;
#else
  return ERROR;
#endif
}

/****************************************************************************
 * Name: __conman_ubmodem_setup_pollfds
 ****************************************************************************/

void __conman_ubmodem_setup_pollfds(struct conman_s *conman,
                                    struct pollfd *pfds, int maxfds,
                                    int *fds_pos, int *min_timeout)
{
  int timeout;
  int ret;

  if (!conman->ub.modem)
    {
      return;
    }

  ret = ubmodem_pollfds_setup(conman->ub.modem, pfds + *fds_pos,
                              maxfds - *fds_pos,
                              &timeout);
  if (ret < 0)
    {
      return;
    }

  *fds_pos += ret;
  if (timeout < *min_timeout)
    {
      *min_timeout = timeout;
    }
}

/****************************************************************************
 * Name: __conman_ubmodem_poll_timedout
 ****************************************************************************/

void __conman_ubmodem_poll_timedout(struct conman_s *conman)
{
  if (!conman->ub.modem)
    {
      return;
    }

  (void)ubmodem_poll_timedout(conman->ub.modem);
}

/****************************************************************************
 * Name: __conman_ubmodem_handle_pollfds
 ****************************************************************************/

void __conman_ubmodem_handle_pollfds(struct conman_s *conman,
                                     struct pollfd *pfds, int npfds)
{
  if (!conman->ub.modem)
    {
      return;
    }

  (void)ubmodem_pollfds_event(conman->ub.modem, pfds, npfds);
}
