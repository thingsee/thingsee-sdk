/****************************************************************************
 * apps/system/conman/conman_ubmodem.c
 *
 *   Copyright (C) 2015-2016 Haltian Ltd. All rights reserved.
 *   Authors: Pekka Ervasti <pekka.ervasti@haltian.com>
 *            Sila Kayo <sila.kayo@haltian.com>
 *            Jussi Kivilinna <jussi.kivilinna@haltian.com>
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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_UBMODEM_SMS_ENABLED
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
#else
static int process_sms_queue(struct conman_s *conman)
{
  (void)conman;
  return OK;
}
#endif

static void event_ip_address(struct ubmodem_s *modem,
    enum ubmodem_event_flags_e event, const void *event_data, size_t datalen,
    void *priv)
{
  const struct ubmodem_event_ip_address_s *ipcfg = event_data;
  struct conman_s *conman = priv;
  uint32_t addr = ntohl(ipcfg->ipaddr.s_addr);
  uint32_t dns1 = ntohl(ipcfg->dns1.s_addr);
  uint32_t dns2 = ntohl(ipcfg->dns2.s_addr);

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
      const struct in_addr *pdns1;
      const struct in_addr *pdns2;
      uint8_t b;

      getrandom(&b, 1);

      pdns1 = (b & 1) ? &ipcfg->dns1 : &ipcfg->dns2;
      pdns2 = (b & 1) ? &ipcfg->dns2 : &ipcfg->dns1;

      dns_setservers(pdns1, pdns2, NULL);
    }
  else if (dns1 != 0)
    {
      dns_setservers(&ipcfg->dns1, NULL, NULL);
    }
  else if (dns2 != 0)
    {
      dns_setservers(&ipcfg->dns2, NULL, NULL);
    }
  else
    {
      const struct in_addr *pdns1;
      const struct in_addr *pdns2;
      struct in_addr gdns1;
      struct in_addr gdns2;
      uint8_t b;

      /* No valid IP address? Try Google's DNS.
       * Should we try offer these also above, as third DNS server? */

      gdns1.s_addr = inet_addr("8.8.8.8");
      gdns2.s_addr = inet_addr("8.8.4.4");

      getrandom(&b, 1);

      pdns1 = (b & 1) ? &gdns1 : &gdns2;
      pdns2 = (b & 1) ? &gdns2 : &gdns1;

      dns_setservers(pdns1, pdns2, NULL);
    }

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

  /* Generate event if lost established connection or failed to open. */

  if (conman->ub.target_level != UBMODEM_LEVEL_POWERED_OFF)
    {
      if ((conman->ub.establishing_lost &&
           conman->ub.status == CONMAN_STATUS_ESTABLISHING) ||
          conman->ub.status == CONMAN_STATUS_ESTABLISHED)
        {
          __conman_send_boardcast_event(conman,
                                        CONMAN_EVENT_LOST_CONNECTION,
                                        NULL, 0);
        }
      else if (conman->ub.status == CONMAN_STATUS_ESTABLISHING)
        {
          __conman_send_boardcast_event(conman,
                                        CONMAN_EVENT_CONNECTION_REQUEST_FAILED,
                                        NULL, 0);
        }
    }

  conman->ub.info_requested = false;
  conman->ub.establishing_lost = false;
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

        up_rngaddentropy((const uint32_t *)conman->ub.imei,
                         sizeof(conman->ub.imei) / sizeof(uint32_t));
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

        up_rngaddentropy((const uint32_t *)conman->ub.imsi,
                         sizeof(conman->ub.imsi) / sizeof(uint32_t));
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

        up_rngaddentropy((const uint32_t *)conman->ub.udopn,
                         sizeof(conman->ub.udopn) / sizeof(uint32_t));
      }
    break;
  case UB_INFO_MCC_MNC:
    conman->ub.mcc_mnc_requested = false;
    if (!status)
      {
        memset(conman->ub.udopn, 0, sizeof(conman->ub.mcc_mnc));
        conman_dbg("Received failed status for MCC_MNC\n");
      }
    else
      {
        snprintf(conman->ub.mcc_mnc, sizeof(conman->ub.mcc_mnc), "%s", data);
        conman_dbg("MCC_MNC received: %s\n", conman->ub.mcc_mnc);

        up_rngaddentropy((const uint32_t *)conman->ub.mcc_mnc,
                         sizeof(conman->ub.mcc_mnc) / sizeof(uint32_t));
      }
    break;
  default:
    break;
  }

  if (conman->ub.imei_requested || conman->ub.imsi_requested ||
      conman->ub.udopn_requested || conman->ub.mcc_mnc_requested)
    {
      return;
    }

  conman_dbg("reached target level (status: %d => %d)\n",
             conman->ub.status, CONMAN_STATUS_ESTABLISHED);

  conman->ub.establishing_lost = false;
  conman->ub.status = CONMAN_STATUS_ESTABLISHED;
  process_sms_queue(conman);

  __conman_send_boardcast_event(conman, CONMAN_EVENT_CONNECTION_ESTABLISHED,
                                NULL, 0);
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

      if (!conman->ub.mcc_mnc_requested)
        {
          conman_dbg("Requesting MCC_MNC\n");
          if (ubmodem_get_info(conman->ub.modem, ubmodem_info_callback, priv, UB_INFO_MCC_MNC) == OK)
            conman->ub.mcc_mnc_requested = true;
          else
            conman_dbg("MCC_MNC request failed!\n");
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

            conman->ub.establishing_lost = false;
            conman->ub.status = CONMAN_STATUS_DESTROYING;
            conman->ub.target_level = UBMODEM_LEVEL_POWERED_OFF;
            ubmodem_request_level(modem, conman->ub.target_level);
          }
        else
          {
            conman_dbg("reached target level (status: %d => %d)\n",
                       conman->ub.status, CONMAN_STATUS_OFF);

            conman->ub.establishing_lost = false;
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

            conman->ub.establishing_lost =
                (conman->ub.status == CONMAN_STATUS_ESTABLISHED);
            conman->ub.status = CONMAN_STATUS_ESTABLISHING;
            ubmodem_request_level(modem, conman->ub.target_level);
          }
        else if (!conman->ub.info_requested || conman->ub.imei_requested ||
                  conman->ub.imsi_requested || conman->ub.udopn_requested ||
                  conman->ub.mcc_mnc_requested)
          {
            conman_dbg("info not yet requested, retry... (status: %d)\n",
                       conman->ub.status);

            conman->ub.status = CONMAN_STATUS_ESTABLISHING;
          }
        else
          {
            conman_dbg("reached target level (status: %d => %d)\n",
                       conman->ub.status, CONMAN_STATUS_ESTABLISHED);

            conman->ub.establishing_lost = false;
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

            conman->ub.establishing_lost = false;
            conman->ub.status = CONMAN_STATUS_DESTROYING;
            conman->ub.target_level = UBMODEM_LEVEL_POWERED_OFF;
            ubmodem_request_level(modem, conman->ub.target_level);
          }
        else
          {
            conman_dbg("reached target level (status: %d => %d)\n",
                       conman->ub.status, CONMAN_STATUS_OFF);

            conman->ub.establishing_lost = false;
            conman->ub.status = CONMAN_STATUS_OFF;
            conman->ub.info_requested = false;
          }
      }
      break;
    }
}

#ifdef CONFIG_UBMODEM_VOICE
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
#endif

#ifndef CONFIG_UBMODEM_DISABLE_CELLLOCATE
static void event_celllocate(struct ubmodem_s *modem,
                             enum ubmodem_event_flags_e event,
                             const void *event_data, size_t datalen,
                             void *priv)
{
  struct conman_s *conman = priv;
  const struct ubmodem_event_cell_location_s *cloc;
  struct conman_event_celllocate_info clinfo = {};
  struct tm t;

  conman_dbg("Got CellLocate event!\n");

  DEBUGASSERT(datalen == sizeof(*cloc));
  cloc = event_data;

  /* Fill location info. */

  if (cloc->location.valid)
    {
      clinfo.have_location = true;
      clinfo.altitude = cloc->location.altitude;
      clinfo.accuracy = cloc->location.accuracy;
      clinfo.latitude = cloc->location.latitude;
      clinfo.longitude = cloc->location.longitude;
    }

  if (cloc->date.valid)
    {
      clinfo.have_time = true;

      /* Prepare time structure. */

      t.tm_year = cloc->date.year - 1900;
      t.tm_mon = cloc->date.month - 1;
      t.tm_mday = cloc->date.day;
      t.tm_hour = cloc->date.hour;
      t.tm_min = cloc->date.min;
      t.tm_sec = cloc->date.sec;

      /* Store GPS time. */

      clinfo.gps_time = mktime(&t);
    }

  __conman_send_boardcast_event(conman, CONMAN_EVENT_CELLLOCATE,
                                &clinfo, sizeof(clinfo));
}
#endif

#ifndef CONFIG_UBMODEM_DISABLE_CELL_ENVIRONMENT
static void event_cell_environment(struct ubmodem_s *modem,
                             enum ubmodem_event_flags_e event,
                             const void *event_data, size_t datalen,
                             void *priv)
{
  struct conman_s *conman = priv;
  const struct ubmodem_event_cell_environment_s *ubenv;
  struct conman_event_cell_environment_s *cenv;
  size_t cenv_len;
  unsigned int i;

  conman_dbg("Got cell environment event!\n");

  DEBUGASSERT(datalen >= sizeof(*ubenv));
  ubenv = event_data;
  DEBUGASSERT(datalen ==
      sizeof(*ubenv) + ubenv->num_neighbors * sizeof(ubenv->neighbors[0]));

  cenv_len = sizeof(*cenv) + ubenv->num_neighbors * sizeof(cenv->neighbors[0]);
  cenv = calloc(1, cenv_len);
  if (!cenv)
    {
      return;
    }

  cenv->have_signal_qual = ubenv->have_signal_qual;
  cenv->have_serving = ubenv->have_serving;
  cenv->num_neighbors = ubenv->num_neighbors;

  if (cenv->have_signal_qual)
    {
      cenv->signal_qual.qual = ubenv->signal_qual.qual;
      cenv->signal_qual.rssi = ubenv->signal_qual.rssi;
    }

  if (cenv->have_serving)
    {
      cenv->serving.cell_id = ubenv->serving.cell_id;
      cenv->serving.mcc = ubenv->serving.mcc;
      cenv->serving.mnc = ubenv->serving.mnc;
      cenv->serving.lac = ubenv->serving.lac;
      cenv->serving.bsic = ubenv->serving.bsic;
      cenv->serving.arfcn = ubenv->serving.arfcn;
      cenv->serving.signal_dbm = ubenv->serving.signal_dbm;
      switch (ubenv->serving.rat)
        {
        case UBMODEM_RAT_GSM:
          cenv->serving.type = CONMAN_CELL_ENVIRONMENT_TYPE_GSM;
          cenv->serving.sc = 0xffff;
          break;
        case UBMODEM_RAT_UMTS:
          cenv->serving.type = CONMAN_CELL_ENVIRONMENT_TYPE_UMTS;
          cenv->serving.sc = ubenv->serving.sc;
          break;
        default:
          break;
        }
    }

  for (i = 0; i < cenv->num_neighbors; i++)
    {
      cenv->neighbors[i].have_mcc_mnc_lac = ubenv->neighbors[i].have_mcc_mnc_lac;
      cenv->neighbors[i].have_cellid = ubenv->neighbors[i].have_cellid;
      cenv->neighbors[i].have_bsic = ubenv->neighbors[i].have_bsic;
      cenv->neighbors[i].have_arfcn = ubenv->neighbors[i].have_arfcn;
      cenv->neighbors[i].have_signal_dbm = ubenv->neighbors[i].have_signal_dbm;

      switch (ubenv->neighbors[i].rat)
        {
        case UBMODEM_RAT_GSM:
          cenv->neighbors[i].type = CONMAN_CELL_ENVIRONMENT_TYPE_GSM;
          cenv->neighbors[i].have_sc = false;
          break;
        case UBMODEM_RAT_UMTS:
          cenv->neighbors[i].type = CONMAN_CELL_ENVIRONMENT_TYPE_UMTS;
          cenv->neighbors[i].have_sc = ubenv->neighbors[i].have_sc;
          break;
        default:
          break;
        }

      if (cenv->neighbors[i].have_mcc_mnc_lac)
        {
          cenv->neighbors[i].mcc = ubenv->neighbors[i].mcc;
          cenv->neighbors[i].mnc = ubenv->neighbors[i].mnc;
          cenv->neighbors[i].lac = ubenv->neighbors[i].lac;
        }
      else
        {
          cenv->neighbors[i].mcc = 0xffff;
          cenv->neighbors[i].mnc = 0xffff;
          cenv->neighbors[i].lac = 0xffff;
        }

      if (cenv->neighbors[i].have_cellid)
        {
          cenv->neighbors[i].cell_id = ubenv->neighbors[i].cell_id;
        }
      else
        {
          cenv->neighbors[i].cell_id = 0xffffffff;
        }

      if (cenv->neighbors[i].have_bsic)
        {
          cenv->neighbors[i].bsic = ubenv->neighbors[i].bsic;
        }
      else
        {
          cenv->neighbors[i].bsic = 0xff;
        }

      if (cenv->neighbors[i].have_arfcn)
        {
          cenv->neighbors[i].arfcn = ubenv->neighbors[i].arfcn;
        }
      else
        {
          cenv->neighbors[i].arfcn = 0xffff;
        }

      if (cenv->neighbors[i].have_signal_dbm)
        {
          cenv->neighbors[i].signal_dbm = ubenv->neighbors[i].signal_dbm;
        }
      else
        {
          cenv->neighbors[i].signal_dbm = INT16_MIN;
        }

      if (cenv->neighbors[i].have_sc)
        {
          cenv->neighbors[i].sc = ubenv->neighbors[i].sc;
        }
      else
        {
          cenv->neighbors[i].sc = 0xffff;
        }
    }

  __conman_send_boardcast_event(conman, CONMAN_EVENT_CELL_ENVIRONMENT,
                                cenv, cenv_len);

  free(cenv);
}
#endif

#ifdef CONFIG_UBMODEM_FTP_ENABLED
static void event_ftp_download_status(struct ubmodem_s *modem,
                             enum ubmodem_event_flags_e event,
                             const void *event_data, size_t datalen,
                             void *priv)
{
  struct conman_s *conman = priv;
  const struct ubmodem_event_ftp_download_status_s *uftpds;
  struct conman_event_ftp_download_status cftpds = {};

  conman_dbg("Got FTP download status event!\n");

  DEBUGASSERT(datalen == sizeof(*uftpds));
  uftpds = event_data;

  cftpds.file_downloaded = uftpds->file_downloaded;

  __conman_send_boardcast_event(conman, CONMAN_EVENT_FTP_DOWNLOAD_STATUS,
                                &cftpds, sizeof(cftpds));
}
#endif

static void event_error(struct ubmodem_s *modem,
                        enum ubmodem_event_flags_e event,
                        const void *event_data, size_t datalen,
                        void *priv)
{
  const enum ubmodem_error_event_e *error;
  struct conman_s *conman = priv;

  DEBUGASSERT(datalen >= sizeof(*error));
  error = event_data;

  switch (*error)
    {
      case UBMODEM_ERROR_STARTUP_VOLTAGE_TOO_LOW:
        conman_dbg("Got %s error event!\n",
                   "UBMODEM_ERROR_STARTUP_VOLTAGE_TOO_LOW");

        __conman_send_boardcast_event(conman,
                                      CONMAN_EVENT_HW_ERROR_TOO_LOW_VOLTAGE,
                                      NULL, 0);

        break;

      default:
        conman_dbg("Got <%d> error event!\n", *error);
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

static void ubmodem_event_cb(struct ubmodem_s *modem,
                             enum ubmodem_event_flags_e event,
                             const void *event_data, size_t datalen,
                             void *priv)
{
  switch (event)
    {
      default:
        return;

      case UBMODEM_EVENT_FLAG_IP_ADDRESS:
        event_ip_address(modem, event, event_data, datalen, priv);
        return;

      case UBMODEM_EVENT_FLAG_FAILED_LEVEL_TRANSITION:
        event_failed_target_level(modem, event, event_data, datalen, priv);
        return;

      case UBMODEM_EVENT_FLAG_TARGET_LEVEL_REACHED:
        event_target_level(modem, event, event_data, datalen, priv);
        return;

#ifdef CONFIG_UBMODEM_VOICE
      case UBMODEM_EVENT_FLAG_CALL_RINGING:
      case UBMODEM_EVENT_FLAG_CALL_ACTIVE:
      case UBMODEM_EVENT_FLAG_CALL_DISCONNECTED:
        event_call_status_change(modem, event, event_data, datalen, priv);
        return;
#endif

#ifndef CONFIG_UBMODEM_DISABLE_CELLLOCATE
      case UBMODEM_EVENT_FLAG_CELL_LOCATION:
        event_celllocate(modem, event, event_data, datalen, priv);
        return;
#endif

#ifndef CONFIG_UBMODEM_DISABLE_CELL_ENVIRONMENT
      case UBMODEM_EVENT_FLAG_CELL_ENVIRONMENT:
        event_cell_environment(modem, event, event_data, datalen, priv);
        return;
#endif

#ifdef CONFIG_UBMODEM_FTP_ENABLED
      case UBMODEM_EVENT_FLAG_FTP_DOWNLOAD_STATUS:
        event_ftp_download_status(modem, event, event_data, datalen, priv);
        return;
#endif

      case UBMODEM_EVENT_FLAG_ERROR:
        event_error(modem, event, event_data, datalen, priv);
        return;

#ifdef CONFIG_SYSTEM_CONMAN_DEBUG
      case UBMODEM_EVENT_FLAG_NEW_LEVEL:
        event_new_level(modem, event, event_data, datalen, priv);
        return;

      case UBMODEM_EVENT_FLAG_TRACE_USRSOCK:
        event_trace_usrsock(modem, event, event_data, datalen, priv);
        return;

      case UBMODEM_EVENT_FLAG_TRACE_CMD_TO_MODEM:
        event_cmd_to_modem(modem, event, event_data, datalen, priv);
        return;

      case UBMODEM_EVENT_FLAG_TRACE_RESP_FROM_MODEM:
        event_resp_from_modem(modem, event, event_data, datalen, priv);
        return;

      case UBMODEM_EVENT_FLAG_TRACE_STATE_CHANGE:
        event_state_change(modem, event, event_data, datalen, priv);
        return;

#ifdef CONFIG_SYSTEM_CONMAN_VERBOSE
      case UBMODEM_EVENT_FLAG_TRACE_DATA_TO_MODEM:
        event_data_to_modem(modem, event, event_data, datalen, priv);
        return;

      case UBMODEM_EVENT_FLAG_TRACE_DATA_FROM_MODEM:
        event_data_from_modem(modem, event, event_data, datalen, priv);
        return;
#endif
#endif
    }
}

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

static void ubmodem_hw_pm_set_activity(void *priv,
                                       enum ubmodem_hw_pm_activity_e type,
                                       bool set)
{
#ifdef BOARD_HAS_SET_MODEM_ACTIVITY
  enum e_board_modem_activity btype;

  switch (type)
    {
    default:
    case UBMODEM_PM_ACTIVITY_LOW:
      btype = BOARD_MODEM_ACTIVITY_LOW;
      break;
    case UBMODEM_PM_ACTIVITY_HIGH:
      btype = BOARD_MODEM_ACTIVITY_HIGH;
      break;
    }

  board_set_modem_activity(btype, set);
#endif
}

static const struct ubmodem_hw_ops_s modem_hw_ops =
  {
      .initialize = ubmodem_hw_init,
      .deinitialize = ubmodem_hw_deinit,
      .vcc_set = ubmodem_hw_vcc_set,
      .reset_pin_set = ubmodem_hw_reset_pin_set,
      .poweron_pin_set = ubmodem_hw_poweron_pin_set,
      .pm_set_activity = ubmodem_hw_pm_set_activity
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
#ifdef CONFIG_SYSTEM_CONMAN_DEBUG
      UBMODEM_EVENT_FLAG_NEW_LEVEL |
      UBMODEM_EVENT_FLAG_TRACE_USRSOCK |
      UBMODEM_EVENT_FLAG_TRACE_CMD_TO_MODEM |
      UBMODEM_EVENT_FLAG_TRACE_RESP_FROM_MODEM |
      UBMODEM_EVENT_FLAG_TRACE_STATE_CHANGE |
#ifdef CONFIG_SYSTEM_CONMAN_VERBOSE
      UBMODEM_EVENT_FLAG_TRACE_DATA_TO_MODEM |
      UBMODEM_EVENT_FLAG_TRACE_DATA_FROM_MODEM |
#endif
#endif
      UBMODEM_EVENT_FLAG_IP_ADDRESS |
      UBMODEM_EVENT_FLAG_FAILED_LEVEL_TRANSITION |
      UBMODEM_EVENT_FLAG_TARGET_LEVEL_REACHED |
#ifdef CONFIG_UBMODEM_VOICE
      UBMODEM_EVENT_FLAG_CALL_RINGING |
      UBMODEM_EVENT_FLAG_CALL_ACTIVE |
      UBMODEM_EVENT_FLAG_CALL_DISCONNECTED |
#endif
      UBMODEM_EVENT_FLAG_ERROR |
#ifdef CONFIG_UBMODEM_FTP_ENABLED
      UBMODEM_EVENT_FLAG_FTP_DOWNLOAD_STATUS |
#endif
#ifndef CONFIG_UBMODEM_DISABLE_CELLLOCATE
      UBMODEM_EVENT_FLAG_CELL_LOCATION |
#endif
#ifndef CONFIG_UBMODEM_DISABLE_CELL_ENVIRONMENT
      UBMODEM_EVENT_FLAG_CELL_ENVIRONMENT |
#endif
      0, ubmodem_event_cb, conman);

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
      if (conman->ub.imei_requested || conman->ub.imsi_requested ||
          conman->ub.udopn_requested || conman->ub.mcc_mnc_requested)
        {
          status->status = CONMAN_STATUS_ESTABLISHING;
        }

      if (conman->ub.target_level == UBMODEM_LEVEL_GPRS)
        {
          status->info.cellu.ipaddr = conman->ub.ipaddr;
        }

      if (status->status == CONMAN_STATUS_ESTABLISHED)
        {
          snprintf(status->info.cellu.imei, sizeof(status->info.cellu.imei), "%s", conman->ub.imei);
          snprintf(status->info.cellu.imsi, sizeof(status->info.cellu.imsi), "%s", conman->ub.imsi);
          snprintf(status->info.cellu.oper_name, sizeof(status->info.cellu.oper_name), "%s", conman->ub.udopn);
          snprintf(status->info.cellu.mcc_mnc, sizeof(status->info.cellu.mcc_mnc), "%s", conman->ub.mcc_mnc);
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
#ifdef CONFIG_UBMODEM_SMS_ENABLED
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
#else
  free(sms);
  errno = -ENOSYS;
  return ERROR;
#endif
}

/****************************************************************************
 * Name: __conman_ubmodem_request_cell_environment
 *
 * Description:
 *   Request for cell environment (serving & neighbor cell-ids, signal levels,
 *   etc).
 *
 * Input Parameters:
 *   conman  : connection manager handle
 *
 * Returned Value:
 *   OK if successfully queued for sending.
 *   Negated error code if case of error.
 ****************************************************************************/

int __conman_ubmodem_request_cell_environment(struct conman_s *conman)
{
#ifndef CONFIG_UBMODEM_DISABLE_CELL_ENVIRONMENT
  if (conman->ub.status != CONMAN_STATUS_ESTABLISHED &&
      conman->ub.status != CONMAN_STATUS_ESTABLISHING)
    {
      return ERROR;
    }

  return ubmodem_request_cell_environment(conman->ub.modem);
#else
  return ERROR;
#endif
}

/****************************************************************************
 * Name: __conman_ubmodem_filesystem_delete
 *
 * Description:
 *  Delete file in modem filesystem
 *
 * Input Parameters:
 *   conman  : connection manager handle
 *   filename: Name of file to delete
 *
 * Returned Value:
 *   OK if successfully queued for sending.
 *   Negated error code if case of error.
 ****************************************************************************/

int __conman_ubmodem_filesystem_delete(struct conman_s *conman,
                                       const char *filename)
{
#ifndef CONFIG_UBMODEM_DISABLE_FILESYSTEM
  return ubmodem_filesystem_delete(conman->ub.modem, filename);
#else
  return ERROR;
#endif
}

/****************************************************************************
 * Name: __conman_ubmodem_ftp_download
 *
 * Description:
 *  Retrieve a file from FTP server
 *
 * Input Parameters:
 *   conman  : connection manager handle
 *   ftp     : parameters configuration for the FTP server connection
 *
 * Returned Value:
 *   OK if successfully queued for sending.
 *   Negated error code if case of error.
 ****************************************************************************/

#ifdef CONFIG_UBMODEM_FTP_ENABLED
int __conman_ubmodem_ftp_download(struct conman_s *conman,
                                  struct conman_msg_ftp_download_s *ftp)
{
  struct ubmodem_ftp_download_s ubftp = {};
  const char *hostname = &ftp->msg_data[0];
  const char *username = &hostname[ftp->hostname_len];
  const char *password = &username[ftp->username_len];
  const char *filepath_src = &password[ftp->password_len];
  const char *filepath_dst = &filepath_src[ftp->filepath_src_len];

  /* Confirm that buffers contain null-terminated strings, without extra nulls
   * inside.
   */
  DEBUGASSERT(strnlen(hostname, ftp->hostname_len) == ftp->hostname_len - 1);
  DEBUGASSERT(strnlen(username, ftp->username_len) == ftp->username_len - 1);
  DEBUGASSERT(strnlen(password, ftp->password_len) == ftp->password_len - 1);
  DEBUGASSERT(strnlen(filepath_src, ftp->filepath_src_len) == ftp->filepath_src_len - 1);
  DEBUGASSERT(strnlen(filepath_dst, ftp->filepath_dst_len) == ftp->filepath_dst_len - 1);

  ubftp.hostname = hostname;
  ubftp.username = username;
  ubftp.password = password;
  ubftp.filepath_src = filepath_src;
  ubftp.filepath_dst = filepath_dst;

  return ubmodem_ftp_download_file(conman->ub.modem, &ubftp, NULL);
}
#else
int __conman_ubmodem_ftp_download(struct conman_s *conman,
                                  struct conman_msg_ftp_download_s *ftp)
{
  return ERROR;
}
#endif /* CONFIG_UBMODEM_FTP_ENABLED */

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
 * Name: __conman_ubmodem_play_audio_resource
 *
 * Description:
 *   Play modem audio resource.
 *
 * Input Parameters:
 *   conman  : connection manager handle
 *   ctl     : audio resource
 *
 * Returned Value:
 *   OK if successfully queued for sending.
 *   Negated error code if case of error.
 ****************************************************************************/

#ifndef CONFIG_UBMODEM_VOICE
int __conman_ubmodem_play_audio_resource(struct conman_s *conman,
    const struct conman_msg_play_audio_resource_s *ctl)
{
  return ERROR;
}
#else
int __conman_ubmodem_play_audio_resource(struct conman_s *conman,
    const struct conman_msg_play_audio_resource_s *ctl)
{
  int ret = OK;

  if (conman->ub.status != CONMAN_STATUS_ESTABLISHED)
    {
      return ERROR;
    }

  switch (ctl->resource.type)
    {
    default:
    case CONMAN_AUDIO_RESOURCE_TYPE_STOP:
      {
        unsigned int audio_resource = 0; /* tone generator */

        ret = ubmodem_stop_audio_resource(conman->ub.modem, audio_resource);
      }
      break;

    case CONMAN_AUDIO_RESOURCE_TYPE_TONE_ID:
      {
        /* See u-blox AT Commands manual, "AT+UPAR" */

        unsigned int audio_resource = 0; /* tone generator */
        unsigned int tone_id = ctl->resource.tone_id.tone_id;

        /* Play audio. */

        ret = ubmodem_play_audio_resource(conman->ub.modem, audio_resource,
                                          tone_id,
                                          ctl->resource.tone_id.nrepetitions);
      }
      break;

    case CONMAN_AUDIO_RESOURCE_TYPE_DTMF_0:
    case CONMAN_AUDIO_RESOURCE_TYPE_DTMF_1:
    case CONMAN_AUDIO_RESOURCE_TYPE_DTMF_2:
    case CONMAN_AUDIO_RESOURCE_TYPE_DTMF_3:
    case CONMAN_AUDIO_RESOURCE_TYPE_DTMF_4:
    case CONMAN_AUDIO_RESOURCE_TYPE_DTMF_5:
    case CONMAN_AUDIO_RESOURCE_TYPE_DTMF_6:
    case CONMAN_AUDIO_RESOURCE_TYPE_DTMF_7:
    case CONMAN_AUDIO_RESOURCE_TYPE_DTMF_8:
    case CONMAN_AUDIO_RESOURCE_TYPE_DTMF_9:
    case CONMAN_AUDIO_RESOURCE_TYPE_DTMF_HASH:
    case CONMAN_AUDIO_RESOURCE_TYPE_DTMF_ASTERISK:
      {
        unsigned int audio_resource = 0; /* tone generator */
        unsigned int tone_id;

        /* Convert DTMF type to u-blox tone_id. */

        tone_id = ctl->resource.type - CONMAN_AUDIO_RESOURCE_TYPE_DTMF_0;

        /* Play audio. */

        ret = ubmodem_play_audio_resource(conman->ub.modem, audio_resource,
                                          tone_id,
                                          ctl->resource.dtmf.nrepetitions);
      }
      break;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: __conman_ubmodem_start_celllocate
 *
 * Description:
 *   Initiate u-blox modem based CellLocate®
 *
 * Input Parameters:
 *   conman  : connection manager handle
 *   ctl     : CellLocate controls
 *
 * Returned Value:
 *   OK if successfully queued for sending.
 *   Negated error code if case of error.
 ****************************************************************************/

int __conman_ubmodem_start_celllocate(struct conman_s *conman,
                                const struct conman_msg_start_celllocate_s *ctl)
{
#ifndef CONFIG_UBMODEM_DISABLE_CELLLOCATE
  if (conman->ub.status != CONMAN_STATUS_ESTABLISHED)
    {
      return ERROR;
    }
  if (conman->ub.target_level < UBMODEM_LEVEL_GPRS)
    {
      conman_dbg("ERROR: CellLocate needs GRPS connection.\n");
      return ERROR;
    }

  return ubmodem_start_cell_locate(conman->ub.modem, ctl->timeout,
                                   ctl->target_accuracy);
#else
  return ERROR;
#endif
}

/****************************************************************************
 * Name: __conman_ubmodem_aid_celllocate
 *
 * Description:
 *   Give modem current location for aiding u-blox CellLocate®
 *
 * Input Parameters:
 *   conman  : connection manager handle
 *   ctl     : CellLocate aid controls
 *
 * Returned Value:
 *   OK if successfully queued for sending.
 *   Negated error code if case of error.
 ****************************************************************************/

int __conman_ubmodem_aid_celllocate(struct conman_s *conman,
                                 const struct conman_msg_aid_celllocate_s *ctl)
{
#ifndef CONFIG_UBMODEM_DISABLE_AID_CELLLOCATE
  if (conman->ub.status == CONMAN_STATUS_OFF)
    {
      return ERROR;
    }
  if (conman->ub.target_level < UBMODEM_LEVEL_CMD_PROMPT)
    {
      conman_dbg("ERROR: CellLocate aiding needs modem power-on.\n");
      return ERROR;
    }

  return ubmodem_cell_locate_give_location_aid(conman->ub.modem, ctl->time,
                                               ctl->latitude, ctl->longitude,
                                               ctl->altitude, ctl->accuracy,
                                               ctl->speed, ctl->direction);
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
