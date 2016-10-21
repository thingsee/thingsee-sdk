/****************************************************************************
 * apps/system/conman/conman_ctl.c
 *
 *   Copyright (C) 2015-2016 Haltian Ltd. All rights reserved.
 *   Authors: Pekka Ervasti <pekka.ervasti@haltian.com>
 *            Jussi Kivilinna <jussi.kivilinna@haltian.com>
 *            Sila Kayo <sila.kayo@haltian.com>
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

#include <string.h>
#include <stdlib.h>
#include <poll.h>
#include <errno.h>

#include <apps/netutils/cJSON.h>

#include "conman_dbg.h"
#include "conman_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

//#define POLLERR_BUG

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int do_destroy_connection(struct conman_s *conman,
                                 enum conman_connection_type_e type)
{
  int ret = ERROR;

  switch (type)
    {
    case CONMAN_DATA:
      DEBUGASSERT(false);
      break;

    case CONMAN_2G:
    case CONMAN_SMS:
      ret = __conman_ubmodem_request_connection(conman, CONMAN_NONE);
      break;

    case CONMAN_WIFI:
      ret = __conman_cc3000_request_connection(conman, CONMAN_NONE);
      break;

    default:
      DEBUGASSERT(false);
      break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __conman_ctl_create_connection
 *
 * Description:
 *   Creates a connection by making a request to the modem or wifi module.
 *
 * Input Parameters:
 *   conman  : connection manager handle
 *   type    : connection type
 *   connid  : allocated connection id
 *
 * Returned Value:
 *   OK    : no errors
 *   ERROR : failure
 *
 ****************************************************************************/

int __conman_ctl_create_connection(struct conman_s *conman,
                                   enum conman_connection_type_e type,
                                   uint32_t *connid)
{
  struct conman_connid_s *conn;
  int ret = ERROR;

  if (type == CONMAN_NONE)
    {
      *connid = CONMAN_CONNID_CLEAR;
      return OK;
    }

  /* Allocate connid. */

  conn = calloc(1, sizeof(*conn));
  if (conn == NULL)
    {
      return ERROR;
    }

  switch (type)
    {
    case CONMAN_NONE:

    case CONMAN_DATA:
      /* First try WiFi. */

      if (conman->connections.amount.wifi > 0)
        {
          ret = __conman_cc3000_request_connection(conman, CONMAN_WIFI);
          if (ret == OK)
            {
              type = CONMAN_WIFI;
              break;
            }
        }

      /* Then try with cellular. */

      if (conman->connections.amount.cellular > 0)
        {
          ret = __conman_ubmodem_request_connection(conman, CONMAN_2G);
          if (ret == OK)
            {
              type = CONMAN_2G;
              break;
            }
        }

      goto errfree;

    case CONMAN_2G:
    case CONMAN_SMS:
      if (conman->connections.amount.cellular > 0)
        {
          ret = __conman_ubmodem_request_connection(conman, type);
          if (ret == OK)
            {
              break;
            }
        }

      goto errfree;

    case CONMAN_WIFI:
      if (conman->connections.amount.wifi > 0)
        {
          ret = __conman_cc3000_request_connection(conman, type);
          if (ret == OK)
            {
              break;
            }
        }

      goto errfree;

    default:
      goto errfree;
    }

  conn->connid = conman->connections.current.connid_next++;
  conn->type = type;
  conn->destroyed = false;
  sq_addlast(&conn->node, &conman->connections.current.connids);

  *connid = conn->connid;

  if (conman->connections.current.connid_next > CONMAN_CONNID_MAX ||
      conman->connections.current.connid_next < CONMAN_CONNID_MIN)
    {
      conman->connections.current.connid_next = CONMAN_CONNID_MIN;
    }

  return OK;

errfree:
  free(conn);

  return ERROR;
}

/****************************************************************************
 * Name: __conman_ctl_destroy_connection
 *
 * Description:
 *  Destroys the current connection by making a request to the modem or
 *  wifi module.
 *
 * Input Parameters:
 *   conman  : connection manager handle
 *   connid  : connection id
 *
 * Returned Value:
 *   OK    : no errors
 *   ERROR : failure
 *
 ****************************************************************************/

int __conman_ctl_destroy_connection(struct conman_s *conman, uint32_t connid)
{
  struct conman_connid_s *conn;
  int ret = ERROR;

  if (connid == CONMAN_CONNID_ALL)
    {
      /* Destroy all. */

      conn = (void *)sq_peek(&conman->connections.current.connids);

      while (conn != NULL)
        {
          if (!conn->destroyed)
            {
              conn->destroyed = true;
              (void)do_destroy_connection(conman, conn->type);
            }
          conn = (void *)sq_next(&conn->node);
        }

      return OK;
    }

  /* Get connid type. */

  conn = (void *)sq_peek(&conman->connections.current.connids);

  while (conn != NULL)
    {
      if (conn->connid == connid)
        break;
      conn = (void *)sq_next(&conn->node);
    }
  if (conn == NULL)
    {
      return ERROR;
    }

  sq_rem(&conn->node, &conman->connections.current.connids);

  if (conn->destroyed)
    {
      /* Already destroyed with CONMAN_CONNID_ALL. */

      free(conn);
      return OK;
    }

  ret = do_destroy_connection(conman, conn->type);
  if (ret < 0)
    {
      ret = ERROR;
    }
  else
    {
      ret = OK;
    }

  free(conn);
  return ret;
}

/****************************************************************************
 * Name: __conman_ctl_get_status_connection
 *
 * Description:
 *  Get status for the current connection.
 *
 * Input Parameters:
 *   conman  : connection manager handle
 *   status  : pointer to status structure
 *
 * Returned Value:
 *   OK         : no errors
 *   ERROR      : failure
 *   EINPROGRESS: took over handling response
 *
 ****************************************************************************/

int __conman_ctl_get_status_connection(struct conman_s *conman,
                                       struct conman_status_s *status)
{
  int ret = ERROR;
  bool destroying;

  /* TODO: Better status function. Per connid? Per type? */

  if (conman->connections.amount.wifi > 0)
    {
      memset(status, 0, sizeof(*status));
      ret = __conman_cc3000_get_status_connection(conman, status);
      if (ret == OK && (status->status == CONMAN_STATUS_ESTABLISHED ||
                        status->status == CONMAN_STATUS_ESTABLISHING))
        {
          return OK;
        }
      if (ret == EINPROGRESS)
        {
          return ret;
        }
    }

  if (ret != OK || (ret == OK && status->status == CONMAN_STATUS_OFF))
    {
      if (conman->connections.amount.cellular > 0)
        {
          memset(status, 0, sizeof(*status));
          ret = __conman_ubmodem_get_status_connection(conman, status);
          if (ret == OK && (status->status == CONMAN_STATUS_ESTABLISHED ||
                            status->status == CONMAN_STATUS_ESTABLISHING))
            {
              return OK;
            }
          if (ret == EINPROGRESS)
            {
              return ret;
            }
        }
    }

  status->conn_type = CONMAN_NONE;
  status->status = CONMAN_STATUS_OFF;
  status->info_type = CONMAN_INFO_NONE;

  destroying = false;
  destroying = destroying || __conman_ubmodem_is_destroying(conman);
  destroying = destroying || __conman_cc3000_is_destroying(conman);
  status->destroying_prev = destroying;

  return OK;
}

/****************************************************************************
 * Name: __conman_ctl_handle_pollin
 *
 * Description:
 *   Handles pollin events from the client to server fifo file descriptor.
 *   Processes the request and sends a reply.
 *
 * Input Parameters:
 *   conman  : connection manager handle
 *   pfd     : pointer to the pollfd struct
 *
 ****************************************************************************/

void __conman_ctl_handle_pollin(struct conman_s *conman, struct pollfd *pfd)
{
  struct conman_hdr hdr;
  int ret;
  char *data = NULL;
  enum conman_resp_value resp = CONMAN_RESP_OK;

  ASSERT(conman);
#ifndef POLLERR_BUG
  DEBUGASSERT(pfd->revents & (POLLIN | POLLERR | POLLHUP));
#endif

#ifndef POLLERR_BUG
  if (pfd->revents & (POLLERR | POLLHUP))
#else
  if (pfd->revents == 0)
#endif
    {
      /* The other end has closed the connection */

      __conman_server_close_sd(conman, pfd->fd);
      return;
    }

  conman->servingsd = pfd->fd;

  ret = __conman_util_block_read(pfd->fd, &hdr, sizeof(hdr));
  if (ret < 0)
    {
      return;
    }

  if (hdr.len)
    {
      data = malloc(hdr.len);
      if (!data)
        {
          conman_dbg("malloc(%d) failed\n", hdr.len);
          __conman_flushfd(pfd->fd);
          return (void) __conman_send_resp(conman, hdr.id, CONMAN_RESP_OOM, NULL, 0);
        }

      ret = __conman_util_block_read(pfd->fd, data, hdr.len);
      if (ret < 0)
        {
          __conman_flushfd(pfd->fd);
          resp = CONMAN_RESP_EIO;
          goto out;
        }
    }
  /* no else - depending on the message type empty payload may be allowed */


  switch (hdr.id)
    {
    case CONMAN_MSG_ENABLE_EVENTS:
      {
        DEBUGASSERT(hdr.len == 0);
        ret = __conman_server_enable_events_for_client(conman, pfd->fd);
        if (ret < 0)
          {
            conman_dbg("__conman_server_enable_events_for_client failed\n");
            resp = CONMAN_RESP_ERROR;
          }
      }
      break;

    case CONMAN_MSG_SET_CONNECTIONS_CONFIG:
      {
        DEBUGASSERT(hdr.len > 0);
        ret = __conman_config_set_connections(conman, data);
        if (ret < 0)
          {
            conman_dbg("__conman_config_set_connections failed\n");
            resp = CONMAN_RESP_ERROR;
          }
      }
      break;

    case CONMAN_MSG_CREATE_CONNECTION:
      {
        uint32_t connid = CONMAN_CONNID_CLEAR;
        enum conman_connection_type_e type;
        DEBUGASSERT(hdr.len == sizeof(type));

        type = *(enum conman_connection_type_e *) data;

        ret = __conman_ctl_create_connection(conman, type, &connid);
        if (ret < 0)
          {
            conman_dbg("conman_ctl_create_connection failed\n");
            resp = CONMAN_RESP_ERROR;
          }
        else
          {
            free(data);

            __conman_send_resp(conman, hdr.id, resp, &connid, sizeof(connid));
            return;
          }
      }
      break;

    case CONMAN_MSG_DESTROY_CONNECTION:
      {
        uint32_t connid;

        DEBUGASSERT(hdr.len == sizeof(uint32_t));

        connid = *(uint32_t *)data;

        ret = __conman_ctl_destroy_connection(conman, connid);
        if (ret < 0)
          {
            conman_dbg("conman_ctl_destroy_connection failed\n");
            resp = CONMAN_RESP_ERROR;
          }
      }
      break;

    case CONMAN_MSG_GET_CONNECTION_STATUS:
      {
        DEBUGASSERT(hdr.len == 0);
        struct conman_status_s status = {};

        free(data);

        ret = __conman_ctl_get_status_connection(conman, &status);
        if (ret < 0)
          {
            conman_dbg("conman_ctl_get_status_connection failed\n");
            resp = CONMAN_RESP_ERROR;
          }

        if (ret != EINPROGRESS)
          {
            __conman_send_resp(conman, hdr.id, resp, &status, sizeof(status));
          }

        return;
      }
      break;

    case CONMAN_MSG_REQUEST_CELL_ENVIRONMENT:
      {
        DEBUGASSERT(hdr.len == 0);

        ret = __conman_ubmodem_request_cell_environment(conman);
        if (ret < 0)
          {
            conman_dbg("__conman_ubmodem_request_cell_environment failed\n");
            resp = CONMAN_RESP_ERROR;
          }
        break;
      }

    case CONMAN_MSG_SEND_SMS:
      {
        struct conman_msg_send_sms_s *sms = (void *)data;

        DEBUGASSERT(hdr.len >= sizeof(struct conman_msg_send_sms_s));
        DEBUGASSERT(sms->receiver_len > 0);
        DEBUGASSERT(sizeof(struct conman_msg_send_sms_s)
                     + sms->receiver_len < hdr.len);
        DEBUGASSERT(sms->message_len > 0);
        DEBUGASSERT(sizeof(struct conman_msg_send_sms_s) + sms->receiver_len
                     + sms->message_len <= hdr.len);

        ret = __conman_ubmodem_send_sms(conman, sms);
        if (ret < 0)
          {
            conman_dbg("conman_ubmodem_send_sms failed\n");
            resp = CONMAN_RESP_ERROR;
          }

        __conman_send_resp(conman, hdr.id, resp, NULL, 0);
        return;
      }
      break;

    case CONMAN_MSG_FTP_DOWNLOAD:
      {
        struct conman_msg_ftp_download_s *ftp = (void *)data;
        size_t size;

        size = sizeof(struct conman_msg_ftp_download_s);
        DEBUGASSERT(size < hdr.len);

        DEBUGASSERT(ftp->hostname_len > 0);
        size += ftp->hostname_len;
        DEBUGASSERT(size < hdr.len);

        DEBUGASSERT(ftp->username_len > 0);
        size += ftp->username_len;
        DEBUGASSERT(size < hdr.len);

        DEBUGASSERT(ftp->password_len > 0);
        size += ftp->password_len;
        DEBUGASSERT(size < hdr.len);

        DEBUGASSERT(ftp->filepath_src_len > 0);
        size += ftp->filepath_src_len;
        DEBUGASSERT(size < hdr.len);

        DEBUGASSERT(ftp->filepath_dst_len > 0);
        size += ftp->filepath_dst_len;
        DEBUGASSERT(size <= hdr.len);

        ret = __conman_ubmodem_ftp_download(conman, ftp);
        if (ret < 0)
          {
            conman_dbg("conman_ubmodem_ftp_download failed\n");
            resp = CONMAN_RESP_ERROR;
          }

        __conman_send_resp(conman, hdr.id, resp, NULL, 0);
        return;
      }
      break;

    case CONMAN_MSG_FILESYSTEM_DELETE:
      {
        DEBUGASSERT(hdr.len > 0);
        DEBUGASSERT(data[hdr.len - 1] == '\0');

        ret = __conman_ubmodem_filesystem_delete(conman, data);
        if (ret < 0)
          {
            conman_dbg("__conman_config_set_connections failed\n");
            resp = CONMAN_RESP_ERROR;
          }
      }
      break;

    case CONMAN_MSG_CALL_ANSWER:
      {
        struct conman_msg_call_answer_s *mute = (void *)data;

        DEBUGASSERT(hdr.len >= sizeof(*mute));

        ret = __conman_ubmodem_call_answer(conman, mute);
        if (ret < 0)
          {
            conman_dbg("__conman_ubmodem_call_answer failed\n");
            resp = CONMAN_RESP_ERROR;
          }
      }
      break;

    case CONMAN_MSG_CALL_HANGUP:
      {
        DEBUGASSERT(hdr.len == 0);

        ret = __conman_ubmodem_call_hangup(conman);
        if (ret < 0)
          {
            conman_dbg("__conman_ubmodem_call_hangup failed\n");
            resp = CONMAN_RESP_ERROR;
          }
      }
      break;

    case CONMAN_MSG_CALL_AUDIO_CONTROL:
      {
        struct conman_msg_call_audioctl_s *ctl = (void *)data;

        DEBUGASSERT(hdr.len >= sizeof(*ctl));

        ret = __conman_ubmodem_call_audioctl(conman, ctl);
        if (ret < 0)
          {
            conman_dbg("__conman_ubmodem_call_audioctl failed\n");
            resp = CONMAN_RESP_ERROR;
          }
      }
      break;

    case CONMAN_MSG_START_CELLLOCATE:
      {
        struct conman_msg_start_celllocate_s *ctl = (void *)data;

        DEBUGASSERT(hdr.len >= sizeof(*ctl));

        ret = __conman_ubmodem_start_celllocate(conman, ctl);
        if (ret < 0)
          {
            conman_dbg("__conman_ubmodem_start_celllocate failed\n");
            resp = CONMAN_RESP_ERROR;
          }
      }
      break;

    case CONMAN_MSG_AID_CELLLOCATE:
      {
        struct conman_msg_aid_celllocate_s *ctl = (void *)data;

        DEBUGASSERT(hdr.len >= sizeof(*ctl));

        ret = __conman_ubmodem_aid_celllocate(conman, ctl);
        if (ret < 0)
          {
            conman_dbg("__conman_ubmodem_aid_celllocate failed\n");
            resp = CONMAN_RESP_ERROR;
          }
      }
      break;

    case CONMAN_MSG_WIFI_SCAN:
      {
        DEBUGASSERT(hdr.len == 0);

        ret = __conman_cc3000_wifi_scan(conman);
        if (ret < 0)
          {
            conman_dbg("__conman_cc3000_wifi_scan failed\n");
            resp = CONMAN_RESP_ERROR;
          }
      }
      break;

    case CONMAN_MSG_PING:
      break;

    default:
      dbg("UNKNOWN COMMAND: %d\n", hdr.id);
      resp = CONMAN_RESP_ERROR;
      break;
    }

out:
  free(data);

  __conman_send_resp(conman, hdr.id, resp, NULL, 0);
}
