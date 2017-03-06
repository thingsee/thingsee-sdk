/****************************************************************************
 * apps/system/conman/conman_client.c
 *
 *   Copyright (C) 2015-2016 Haltian Ltd. All rights reserved.
 *   Author: Pekka Ervasti <pekka.ervasti@haltian.com>
 *   Author: Sila Kayo <sila.kayo@haltian.com>
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
#include <sys/socket.h>
#include <errno.h>
#include <pthread.h>
#include <sys/un.h>

#include "conman_dbg.h"
#include "conman_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

struct iovec
{
  FAR void *iov_base; /* Starting address */
  size_t iov_len;     /* Number of bytes to transfer */
};

struct queued_event_s
{
  sq_entry_t node;
  uint8_t type;
  void *payload;
  size_t payloadlen;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: conman_send_reqv
 *
 * Description:
 *   Writes request header and data to file descriptor fd.
 *
 * Input Parameters:
 *   fd  : file descriptor to write to
 *   id  : message id
 *   bufs : data to send
 *   iovcnt : number of data buffers
 *
 * Returned Value:
 *   OK    : no errors
 *   ERROR : failure
 *
 ****************************************************************************/

static int conman_send_reqv(int fd, uint8_t id, const struct iovec *bufs,
                            size_t iovcnt)
{
  struct conman_hdr hdr;
  size_t len = 0;
  size_t i;
  int ret;

  for (i = 0; i < iovcnt; i++)
    {
      len += bufs[i].iov_len;
    }

  hdr.id = id;
  hdr.len = len;

  ret = __conman_util_block_write(fd, &hdr, sizeof(hdr));
  if (ret < 0)
    {
      return ERROR;
    }

  if (len > 0)
    {
      for (i = 0; i < iovcnt; i++)
        {
          ret = __conman_util_block_write(fd, bufs[i].iov_base,
                                          bufs[i].iov_len);
          if (ret < 0)
            {
              return ERROR;
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Name: conman_send_reqv
 *
 * Description:
 *   Writes request header and data to file descriptor fd.
 *
 * Input Parameters:
 *   fd  : file descriptor to write to
 *   id  : message id
 *   buf : data to send
 *   len : length of data in bytes
 *
 * Returned Value:
 *   OK    : no errors
 *   ERROR : failure
 *
 ****************************************************************************/

static int conman_send_req(int fd, uint8_t id, const void * const buf,
                           size_t len)
{
  struct iovec iov;

  iov.iov_base = (void *)buf;
  iov.iov_len = len;

  return conman_send_reqv(fd, id, &iov, 1);
}

/****************************************************************************
 * Name: conman_wait_for_response
 *
 * Description:
 *   Makes a blocking read to the conman server waiting for a response.
 *
 * Input Parameters:
 *   client : client handle
 *   events_only : flag, set true if checking only for events
 *
 * Returned Value:
 *   OK    : no errors
 *   ERROR : failure
 *
 ****************************************************************************/

static int conman_wait_for_response(struct conman_client_s *client,
                                    bool events_only, sq_queue_t *qevents)
{
  struct conman_resp_hdr hdr;
  int ret;

  do
    {
      ret = __conman_util_block_read(client->sd, &hdr, sizeof(hdr));
      if (ret < 0)
        {
          return ERROR;
        }

      if (client->payload)
        {
          free(client->payload);
          client->payload = NULL;
        }

      client->respval = hdr.respval;
      client->payloadlen = hdr.head.len;

      if (client->payloadlen > 0)
        {
          client->payload = malloc(client->payloadlen);
          if (!client->payload)
            {
              __conman_flushfd(client->sd);

              return ERROR;
            }

          ret = __conman_util_block_read(client->sd, client->payload,
                                         client->payloadlen);
          if (ret < 0)
            {
              free(client->payload);
              client->payload = NULL;
              return ERROR;
            }
        }

      /* Check if this is event instead of response. */

      if (hdr.respval == CONMAN_RESP_EVENT)
        {
          /* Does this client handle events? */

          if (!client->event_callback || !qevents)
            {
              free(client->payload);
              client->payload = NULL;
            }
          else
            {
              struct queued_event_s *item;

              /* Queue event. We cannot call event_callback as
               * callback function might issue new conman commands, and we
               * might still be waiting response for another command.
               */
              item = calloc(1, sizeof(*item));
              if (item)
                {
                  sq_addlast(&item->node, qevents);
                  item->type = hdr.head.id;
                  item->payload = client->payload;
                  item->payloadlen = hdr.head.len;

                  client->payload = NULL;
                }
              else
                {
                  dbg("out of memory\n");
                  free(client->payload);
                  client->payload = NULL;
                }
            }

          if (events_only)
            {
              /* Break out after handling one event, so that we do not
               * block on waiting next event.
               */
              break;
            }
        }
      else
        {
          if (events_only)
            {
              dbg("Expected event, got %d\n", hdr.respval);
              return ERROR;
            }
        }
    }
  while (hdr.respval == CONMAN_RESP_EVENT);

  return OK;
}

/****************************************************************************
 * Name: handle_queued_events
 ****************************************************************************/

static void handle_queued_events(struct conman_client_s *client,
                                 sq_queue_t *qevents)
{
  struct queued_event_s *item;

  item = (void *)sq_peek(qevents);
  while (item)
    {
      struct queued_event_s *curr = item;

      item = (void *)sq_next(&item->node);

      client->event_callback(client, curr->type, curr->payload,
                             curr->payloadlen, client->event_priv);
      free(curr->payload);
      free(curr);
    }
}

/****************************************************************************
 * Name: do_command_no_payload
 ****************************************************************************/

static int do_command_no_payload(struct conman_client_s *client, uint8_t type,
                                 const void *buf, size_t buflen)
{
  sq_queue_t qevents;
  int ret;

  ret = conman_send_req(client->sd, type, buf, buflen);
  if (ret != OK)
    {
      conman_dbg("conman_send_req failed\n");
      return ERROR;
    }

  sq_init(&qevents);

  ret = conman_wait_for_response(client, false, &qevents);
  if (ret != OK)
    {
      conman_dbg("conman communication failed\n");
      ret = ERROR;
      goto out;
    }

  if (client->respval != CONMAN_RESP_OK)
    {
      ret = ERROR;
      goto out;
    }

  DEBUGASSERT(client->payload == NULL);

  ret = OK;

out:
  handle_queued_events(client, &qevents);
  return ret;
}

/****************************************************************************
 * Name: conman_client_enable_events
 *
 * Description:
 *   Sends a configuration file to server for parsing.
 *
 * Input Parameters:
 *   client : client handle
 *
 * Returned Value:
 *   OK    : no errors
 *   ERROR : failure
 *
 ****************************************************************************/

static int conman_client_enable_events(struct conman_client_s *client)
{
  conman_dbg("\n");

  return do_command_no_payload(client, CONMAN_MSG_ENABLE_EVENTS, NULL, 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: conman_client_init_events
 *
 * Description:
 *   Opens socket for communication with the server.
 *
 * Input Parameters:
 *   client        : client handle
 *   event_callback: event callback function
 *   event_priv    : private data for callback function
 *
 * Returned Value:
 *   OK    : no errors
 *   ERROR : failure
 *
 ****************************************************************************/

int conman_client_init_events(struct conman_client_s *client,
                              conman_event_callback_fn_t event_callback,
                              void *event_priv)
{
  struct sockaddr_un myaddr;
  socklen_t addrlen;
  int ret;

  memset(client, 0, sizeof(*client));

  client->event_callback = event_callback;
  client->event_priv = event_priv;

  /* Create a new Unix domain socket */

  client->sd = socket(PF_LOCAL, SOCK_STREAM, 0);
  if (client->sd < 0)
    {
      conman_dbg("socket failed: %d\n", errno);
      return ERROR;
    }

  /* Connect the socket to the server */

  addrlen = strlen(CONFIG_CONMAN_LISTEN_SOCKET_ADDR);
  if (addrlen > UNIX_PATH_MAX - 1)
    {
      addrlen = UNIX_PATH_MAX - 1;
    }

  myaddr.sun_family = AF_LOCAL;
  strncpy(myaddr.sun_path, CONFIG_CONMAN_LISTEN_SOCKET_ADDR, addrlen);
  myaddr.sun_path[addrlen] = '\0';

  conman_dbg("Connecting to %s\n", CONFIG_CONMAN_LISTEN_SOCKET_ADDR);
  addrlen += sizeof(sa_family_t) + 1;
  ret = connect(client->sd, (struct sockaddr *)&myaddr, addrlen);
  if (ret < 0)
    {
      conman_dbg("connect failed: %d\n", errno);
      goto errout_with_socket;
    }

  conman_dbg("Connected\n");

  if (event_callback)
    {
      /* Enable events. */

      if (conman_client_enable_events(client) < 0)
        {
          conman_dbg("conman_client_enable_events failed\n");
          goto errout_with_socket;
        }
    }

  return OK;

errout_with_socket:
  close(client->sd);
  return ERROR;
}

/****************************************************************************
 * Name: conman_client_init
 *
 * Description:
 *   Opens socket for communication with the server.
 *
 * Input Parameters:
 *   client : client handle
 *
 * Returned Value:
 *   OK    : no errors
 *   ERROR : failure
 *
 ****************************************************************************/

int conman_client_init(struct conman_client_s *client)
{
  return conman_client_init_events(client, NULL, NULL);
}

/****************************************************************************
 * Name: conman_client_uninit
 *
 * Description:
 *   Closes the 2 fifos used by client server communication.
 *
 * Input Parameters:
 *   client : client handle
 *
 ****************************************************************************/

void conman_client_uninit(struct conman_client_s *client)
{
  close(client->sd);
  free(client->payload);
}

/****************************************************************************
 * Name: conman_client_handle_events
 *
 * Description:
 *   Handle pending events on client socket (called after getting POLLIN-event)
 *
 * Input Parameters:
 *   client : client handle
 *
 * Returned Value:
 *   OK    : no errors
 *   ERROR : failure
 *
 ****************************************************************************/

int conman_client_handle_events(struct conman_client_s *client)
{
  sq_queue_t qevents;
  int ret;

  sq_init(&qevents);

  ret = conman_wait_for_response(client, true, &qevents);

  handle_queued_events(client, &qevents);

  return ret;
}

/****************************************************************************
 * Name: conman_client_set_connections_config
 *
 * Description:
 *   Sends a configuration file to server for parsing.
 *
 * Input Parameters:
 *   client : client handle
 *
 * Returned Value:
 *   OK    : no errors
 *   ERROR : failure
 *
 ****************************************************************************/

int conman_client_set_connections_config(struct conman_client_s *client,
                                         const char *config)
{
  conman_dbg("%s\n", config);

  return do_command_no_payload(client, CONMAN_MSG_SET_CONNECTIONS_CONFIG,
                               config, strlen(config) + 1);
}

/****************************************************************************
 * Name: conman_client_request_connection
 *
 * Description:
 *   Sends a connection request to the server.
 *
 * Input Parameters:
 *   client : client handle
 *   type   : connection type
 *   connid : returned connection id
 *
 * Returned Value:
 *   OK    : no errors
 *   ERROR : failure
 *
 ****************************************************************************/

int conman_client_request_connection(struct conman_client_s *client,
                                     enum conman_connection_type_e type,
                                     uint32_t *connid)
{
  sq_queue_t qevents;
  int ret;

  ret = conman_send_req(client->sd, CONMAN_MSG_CREATE_CONNECTION, &type,
      sizeof(type));
  if (ret != OK)
    {
      conman_dbg("conman_send_req failed\n");
      return ERROR;
    }

  sq_init(&qevents);

  ret = conman_wait_for_response(client, false, &qevents);
  if (ret != OK)
    {
      conman_dbg("conman communication failed\n");
      ret = ERROR;
      goto out;
    }

  if (client->respval != CONMAN_RESP_OK)
    {
      ret = ERROR;
      goto out;
    }

  DEBUGASSERT(sizeof(*connid) == client->payloadlen);

  memcpy(connid, client->payload, sizeof(*connid));
  free(client->payload);
  client->payload = NULL;

  ret = OK;

out:
  handle_queued_events(client, &qevents);
  return ret;
}

/****************************************************************************
 * Name: conman_client_destroy_connection
 *
 * Description:
 *   Request server to destroy connection.
 *
 * Input Parameters:
 *   client : client handle
 *   connid : id of connection to destroy
 *
 * Returned Value:
 *   OK    : no errors
 *   ERROR : failure
 *
 ****************************************************************************/

int conman_client_destroy_connection(struct conman_client_s *client,
                                     uint32_t connid)
{
  return do_command_no_payload(client, CONMAN_MSG_DESTROY_CONNECTION, &connid,
                               sizeof(connid));
}

/****************************************************************************
 * Name: conman_client_get_connection_status
 *
 * Description:
 *   Sends a connection request to the server.
 *
 * Input Parameters:
 *   client : client handle
 *   type   : connection type
 *
 * Returned Value:
 *   OK    : no errors
 *   ERROR : failure
 *
 ****************************************************************************/

int conman_client_get_connection_status(struct conman_client_s *client,
                                        struct conman_status_s *status)
{
  sq_queue_t qevents;
  int ret;

  if (!status)
    {
      return ERROR;
    }

  ret = conman_send_req(client->sd, CONMAN_MSG_GET_CONNECTION_STATUS,
                        NULL, 0);
  if (ret != OK)
    {
      conman_dbg("conman_send_req failed\n");
      return ERROR;
    }

  sq_init(&qevents);

  ret = conman_wait_for_response(client, false, &qevents);
  if (ret != OK)
    {
      conman_dbg("conman communication failed\n");
      ret = ERROR;
      goto out;
    }

  if (client->respval != CONMAN_RESP_OK)
    {
      ret = ERROR;
      goto out;
    }

  DEBUGASSERT(sizeof(*status) == client->payloadlen);

  memcpy(status, client->payload, sizeof(*status));
  free(client->payload);
  client->payload = NULL;

  ret = OK;

out:
  handle_queued_events(client, &qevents);
  return ret;
}

/****************************************************************************
 * Name: conman_client_send_sms
 *
 * Description:
 *   Send SMS message to spesified phone number.
 *
 * Input Parameters:
 *   client      : client handle
 *   phonenumber : phone number
 *   message     : message to send
 *
 * Returned Value:
 *   OK    : SMS successfully queued for sending (actual sending will not
 *           happen as this function returns).
 *   ERROR : failure
 *
 ****************************************************************************/

int conman_client_send_sms(struct conman_client_s *client,
                           const char *phonenumber, const char *message)
{
  struct conman_msg_send_sms_s sms = {};
  size_t receiverlen = strlen(phonenumber);
  size_t messagelen = strlen(message);
  struct iovec bufs[3];
  sq_queue_t qevents;
  int ret;

  sms.message_len = messagelen + 1;
  sms.receiver_len = receiverlen + 1;

  if (sms.message_len != messagelen + 1)
    {
      conman_dbg("Too long message\n");
      return ERROR;
    }

  if (sms.receiver_len != receiverlen + 1)
    {
      conman_dbg("Too long phone-number\n");
      return ERROR;
    }

  bufs[0].iov_base = &sms;
  bufs[0].iov_len = sizeof(sms);
  bufs[1].iov_base = (void *)phonenumber;
  bufs[1].iov_len = receiverlen + 1;
  bufs[2].iov_base = (void *)message;
  bufs[2].iov_len = messagelen + 1;

  ret = conman_send_reqv(client->sd, CONMAN_MSG_SEND_SMS, bufs, 3);
  if (ret != OK)
    {
      conman_dbg("conman_send_reqv failed\n");
      return ERROR;
    }

  sq_init(&qevents);

  ret = conman_wait_for_response(client, false, &qevents);
  if (ret != OK)
    {
      conman_dbg("conman communication failed\n");
      ret = ERROR;
      goto out;
    }

  DEBUGASSERT(client->payload == NULL);

  if (client->respval != CONMAN_RESP_OK)
    {
      ret = ERROR;
      goto out;
    }

  ret = OK;

out:
  handle_queued_events(client, &qevents);
  return ret;
}

/****************************************************************************
 * Name: conman_client_filesystem_delete
 *
 * Description:
 *   Delete file from modem filesystem.
 *
 * Input Parameters:
 *   client       : client handle
 *   filename     : name of file to be removed
 *
 * Returned Value:
 *   ERROR if failed, OK on success.
 *
 ****************************************************************************/

int conman_client_filesystem_delete(struct conman_client_s *client,
                                    char *filename)
{
  conman_dbg("%s\n", filename);

  return do_command_no_payload(client, CONMAN_MSG_FILESYSTEM_DELETE,
                               filename, strlen(filename) + 1);
}

/****************************************************************************
 * Name: conman_client_ftp_download
 *
 * Description:
 *   Retrieve a file from FTP server to modem filesystem.
 *
 * Input Parameters:
 *   client       : client handle
 *   hostname     : FTP server hostname
 *   username     : FTP username
 *   password     : FTP password
 *   filepath_src : Remote file path to retrieve
 *   filepath_dst : Local file path to be stored
 *
 * Returned Value:
 *   ERROR if failed, OK on success.
 *
 ****************************************************************************/

int conman_client_ftp_download(struct conman_client_s *client,
                               char *hostname,
                               char *username,
                               char *password,
                               char *filepath_src,
                               char *filepath_dst)
{
  struct conman_msg_ftp_download_s ftp = {};
  size_t hostname_len = strlen(hostname);
  size_t username_len = strlen(username);
  size_t password_len = strlen(password);
  size_t filepath_src_len = strlen(filepath_src);
  size_t filepath_dst_len = strlen(filepath_dst);
  sq_queue_t qevents;
  struct iovec bufs[6];
  int ret;

  ftp.hostname_len = hostname_len + 1;
  ftp.username_len = username_len + 1;
  ftp.password_len = password_len + 1;
  ftp.filepath_src_len = filepath_src_len + 1;
  ftp.filepath_dst_len = filepath_dst_len + 1;

  if (ftp.hostname_len != hostname_len + 1)
    {
      conman_dbg("Too long hostname\n");
      return ERROR;
    }

  if (ftp.username_len != username_len + 1)
    {
      conman_dbg("Too long username\n");
      return ERROR;
    }

  if (ftp.password_len != password_len + 1)
    {
      conman_dbg("Too long password\n");
      return ERROR;
    }

  if (ftp.filepath_src_len != filepath_src_len + 1)
    {
      conman_dbg("Too long filepath_src\n");
      return ERROR;
    }

  if (ftp.filepath_dst_len != filepath_dst_len + 1)
    {
      conman_dbg("Too long filepath_dst\n");
      return ERROR;
    }

  bufs[0].iov_base = &ftp;
  bufs[0].iov_len = sizeof(ftp);
  bufs[1].iov_base = (void *)hostname;
  bufs[1].iov_len = ftp.hostname_len;
  bufs[2].iov_base = (void *)username;
  bufs[2].iov_len = ftp.username_len;
  bufs[3].iov_base = (void *)password;
  bufs[3].iov_len = ftp.password_len;
  bufs[4].iov_base = (void *)filepath_src;
  bufs[4].iov_len = ftp.filepath_src_len;
  bufs[5].iov_base = (void *)filepath_dst;
  bufs[5].iov_len = ftp.filepath_dst_len;

  ret = conman_send_reqv(client->sd, CONMAN_MSG_FTP_DOWNLOAD, bufs, 6);
  if (ret != OK)
    {
      conman_dbg("conman_send_reqv failed\n");
      return ERROR;
    }

  sq_init(&qevents);

  ret = conman_wait_for_response(client, false, &qevents);
  if (ret != OK)
    {
      conman_dbg("conman communication failed\n");
      ret = ERROR;
      goto out;
    }

  DEBUGASSERT(client->payload == NULL);

  if (client->respval != CONMAN_RESP_OK)
    {
      ret = ERROR;
      goto out;
    }

  ret = OK;

out:
  handle_queued_events(client, &qevents);
  return ret;
}

/****************************************************************************
 * Name: conman_client_call_answer
 *
 * Description:
 *   Sends a voice-call answer request to the server.
 *
 * Input Parameters:
 *   client       : client handle
 *   mute_speaker : answer with speaker muted
 *   mute_mic     : answer with mic muted
 *
 * Returned Value:
 *   OK    : no errors
 *   ERROR : failure
 *
 ****************************************************************************/

int conman_client_call_answer(struct conman_client_s *client,
                              bool mute_speaker, bool mute_mic)
{
  struct conman_msg_call_answer_s data = {};

  data.mic_mute = mute_mic;
  data.speaker_mute = mute_speaker;

  return do_command_no_payload(client, CONMAN_MSG_CALL_ANSWER,
                               &data, sizeof(data));
}

/****************************************************************************
 * Name: conman_client_call_hangup
 *
 * Description:
 *   Sends a voice-call hangup request to the server.
 *
 * Input Parameters:
 *   client       : client handle
 *
 * Returned Value:
 *   OK    : no errors
 *   ERROR : failure
 *
 ****************************************************************************/

int conman_client_call_hangup(struct conman_client_s *client)
{
  return do_command_no_payload(client, CONMAN_MSG_CALL_HANGUP, NULL, 0);
}

/****************************************************************************
 * Name: conman_client_call_audio_control
 *
 * Description:
 *   Sends a voice-call audio_control request to the server.
 *
 * Input Parameters:
 *   client    : client handle
 *   audio_out : enable audio output (for making ringing audiable)
 *
 * Returned Value:
 *   OK    : no errors
 *   ERROR : failure
 *
 ****************************************************************************/

int conman_client_call_audio_control(struct conman_client_s *client,
                                     bool audio_out_on)
{
  struct conman_msg_call_audioctl_s data = {};

  data.audio_out_on = audio_out_on;

  return do_command_no_payload(client, CONMAN_MSG_CALL_AUDIO_CONTROL,
                               &data, sizeof(data));
}

/****************************************************************************
 * Name: conman_client_play_audio_resource
 *
 * Description:
 *   Play modem audio resource.
 *
 * Input Parameters:
 *   client    : client handle
 *   resource  : audio resource selection and configuration
 *
 * Returned Value:
 *   OK    : no errors
 *   ERROR : failure
 *
 ****************************************************************************/

int conman_client_play_audio_resource(struct conman_client_s *client,
        struct conman_event_play_audio_resource *resource)
{
  struct conman_msg_play_audio_resource_s data = { .resource = *resource };

  return do_command_no_payload(client, CONMAN_MSG_PLAY_AUDIO_RESOURCE,
                               &data, sizeof(data));
}

/****************************************************************************
 * Name: conman_client_start_celllocate
 *
 * Description:
 *   Initiate u-blox modem based CellLocate®
 *
 * Input Parameters:
 *   client         : client handle
 *   timeout        : timeout for CellLocate® (in seconds)
 *   target_accuracy: target accuracy (in seconds)
 *
 * Returned Value:
 *   OK    : no errors
 *   ERROR : failure
 *
 ****************************************************************************/

int conman_client_start_celllocate(struct conman_client_s *client,
                                   int timeout, int target_accuracy)
{
  struct conman_msg_start_celllocate_s data = {};

  data.timeout = timeout;
  data.target_accuracy = target_accuracy;

  return do_command_no_payload(client, CONMAN_MSG_START_CELLLOCATE,
                               &data, sizeof(data));
}

/****************************************************************************
 * Name: conman_client_aid_celllocate
 *
 * Description:
 *   Give modem current location for aiding u-blox CellLocate®
 *
 * Input Parameters:
 *   client         : client handle
 *   time           : UTC timestamp of aided location
 *   latitude       : Estimated latitude in degrees
 *   longitude      : Estimated longitude in degrees
 *   altitude       : Estimated altitude in meters
 *   accuracy       : Estimated accuracy in meters
 *   speed          : Estimated speed in meters per second
 *   direction      : Estimated direction of speed in degrees
 *                    (north zero, clockwise)
 *
 * Returned Value:
 *   OK    : no errors
 *   ERROR : failure
 *
 ****************************************************************************/

int conman_client_aid_celllocate(struct conman_client_s *client,
                                 time_t time, double latitude, double longitude,
                                 int altitude, unsigned int accuracy,
                                 unsigned int speed, unsigned int direction)
{
  struct conman_msg_aid_celllocate_s data = {};

  data.time = time;
  data.latitude = latitude;
  data.longitude = longitude;
  data.altitude = altitude;
  data.accuracy = accuracy;
  data.speed = speed;
  data.direction = direction;

  return do_command_no_payload(client, CONMAN_MSG_AID_CELLLOCATE,
                               &data, sizeof(data));
}

/****************************************************************************
 * Name: conman_client_request_cell_environment
 *
 * Description:
 *   Request cell environment information from u-blox modem,
 *   results are returned with CONMAN_EVENT_CELL_ENVIRONMENT
 *
 * Input Parameters:
 *   client         : client handle
 *
 * Returned Value:
 *   OK    : no errors
 *   ERROR : failure
 *
 ****************************************************************************/

int conman_client_request_cell_environment(struct conman_client_s *client)
{
  return do_command_no_payload(client, CONMAN_MSG_REQUEST_CELL_ENVIRONMENT,
                               NULL, 0);
}

/****************************************************************************
 * Name: conman_client_ping
 *
 * Description:
 *   Ping/wake conman server.
 *
 * Input Parameters:
 *   client : client handle
 *
 * Returned Value:
 *   OK    : no errors
 *   ERROR : failure
 *
 ****************************************************************************/

int conman_client_ping(struct conman_client_s *client)
{
  return do_command_no_payload(client, CONMAN_MSG_PING, NULL, 0);
}

/****************************************************************************
 * Name: conman_client_wifi_scan
 *
 * Description:
 *   Initiate WiFi scanning
 *
 * Input Parameters:
 *   client         : client handle
 *
 * Returned Value:
 *   OK    : no errors
 *   ERROR : failure
 *
 ****************************************************************************/

int conman_client_wifi_scan(struct conman_client_s *client)
{
#ifdef CONFIG_SYSTEM_CONMAN_WIFI
  return do_command_no_payload(client, CONMAN_MSG_WIFI_SCAN, NULL, 0);
#else
  errno = -ENOSYS;
  return ERROR;
#endif
}
