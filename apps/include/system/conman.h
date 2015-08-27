/****************************************************************************
 * apps/include/system/conman.h
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

#ifndef __APPS_INCLUDE_SYSTEM_CONMAN_H
#define __APPS_INCLUDE_SYSTEM_CONMAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <netinet/in.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CONMAN_CONNID_ALL UINT32_MAX
#define CONMAN_CONNID_CLEAR 0
#define CONMAN_CONNID_MIN (CONMAN_CONNID_CLEAR + 1)
#define CONMAN_CONNID_MAX (CONMAN_CONNID_ALL - 1)

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

struct conman_client_s;

enum conman_connection_type_e
{
  CONMAN_NONE = 0,
  CONMAN_2G,
  CONMAN_WIFI,
  CONMAN_DATA, /* 2G or WIFI */
  CONMAN_SMS,
};

enum conman_status_type_e
{
  CONMAN_STATUS_OFF = 0,
  CONMAN_STATUS_ESTABLISHING,
  CONMAN_STATUS_ESTABLISHED,
  CONMAN_STATUS_FAILED,
  CONMAN_STATUS_DESTROYING,
};

enum conman_event_type_e
{
  CONMAN_EVENT_CALL_INCOMING = 1,
  CONMAN_EVENT_CALL_ACTIVE,
  CONMAN_EVENT_CALL_DISCONNECTED,
};

enum conman_resp_value
{
  CONMAN_RESP_OK = 0,
  CONMAN_RESP_EVENT,
  CONMAN_RESP_OOM,
  CONMAN_RESP_PARSE_FAIL,
  CONMAN_RESP_NO_CONNECTIONS,
  CONMAN_RESP_EIO,
  CONMAN_RESP_ERROR = -1,
};

enum conman_info_type_e
{
  CONMAN_INFO_NONE = 0,
  CONMAN_INFO_CELLULAR,
  CONMAN_INFO_WIFI,
};

typedef void (* conman_event_callback_fn_t)(struct conman_client_s *client,
                                            enum conman_event_type_e event,
                                            void *payload, size_t payloadlen,
                                            void *priv);

struct conman_client_s
{
  int sd;

  conman_event_callback_fn_t event_callback;
  void *event_priv;
  enum conman_resp_value respval;
  size_t payloadlen;
  void *payload;
};

struct conman_event_call_incoming_info
{
  char number[20];
  uint8_t numbertype;
};

struct conman_status_cellu_info
{
  char oper_name[32 + 1];
  char imei[15 + 1];
  char imsi[15 + 1];
  struct in_addr ipaddr;
};

struct conman_status_wifi_info
{
  char ssid_name[32 + 1];
  int16_t rssi;
  struct in_addr ipaddr;
};

struct conman_status_s
{
  enum conman_connection_type_e conn_type;
  enum conman_status_type_e status;
  bool destroying_prev;

  enum conman_info_type_e info_type;
  union
  {
    struct conman_status_cellu_info cellu;
    struct conman_status_wifi_info wifi;
  } info;
};

/****************************************************************************
 * Name: conman_client_init
 *
 * Description:
 *   Initialize client for communication.
 *
 * Input Parameters:
 *   client : client handle
 *
 * Returned Value:
 *   OK    : no errors
 *   ERROR : failure
 *
 ****************************************************************************/

int conman_client_init(struct conman_client_s *client);

/****************************************************************************
 * Name: conman_client_init_events
 *
 * Description:
 *   Initialize client for communication with boardcast events enabled.
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
                              void *event_priv);

/****************************************************************************
 * Name: conman_client_uninit
 *
 * Description:
 *   Free resources.
 *
 * Input Parameters:
 *   client : client handle
 *
 ****************************************************************************/

void conman_client_uninit(struct conman_client_s *client);

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

int conman_client_handle_events(struct conman_client_s *client);

/****************************************************************************
 * Name: conman_client_set_connections_config
 *
 * Description:
 *   Sends the configuration file to server for parsing.
 *   Format specified in the Thingsee Cloud API.
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
    const char *config);

/****************************************************************************
 * Name: conman_client_request_connection
 *
 * Description:
 *   Sends a connection request to the server.
 *
 * Input Parameters:
 *   client : client handle
 *   type   : connection type
 *   connid : returned connection id (valid connids are in
 *            range CONMAN_CONNID_MIN..CONMAN_CONNID_MAX)
 *
 * Returned Value:
 *   OK    : no errors
 *   ERROR : failure
 *
 ****************************************************************************/

int conman_client_request_connection(struct conman_client_s *client,
                                     enum conman_connection_type_e type,
                                     uint32_t *connid);

/****************************************************************************
 * Name: conman_client_destroy_connection
 *
 * Description:
 *   Request server to destroy connection.
 *
 * Input Parameters:
 *   client : client handle
 *   connid : id of connection to destroy, use CONMAN_CONNID_ALL to
 *            destroy all currently active connections
 *
 * Returned Value:
 *   OK    : no errors
 *   ERROR : failure
 *
 ****************************************************************************/

int conman_client_destroy_connection(struct conman_client_s *client,
                                     uint32_t connid);

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
                                        struct conman_status_s *status);

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
                           const char *phonenumber, const char *message);

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
                              bool mute_speaker, bool mute_mic);

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

int conman_client_call_hangup(struct conman_client_s *client);

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
                                     bool audio_out_on);

#endif /* __APPS_INCLUDE_SYSTEM_CONMAN_H */
