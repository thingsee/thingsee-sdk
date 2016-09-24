/****************************************************************************
 * apps/include/system/conman.h
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

#ifndef __APPS_INCLUDE_SYSTEM_CONMAN_H
#define __APPS_INCLUDE_SYSTEM_CONMAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <netinet/in.h>
#include <time.h>

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
  /* Common events: */
  CONMAN_EVENT_LOST_CONNECTION = 1,
  CONMAN_EVENT_CONNECTION_REQUEST_FAILED,
  CONMAN_EVENT_CONNECTION_ESTABLISHED,
  CONMAN_EVENT_HW_ERROR_TOO_LOW_VOLTAGE,
  /* Modem events: */
  CONMAN_EVENT_CELL_ENVIRONMENT,
  CONMAN_EVENT_CALL_INCOMING,
  CONMAN_EVENT_CALL_ACTIVE,
  CONMAN_EVENT_CALL_DISCONNECTED,
  /* u-blox modem specific events: */
  CONMAN_EVENT_CELLLOCATE,
  CONMAN_EVENT_FTP_DOWNLOAD_STATUS,
  /* Wifi events: */
  CONMAN_EVENT_WIFI_SCAN,
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
} packed_struct;

struct conman_event_celllocate_info
{
  bool have_time:1;         /* Got GPS time. */
  bool have_location:1;     /* Got location. */
  unsigned int accuracy:30; /* Accuracy estimate in meters. */
  int altitude;             /* Height above mean sea level in meters. */
  double latitude;          /* Latitude as degrees. */
  double longitude;         /* Longitude as degrees. */
  time_t gps_time;          /* GPS time. */
} packed_struct;

struct conman_event_ftp_download_status
{
  bool file_downloaded;
} packed_struct;

enum conman_event_cell_environment_type
{
  CONMAN_CELL_ENVIRONMENT_TYPE_UNKNOWN = 0,
  CONMAN_CELL_ENVIRONMENT_TYPE_GSM,
  CONMAN_CELL_ENVIRONMENT_TYPE_UMTS,
};

struct conman_event_cell_environment_signal_qual_s
{
  int16_t rssi; /* dBm */
  int8_t qual;  /* range: 0..7 */
} packed_struct;

struct conman_event_cell_environment_serving_s
{
  enum conman_event_cell_environment_type type;
  uint32_t cell_id;   /* UMTS 28-bit, GSM 16-bit */
  uint16_t mcc;
  uint16_t mnc;
  uint16_t lac;
  uint8_t bsic;
  uint16_t arfcn;
  uint16_t sc;        /* UMTS only */
  int16_t signal_dbm; /* UMTS: "-115 + <rscp_lev>", GSM: "-110 + <rxlev>" */
} packed_struct;

struct conman_event_cell_environment_neighbor_s
{
  enum conman_event_cell_environment_type type:3;
  bool have_mcc_mnc_lac:1;
  bool have_cellid:1;
  bool have_sc:1;
  bool have_signal_dbm:1;
  bool have_bsic:1;
  bool have_arfcn:1;
  uint16_t mcc;
  uint16_t mnc;
  uint16_t lac;
  uint32_t cell_id;
  uint8_t bsic;
  uint16_t arfcn;
  uint16_t sc;        /* UMTS only */
  int16_t signal_dbm; /* UMTS: "-115 + <rscp_lev>", GSM: "-110 + <rxlev>" */
} packed_struct;

struct conman_event_cell_environment_s
{
  bool have_signal_qual:1;
  bool have_serving:1;
  uint8_t num_neighbors;

  struct conman_event_cell_environment_signal_qual_s signal_qual;
  struct conman_event_cell_environment_serving_s serving;
  struct conman_event_cell_environment_neighbor_s neighbors[];
} packed_struct;

struct conman_event_wifi_scan_entry_s
{
  uint8_t ssid_len;
  int8_t rssi;
  char ssid[32];
  uint8_t bssid[6];
} packed_struct;

struct conman_event_wifi_scan_results_s
{
  uint8_t num_results;
  struct conman_event_wifi_scan_entry_s results[];
} packed_struct;

struct conman_status_cellu_info
{
  char oper_name[32 + 1];
  char mcc_mnc[7 + 1];
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
                                    char *filename);

/****************************************************************************
 * Name: conman_client_ftp_download
 *
 * Description:
 *   Retrieve a file from FTP server.
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
                               char *filepath_dst);

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
                                   int timeout, int target_accuracy);

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
                                 unsigned int speed, unsigned int direction);

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

int conman_client_request_cell_environment(struct conman_client_s *client);

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

int conman_client_ping(struct conman_client_s *client);

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

int conman_client_wifi_scan(struct conman_client_s *client);

#endif /* __APPS_INCLUDE_SYSTEM_CONMAN_H */
