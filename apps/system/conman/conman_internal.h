/****************************************************************************
 * apps/system/conman/conman_internal.h
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

#ifndef __SYSTEM_CONMAN_CONMAN_INTERNAL_H_
#define __SYSTEM_CONMAN_CONMAN_INTERNAL_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <assert.h>
#include <debug.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdbool.h>
#include <sys/stat.h>

#include <apps/system/conman.h>
#include <apps/system/ubmodem.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CONFIG_CONMAN_LISTEN_SOCKET_ADDR        "/dev/conman"
#define CONFIG_CONMAN_CLIENTS_MAX               5

#ifndef offset_of
#define offset_of(type, member) ((intptr_t)(&(((type *)0)->member)))
#endif

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

struct conman_wifi_connection_s
{
  uint32_t id;
  char *ssid;
  char *password;
  char *encryption;
};

struct conman_cellular_connection_s
{
  uint32_t id;
  char *access_point_name;
  char *access_point_ipaddr;
  char *proxy_address;
  char *proxy_port;
  char *user_name;
  char *password;
  char *mobile_country_code;
  char *mobile_network_code;
  char *auth_type;
  char *pin;
  bool enable_roaming;
};

struct conman_connection_entry_s
{
  sq_entry_t entry;

  union
  {
    const struct conman_cellular_connection_s *cellular;
    const struct conman_wifi_connection_s *wifi;
  } connection;
};

struct conman_sd_entry_s
{
  sq_entry_t entry;
  int sd;
  bool events_enabled;
};

struct conman_connid_s
{
  sq_entry_t node;
  uint32_t connid;
  enum conman_connection_type_e type:8;
  bool destroyed:1;
};

struct conman_s
{
  int listensd;
  int servingsd;

  struct
  {
      sq_queue_t sds;
  } server;

  struct
  {
    sq_queue_t cellular;
    sq_queue_t wifi;

    struct
    {
      int cellular;
      int wifi;
    } amount;

    struct
    {
      uint32_t cellular;
      uint32_t wifi;

      sq_queue_t connids;
      uint32_t connid_next;

      uint32_t cellular_refcnt;
      uint32_t wifi_refcnt;
    } current;
  } connections;

  struct
  {
    struct ubmodem_s *modem;
    enum ubmodem_func_level_e target_level;
    enum conman_status_type_e status;

    struct in_addr ipaddr;
    char imei[15 + 1];
    char imsi[15 + 1];
    char udopn[32 + 1];
    char mcc_mnc[7 + 1];

    bool info_requested:1;
    bool imei_requested:1;
    bool imsi_requested:1;
    bool udopn_requested:1;
    bool mcc_mnc_requested:1;
    bool sms_connection_requested:1;
    bool sms_sending_active:1;
    bool establishing_lost:1;

    sq_queue_t sms_queue;
    uint32_t sms_connid;
  } ub;
};

enum conman_msgs_ids
{
  CONMAN_MSG_ENABLE_EVENTS = 1,
  CONMAN_MSG_PING,
  /* Connection control commands: */
  CONMAN_MSG_SET_CONNECTIONS_CONFIG,
  CONMAN_MSG_CREATE_CONNECTION,
  CONMAN_MSG_DESTROY_CONNECTION,
  CONMAN_MSG_GET_CONNECTION_STATUS,
  /* Modem specific commands: */
  CONMAN_MSG_REQUEST_CELL_ENVIRONMENT,
  CONMAN_MSG_SEND_SMS,
  CONMAN_MSG_CALL_ANSWER,
  CONMAN_MSG_CALL_HANGUP,
  CONMAN_MSG_CALL_AUDIO_CONTROL,
  CONMAN_MSG_PLAY_AUDIO_RESOURCE,
  /* u-blox modem specific commands: */
  CONMAN_MSG_START_CELLLOCATE,
  CONMAN_MSG_AID_CELLLOCATE,
  CONMAN_MSG_FTP_DOWNLOAD,
  CONMAN_MSG_FILESYSTEM_DELETE,
  /* Wifi specific commands: */
  CONMAN_MSG_WIFI_SCAN,
};

struct conman_hdr
{
  uint8_t id;
  size_t len;
} packed_struct;

struct conman_resp_hdr
{
  struct conman_hdr head;
  enum conman_resp_value respval;
} packed_struct;

struct conman_req
{
  struct conman_hdr hdr;
  char data[];
} packed_struct;

struct conman_resp
{
  struct conman_hdr hdr;
  uint8_t resp;
} packed_struct;

struct conman_msg_send_sms_s
{
  uint16_t receiver_len;
  uint16_t message_len;
  char sms_data[];
} packed_struct;

struct conman_msg_call_answer_s
{
  bool speaker_mute;
  bool mic_mute;
} packed_struct;

struct conman_msg_call_audioctl_s
{
  bool audio_out_on;
} packed_struct;

struct conman_msg_play_audio_resource_s
{
  struct conman_event_play_audio_resource resource;
} packed_struct;

struct conman_msg_start_celllocate_s
{
  int timeout;
  int target_accuracy;
} packed_struct;

struct conman_msg_aid_celllocate_s
{
  time_t time;
  double latitude;
  double longitude;
  int altitude;
  unsigned int accuracy;
  uint16_t speed;
  uint16_t direction;
} packed_struct;

struct conman_msg_wifi_scan_s
{
  int32_t max_scan_secs;
} packed_struct;

struct conman_msg_ftp_download_s
{
  uint16_t hostname_len;
  uint16_t username_len;
  uint16_t password_len;
  uint16_t filepath_src_len;
  uint16_t filepath_dst_len;
  char msg_data[];
} packed_struct;

int __conman_ubmodem_initialize(struct conman_s *conman, int *maxfds);

int __conman_ubmodem_request_connection(struct conman_s *conman, enum conman_connection_type_e type);

int __conman_ubmodem_get_status_connection(struct conman_s *conman,
                                           struct conman_status_s *status);

void __conman_ubmodem_setup_pollfds(struct conman_s *conman,
                                    struct pollfd *pfds, int maxfds,
                                    int *fds_pos, int *min_timeout);

void __conman_ubmodem_poll_timedout(struct conman_s *conman);

void __conman_ubmodem_handle_pollfds(struct conman_s *conman,
                                     struct pollfd *pfds, int npfds);

bool __conman_ubmodem_is_destroying(struct conman_s *conman);

int __conman_ubmodem_request_cell_environment(struct conman_s *conman);

int __conman_ubmodem_send_sms(struct conman_s *conman,
                              struct conman_msg_send_sms_s *sms);

int __conman_ubmodem_call_answer(struct conman_s *conman,
                                 const struct conman_msg_call_answer_s *mute);

int __conman_ubmodem_call_hangup(struct conman_s *conman);

int __conman_ubmodem_call_audioctl(struct conman_s *conman,
                                   const struct conman_msg_call_audioctl_s *ctl);

int __conman_ubmodem_play_audio_resource(struct conman_s *conman,
                                         const struct conman_msg_play_audio_resource_s *ctl);

int __conman_ubmodem_start_celllocate(struct conman_s *conman,
                                const struct conman_msg_start_celllocate_s *ctl);

int __conman_ubmodem_aid_celllocate(struct conman_s *conman,
                                 const struct conman_msg_aid_celllocate_s *ctl);

int __conman_ubmodem_ftp_download(struct conman_s *conman,
                                  struct conman_msg_ftp_download_s *ftp);

int __conman_ubmodem_filesystem_delete(struct conman_s *conman,
                                       const char *filename);

int __conman_cc3000_initialize(struct conman_s *conman);

int __conman_cc3000_request_connection(struct conman_s *conman,
                                       enum conman_connection_type_e type);

int __conman_cc3000_get_status_connection(struct conman_s *conman,
                                          struct conman_status_s *status);

bool __conman_cc3000_is_destroying(struct conman_s *conman);

int __conman_cc3000_wifi_scan(struct conman_s *conman);

unsigned int __conman_cc3000_get_max_pollfds(struct conman_s *conman);

void __conman_cc3000_setup_pollfds(struct conman_s *conman,
                                   struct pollfd *pfds, int maxfds,
                                   int *fds_pos, int *min_timeout);

void __conman_cc3000_handle_pollfds(struct conman_s *conman,
                                    struct pollfd *pfds);

void __conman_config_init(struct conman_s *conman);

int __conman_config_set_connections(struct conman_s *conman,
    const char *config);

bool __conman_config_modem_config_cb(struct ubmodem_s *modem,
    const char *variable, char *buf, size_t buflen, void *priv);

const struct conman_wifi_connection_s *
__conman_config_current_wifi_connection(struct conman_s *conman);

void __conman_ctl_handle_pollin(struct conman_s *conman, struct pollfd *pfd);

int __conman_ctl_create_connection(struct conman_s *conman,
                                   enum conman_connection_type_e type,
                                   uint32_t *connid);

int __conman_ctl_destroy_connection(struct conman_s *conman,
                                    uint32_t connid);

int __conman_ctl_get_status_connection(struct conman_s *conman,
                                       struct conman_status_s *status);

int __conman_client_init_fifos(struct conman_s *conman);

int __conman_util_block_read(int fd, void *buffer, size_t len);

int __conman_util_block_write(int fd, const void * const buffer, size_t len);

int __conman_send_resp(struct conman_s *conman, uint8_t id,
                       enum conman_resp_value respval, void *data,
                       size_t datalen);

void __conman_send_boardcast_event(struct conman_s *conman,
                                   enum conman_msgs_ids type,
                                   const void *payload, size_t payloadlen);

void __conman_flushfd(int fd);

int __conman_server_initialize(struct conman_s *conman, int *max_pfds);

void __conman_server_handle_pollin(struct conman_s *conman, struct pollfd *pfd);

void __conman_server_setup_pollfds(struct conman_s *conman,
                                   struct pollfd *pfds, int maxfds,
                                   int *fds_pos, int *min_timeout);

int __conman_server_close_sd(struct conman_s *conman, int sd);

int __conman_server_enable_events_for_client(struct conman_s *conman, int sd);

#endif /* __SYSTEM_CONMAN_CONMAN_INTERNAL_H_ */

