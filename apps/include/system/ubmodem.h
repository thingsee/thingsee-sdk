/****************************************************************************
 * apps/include/system/ubmodem.h
 *
 *   Copyright (C) 2014-2017 Haltian Ltd. All rights reserved.
 *   Author: Jussi Kivilinna <jussi.kivilinna@haltian.com>
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

#ifndef __APPS_INCLUDE_SYSTEM_UBMODEM_H
#define __APPS_INCLUDE_SYSTEM_UBMODEM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <stdbool.h>
#include <netinet/in.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Maximum filename length (as defined in u-blox AT command manual). */

#define UBMODEM_FILESYSTEM_NAME_MAX_LEN 47

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

/* Forward declaration of modem data structures. */

struct ubmodem_s;

/* Modem module functionality levels */

enum ubmodem_func_level_e
{
  UBMODEM_LEVEL_POWERED_OFF = 0,      /* Powered off */
  UBMODEM_LEVEL_STUCK_HARDWARE,       /* Modem hardware error level */
  UBMODEM_LEVEL_POWERED_ON,           /* Powered on (initial state after boot) */
  UBMODEM_LEVEL_CMD_PROMPT,           /* AT command prompt initialized */
  UBMODEM_LEVEL_SIM_ENABLED,          /* SIM card set up / PIN entered */
  UBMODEM_LEVEL_NETWORK,              /* GSM network connected */
  UBMODEM_LEVEL_GPRS,                 /* GPRS/Internet connected */
  __UBMODEM_LEVEL_MAX
};

/* Event types as flags */

enum ubmodem_event_flags_e
{
  UBMODEM_EVENT_FLAG_NEW_LEVEL               = 1 << 0,
  UBMODEM_EVENT_FLAG_TARGET_LEVEL_REACHED    = 1 << 1,
  UBMODEM_EVENT_FLAG_FAILED_LEVEL_TRANSITION = 1 << 2,
  UBMODEM_EVENT_FLAG_IP_ADDRESS              = 1 << 3,
  UBMODEM_EVENT_FLAG_CELL_LOCATION           = 1 << 4,
  UBMODEM_EVENT_FLAG_CELL_ENVIRONMENT        = 1 << 5,
  UBMODEM_EVENT_FLAG_CALL_RINGING            = 1 << 6,
  UBMODEM_EVENT_FLAG_CALL_ACTIVE             = 1 << 7,
  UBMODEM_EVENT_FLAG_CALL_DISCONNECTED       = 1 << 8,
  UBMODEM_EVENT_FLAG_FTP_DOWNLOAD_STATUS     = 1 << 9,
  UBMODEM_EVENT_FLAG_FILESYSTEM_OPER_COMPLETE= 1 << 10,
  UBMODEM_EVENT_FLAG_ERROR                   = 1 << 11,

  /* Debug traces. */

  UBMODEM_EVENT_FLAG_TRACE_USRSOCK           = 1 << 26,
  UBMODEM_EVENT_FLAG_TRACE_CMD_TO_MODEM      = 1 << 27,
  UBMODEM_EVENT_FLAG_TRACE_RESP_FROM_MODEM   = 1 << 28,
  UBMODEM_EVENT_FLAG_TRACE_STATE_CHANGE      = 1 << 29,
  UBMODEM_EVENT_FLAG_TRACE_DATA_TO_MODEM     = 1 << 30,
  UBMODEM_EVENT_FLAG_TRACE_DATA_FROM_MODEM   = 1 << 31,
};

/* Error event types */

enum ubmodem_error_event_e
{
  UBMODEM_ERROR_STARTUP_VOLTAGE_TOO_LOW      = 0,
};

/* Filesystem operation types */

enum ubmodem_filesystem_oper_e
{
  MODEM_FILESYSTEM_DELETE = 1,
};

/* Info types for 'ubmodem_get_info' */

typedef enum ubmodem_info_type
{
  UB_INFO_IMEI = 0,
  UB_INFO_IMSI,
  UB_INFO_ICCID,
  UB_INFO_MSISDN_1,
  UB_INFO_UDOPN,
  UB_INFO_MCC_MNC,
} ubmodem_info_type_t;

/* Event data structure for UBMODEM_EVENT_FLAG_NEW_LEVEL. */

struct ubmodem_event_new_level_s
{
  enum ubmodem_func_level_e new_level;
  enum ubmodem_func_level_e old_level;
};

/* Event data structure for UBMODEM_EVENT_FLAG_TARGET_LEVEL_REACHED. */

struct ubmodem_event_target_level_reached_s
{
  enum ubmodem_func_level_e new_level;
};

/* Event data structure for UBMODEM_EVENT_FLAG_FAILED_LEVEL_TRANSITION. */

struct ubmodem_event_transition_failed_s
{
  enum ubmodem_func_level_e current_level;
  enum ubmodem_func_level_e target_level;
  const char *reason;
};

/* Event data structure for UBMODEM_EVENT_FLAG_IP_ADDRESS. */

struct ubmodem_event_ip_address_s
{
  struct in_addr ipaddr; /* Current IP address for GPRS connection. */
  struct in_addr dns1;   /* Primary DNS server for GPRS connection. */
  struct in_addr dns2;   /* Secondary DNS server for GPRS connection. */
};

/* Event data structure for UBMODEM_EVENT_FLAG_CALL_RINGING. */

struct ubmodem_voice_call_ringing_s
{
  short number_type;
  short subaddress_type;
  char number[20];
  char subaddress[20];
};

/* Event subtypes for UBMODEM_EVENT_FLAG_TRACE_USRSOCK. */

enum ubmodem_usrsock_trace_types_e
{
  UBMODEM_TRACE_USRSOCK_REQ = 0,
  UBMODEM_TRACE_USRSOCK_RESP = 1,
  UBMODEM_TRACE_USRSOCK_DATARESP = 2,
  UBMODEM_TRACE_USRSOCK_EVENT = 3,
};

/* Event data structure for UBMODEM_EVENT_FLAG_CELL_LOCATION. */

struct ubmodem_event_cell_location_s
{
  struct
  {
    /* Got date/time? */

    bool valid;

    /* Year */

    int year;

    /* Month */

    int month;

    /* Day */

    int day;

    /* Hour */

    int hour;

    /* Minutes */

    int min;

    /* Seconds */

    int sec;
  } date packed_struct;

  struct
  {
    /* Got location? */

    bool valid;

    /* Latitude as degrees */

    double latitude;

    /* Longitude as degrees */

    double longitude;

    /* Height above mean sea level in meters */

    int32_t altitude;

    /* Accuracy estimate in meters */

    int32_t accuracy;
  } location packed_struct;
} packed_struct;

/* Event data structure for UBMODEM_EVENT_FLAG_CELL_ENVIRONMENT. */

enum ubmodem_rat_type_e
{
  UBMODEM_RAT_GSM = 0,
  UBMODEM_RAT_UMTS,
  __UBMODEM_RAT_MAX,
};

struct ubmodem_event_cell_env_neighbor_s
{
  bool have_mcc_mnc_lac:1;
  bool have_cellid:1;
  bool have_bsic:1;
  bool have_arfcn:1;
  bool have_sc:1;
  bool have_signal_dbm:1;
  enum ubmodem_rat_type_e rat;
  uint16_t mcc;
  uint16_t mnc;
  uint16_t lac;
  uint32_t cell_id;
  uint8_t bsic; /* GSM only */
  uint16_t arfcn;
  uint16_t sc; /* UMTS only */
  /* UMTS: "-115 + <rscp_lev>", GSM: "-110 + <rxlev>" */
  int16_t signal_dbm;
} packed_struct;

#define UBMODEM_MAX_CELL_ENVIRONMENT_NEIGHBORS 32

struct ubmodem_event_cell_environment_s
{
  bool have_signal_qual:1;
  bool have_serving:1;

  struct
  {
    int16_t rssi; /* dBm */
    int8_t qual;  /* range: 0..7 */
  } signal_qual;

  struct
  {
    enum ubmodem_rat_type_e rat;
    uint16_t mcc;
    uint16_t mnc;
    uint16_t lac;
    uint32_t cell_id;
    uint8_t bsic; /* GSM only */
    uint16_t arfcn;
    uint16_t sc; /* UMTS only */
    /* UMTS: "-115 + <rscp_lev>", GSM: "-110 + <rxlev>" */
    int16_t signal_dbm;
  } serving packed_struct;

  uint8_t num_neighbors;
  /* Max: UBMODEM_MAX_CELL_ENVIRONMENT_NEIGHBORS */
  struct ubmodem_event_cell_env_neighbor_s neighbors[];
} packed_struct;

/* Event data structure for UBMODEM_EVENT_FLAG_FTP_DOWNLOAD_STATUS. */

struct ubmodem_event_ftp_download_status_s
{
  bool file_downloaded;
  void *priv;
} packed_struct;

enum ubmodem_tcp_conn_event_e
{
  UBMODEM_TCP_EVENT_CONNECTED = 0,
  UBMODEM_TCP_EVENT_CONNECT_FAILED,
  UBMODEM_TCP_EVENT_OUTBUF_SENT,
  UBMODEM_TCP_EVENT_INPUT_DATA,
  UBMODEM_TCP_EVENT_CLOSED
};

/* Filesystem operation structure, event data structure for
 * UBMODEM_EVENT_FLAG_FILESYSTEM_OPER_COMPLETE. */

struct ubmodem_filesystem_oper_s
{
  enum ubmodem_filesystem_oper_e type;
  int result;
  union
  {
    struct
    {
      char name[UBMODEM_FILESYSTEM_NAME_MAX_LEN + 1];
    } delete;
  } oper;
} packed_struct;


/* Configuration callback function type, used to fetch configuration settings
 * for modem.
 *
 * Input parameters:
 *   name     : Configuration item name, currently in use:
 *              "modem.pin"
 *              "modem.apn_name"
 *              "modem.apn_user"
 *              "modem.apn_password"
 *              "modem.apn_ipaddr"
 *   buf      : Output buffer for string value
 *   buflen   : Output buffer max length
 *   priv     : Private data pointer
 * Returns:
 *   True, if configuration item was found and loaded to 'buf'.
 */

typedef bool (* ubmodem_config_fn_t)(struct ubmodem_s *modem,
                                      const char *name, char *buf,
                                      size_t buflen, void *priv);

/* Modem event callback function type */

typedef void (*ubmodem_event_func_t)(struct ubmodem_s *modem,
                                   enum ubmodem_event_flags_e event,
                                   const void *event_data, size_t datalen,
                                   void *priv);

/* Send SMS callback function type */

typedef void (*ubmodem_send_sms_cb_t)(struct ubmodem_s *modem,
                                      bool sms_sent,
                                      void *priv);

/* Info callback function type */

typedef void (*ubmodem_info_cb_t)(void *priv, const char *data, int datalen,
                                  bool status, ubmodem_info_type_t type);

/* Modem hardware control power-management activity levels. */

enum ubmodem_hw_pm_activity_e
{
  UBMODEM_PM_ACTIVITY_LOW = 0,
  UBMODEM_PM_ACTIVITY_HIGH,
};

/* Modem hardware control operations structure. */

struct ubmodem_hw_ops_s
{
  /**************************************************************************
   * Name: ubmodem_hw_ops_s->initialize
   *
   * Description:
   *   Initialize the u-blox modem.
   *
   * Input Parameters:
  *    is_vcc_off:   Value is set 'true' if modem is fully powered off at
  *                  start-up.
  *
   * Return:
   *   Opened modem serial port file descriptor.
   *
   **************************************************************************/

  int (*initialize)(void *priv, bool *is_vcc_off);

  /**************************************************************************
   * Name: ubmodem_hw_ops_s->deinitialize
   *
   * Description:
   *   Deinitialize the u-blox modem and close serial_fd.
   *
   **************************************************************************/

  int (*deinitialize)(void *priv, int serial_fd);

  /**************************************************************************
   * Name: ubmodem_hw_ops_s->vcc_set
   *
   * Description:
   *   Set modem VCC.
   *
   * Input Parameters:
   *   on:   New requested VCC on/off state.
   *
   * Return:
   *   Return new VCC state.
   *
   **************************************************************************/

  bool (*vcc_set)(void *priv, bool on);

  /**************************************************************************
   * Name: ubmodem_hw_ops_s->poweron_pin_set
   *
   * Description:
   *   Control POWER_ON gpio.
   *
   * Input Parameters:
   *   set:  enable or disable pin.
   *
   * Return value:
   *   Recommended time in milliseconds to sleep after setting pin for proper
   *   operation of modem.
   *
   **************************************************************************/

  uint32_t (*poweron_pin_set)(void *priv, bool set);

  /**************************************************************************
   * Name: ubmodem_hw_ops_s->reset_pin_set
   *
   * Description:
   *   Control RESET_N gpio.
   *
   * Input Parameters:
   *   set:  enable or disable pin.
   *
   * Return value:
   *   Recommended time in milliseconds to sleep after setting pin for proper
   *   operation of modem.
   *
   **************************************************************************/

  uint32_t (*reset_pin_set)(void *priv, bool set);

  /**************************************************************************
   * Name: ubmodem_hw_ops_s->pm_set_activity
   *
   * Description:
   *   Control low/high activity state for power-management
   *
   * Input Parameters:
   *   type: activity type (high/low)
   *   set:  set activity or unset activity by one step
   *
   **************************************************************************/

  void (*pm_set_activity)(void *priv, enum ubmodem_hw_pm_activity_e type,
                          bool set);
};

/* Modem FTP download parameters structure. */

struct ubmodem_ftp_download_s
{
  /* FTP server hostname */

  const char *hostname;

  /* Login user-name */

  const char *username;

  /* Login password */

  const char *password;

  /* Path to source file on FTP server */

  const char *filepath_src;

  /* Path to destination file on modem internal filesystem */

  const char *filepath_dst;

  /* Caller private structure */

  void *priv;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ubmodem_start_cell_locate
 *
 * Description:
 *   Ask modem to perform CellLocate work. Modem must be in GPRS level.
 *
 * Input Parameters:
 *   modem           : Modem structure
 *   locate_timeout  : Timeout period (in seconds)
 *   allowed_accuracy: Target accuracy (in meters)
 *
 * Returned valued:
 *   ERROR if failed, OK on success.
 *
 ****************************************************************************/

int ubmodem_start_cell_locate(struct ubmodem_s *modem, int locate_timeout,
                              int target_accuracy);

/****************************************************************************
 * Name: ubmodem_cell_locate_give_location_aid
 *
 * Description:
 *   Give modem external location information to aid Cell-Locate (+ULOCAID).
 *   Modem will send this information to u-blox CellLocate servers and
 *   potentially improve CellLocate database over time.
 *
 * Input Parameters:
 *   modem    : Modem structure
 *   time     : UTC timestamp of aided location
 *   latitude : Estimated latitude in degrees
 *   longitude: Estimated longitude in degrees
 *   altitude : Estimated altitude in meters
 *   accuracy : Estimated accuracy in meters
 *   speed    : Estimated speed in meters per second
 *   direction: Estimated direction of speed in degrees (north zero, clockwise)
 *
 * Returned valued:
 *   ERROR if failed, OK on success.
 *
 ****************************************************************************/

int ubmodem_cell_locate_give_location_aid(struct ubmodem_s *modem,
                                          time_t time, double latitude,
                                          double longitude, int altitude,
                                          unsigned int accuracy,
                                          unsigned int speed,
                                          unsigned int direction);

/****************************************************************************
 * Name: ubmodem_request_cell_environment
 *
 * Description:
 *   Ask cell environment information from modem
 *
 * Input Parameters:
 *   modem           : Modem structure
 *
 * Returned valued:
 *   ERROR if failed, OK on success.
 *
 ****************************************************************************/

int ubmodem_request_cell_environment(struct ubmodem_s *modem);

/****************************************************************************
 * Name: ubmodem_register_event_listener
 *
 * Description:
 *   Register modem event listener.
 *
 * Input Parameters:
 *   event_types   : Event type flags this callback listens for.
 *   event_callback: Callback function for the event.
 *   callback_priv : Private data for callback.
 *
 * Returned Value:
 *   OK: Success.
 *   ERROR: Failed.
 *
 ****************************************************************************/

int ubmodem_register_event_listener(struct ubmodem_s *modem,
                                     uint32_t event_types,
                                     ubmodem_event_func_t event_callback,
                                     void *callback_priv);

/****************************************************************************
 * Name: ubmodem_unregister_event_listener
 *
 * Description:
 *   Unregister modem event listener.
 *
 * Input Parameters:
 *   event_callback: Callback function for the event.
 *
 * Returned Value:
 *   OK: Success.
 *   ERROR: Failed.
 *
 ****************************************************************************/

int ubmodem_unregister_event_listener(struct ubmodem_s *modem,
                                       ubmodem_event_func_t event_callback);

/****************************************************************************
 * Name: ubmodem_request_level
 *
 * Description:
 *   Request functionality level from modem.
 *
 * Input Parameters:
 *   level   : Functional level requested for modem.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void ubmodem_request_level(struct ubmodem_s *modem,
                            enum ubmodem_func_level_e level);

/****************************************************************************
 * Name: ubmodem_set_config_callback
 *
 * Description:
 *   Setup configuration callback for modem.
 *
 * Input Parameters:
 *   config_cb : Configuration callback
 *   priv      : Private data pointer for callback
 *
 ****************************************************************************/

void ubmodem_set_config_callback(struct ubmodem_s *modem,
                                  ubmodem_config_fn_t config_cb, void *priv);

/****************************************************************************
 * Name: ubmodem_initialize
 *
 * Description:
 *   Initialization for modem library
 *
 * Input Parameters:
 *   hw_ops           : Modem HW control function structure
 *   hw_ops_priv      : Private data pointer for HW control functions
 *
 * Returned Values:
 *   Not NULL means the function was executed successfully
 *   NULL means the function was executed unsuccessfully
 *
 ****************************************************************************/

struct ubmodem_s *ubmodem_initialize(const struct ubmodem_hw_ops_s *hw_ops,
                                     void *hw_ops_priv);

/****************************************************************************
 * Name: ubmodem_uninitialize
 *
 * Description:
 *   Uninitialization for modem library
 *
 * Input Parameters:
 *   modem:  Pointer for modem library object from ubmodem_initialize
 *
 * Returned Values:
 *   None.
 *
 ****************************************************************************/

void ubmodem_uninitialize(struct ubmodem_s *modem);

/****************************************************************************
 * Name: ubmodem_get_info
 *
 * Description:
 *   Get information from modem and call callback when receive it.
 *
 * Input Parameters:
 *   result_cb     : Callback function to call when info available.
 *   type          : Info type to receive.
 *
 ****************************************************************************/

int ubmodem_get_info(struct ubmodem_s *modem, ubmodem_info_cb_t result_cb,
                     void *priv, ubmodem_info_type_t type);

#ifdef CONFIG_UBMODEM_SMS_ENABLED

/****************************************************************************
 * Name: ubmodem_send_sms
 *
 * Description:
 *   Attempt to send SMS.
 *
 * Input Parameters:
 *   modem     : Pointer for modem library object from ubmodem_initialize
 *   receiver  : Phone number to send SMS
 *   message   : Text message to send, UTF-8 encoded
 *   result_cb : SMS sent/failed result callback
 *   result_priv: private data for callback
 *
 * Returned valued:
 *   ERROR if failed, OK on success.
 *
 ****************************************************************************/

int ubmodem_send_sms(struct ubmodem_s *modem, const char *receiver,
                     const char *message, ubmodem_send_sms_cb_t result_cb,
                     void *result_priv);

#endif /* CONFIG_UBMODEM_SMS_ENABLED */

#ifdef CONFIG_UBMODEM_FTP_ENABLED

/****************************************************************************
 * Name: ubmodem_ftp_download_file
 *
 * Description:
 *   Retrieve a file from FTP server.
 *
 * Input Parameters:
 *   modem : Pointer for modem library object from ubmodem_initialize
 *   ftp   : FTP configuration parameters
 *   priv  : caller private data pointer
 *
 * Returned valued:
 *   ERROR if failed, OK on success.
 *
 ****************************************************************************/

int ubmodem_ftp_download_file(struct ubmodem_s *modem,
                              struct ubmodem_ftp_download_s *ftp, void *priv);

#endif /* CONFIG_UBMODEM_FTP_ENABLED */

/****************************************************************************
 * Name: ubmodem_filesystem_delete
 *
 * Description:
 *   Delete file in modem filesystem
 *
 * Input Parameters:
 *   modem           : Modem structure
 *   filename        : Name of file to delete
 *
 * Returned valued:
 *   ERROR if failed, OK on successfully start of task.
 *
 ****************************************************************************/

int ubmodem_filesystem_delete(struct ubmodem_s *modem, const char *filename);

#ifdef CONFIG_UBMODEM_VOICE

/****************************************************************************
 * Name: ubmodem_voice_answer
 *
 * Description:
 *   Answer pending voice-call
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void ubmodem_voice_answer(struct ubmodem_s *modem, bool mute_mic,
                          bool mute_speaker);

/****************************************************************************
 * Name: ubmodem_voice_hangup
 *
 * Description:
 *   Hang-up pending or active voice-call
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void ubmodem_voice_hangup(struct ubmodem_s *modem);

/****************************************************************************
 * Name: ubmodem_audio_setup
 *
 * Description:
 *   Audio setup, enable/disable OUT/IN.
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void ubmodem_audio_setup(struct ubmodem_s *modem, bool out_on, bool in_on);

/****************************************************************************
 * Name: ubmodem_play_audio_resource
 *
 * Description:
 *   Play audio resource (see documentation for AT+UPAR)
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

int ubmodem_play_audio_resource(struct ubmodem_s *modem,
                                unsigned int audio_resource,
                                unsigned int tone_id,
                                unsigned int nof_repeats);

/****************************************************************************
 * Name: ubmodem_stop_audio_resource
 *
 * Description:
 *   Stop audio resource (see documentation for AT+USAR)
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

int ubmodem_stop_audio_resource(struct ubmodem_s *modem,
                                unsigned int audio_resource);

#endif /* CONFIG_UBMODEM_VOICE */

/****************************************************************************
 * Name: ubmodem_get_func_level
 *
 * Description:
 *   Read current functionality level of modem
 *
 ****************************************************************************/

enum ubmodem_func_level_e ubmodem_get_func_level(struct ubmodem_s *modem);

/****************************************************************************
 * Name: ubmodem_is_powered_off
 *
 * Description:
 *   Check if modem in powered-off state and internal state machine is not
 *   active.
 *
 * Input Parameters:
 *   None.
 *
 ****************************************************************************/

bool ubmodem_is_powered_off(struct ubmodem_s *modem);

/****************************************************************************
 * Name: ubmodem_setup_poll
 *
 * Description:
 *   Setup pollfd structure and timeout value for ubmodem library
 *
 * Input Parameters:
 *   None.
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/

int ubmodem_poll_setup(struct ubmodem_s *modem, struct pollfd *pfd,
                       int *timeout);

/****************************************************************************
 * Name: ubmodem_pollfds_setup
 *
 * Description:
 *   Setup pollfd structure and timeout value for ubmodem library
 *
 * Input Parameters:
 *   None.
 *
 * Returned Values:
 *   Number of setup pollfds.
 *
 ****************************************************************************/

int ubmodem_pollfds_setup(struct ubmodem_s *modem, struct pollfd *pfd,
                          int numpfds, int *timeout);

/****************************************************************************
 * Name: ubmodem_poll_event
 *
 * Description:
 *   Indicate poll event for ubmodem library. Library will handle current
 *   pending I/O and internal state changes.
 *
 * Input Parameters:
 *   pfd: Poll structure for ubmodem, setup with ubmodem_poll_setup
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/

int ubmodem_poll_event(struct ubmodem_s *modem, struct pollfd *pfd);

/****************************************************************************
 * Name: ubmodem_pollfds_event
 *
 * Description:
 *   Indicate poll event for ubmodem library. Library will handle current
 *   pending I/O and internal state changes.
 *
 * Input Parameters:
 *   pfd: Poll structure array for ubmodem, setup with ubmodem_poll_setup
 *   numfds: Number of poll structures
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/

int ubmodem_pollfds_event(struct ubmodem_s *modem, struct pollfd *pfd,
                          int numfds);

/****************************************************************************
 * Name: ubmodem_poll_timedout
 *
 * Description:
 *   Indicate that poll timed out. This allows library to handle internal
 *   timed state changes.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/

int ubmodem_poll_timedout(struct ubmodem_s *modem);

/****************************************************************************
 * Name: ubmodem_poll_max_fds
 *
 * Description:
 *   Maximum number of pollfds ubmodem can setup in ubmodem_poll_setup.
 ****************************************************************************/

int ubmodem_poll_max_fds(struct ubmodem_s *modem);

#endif /* __APPS_INCLUDE_SYSTEM_UBMODEM_H */
