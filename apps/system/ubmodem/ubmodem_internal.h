/****************************************************************************
 * apps/system/ubmodem/ubmodem_internal.h
 *
 *   Copyright (C) 2014-2016 Haltian Ltd. All rights reserved.
 *   Author: Jussi Kivilinna <jussi.kivilinna@haltian.com>
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

#ifndef __SYSTEM_UBMODEM_UBMODEM_INTERNAL_H_
#define __SYSTEM_UBMODEM_UBMODEM_INTERNAL_H_

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

#include <apps/system/ubmodem.h>

#include "ubmodem_parser.h"
#include "ubmodem_command.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Maximum sockets on Sara-G350 is 7 (0..6). */

#define MODEM_MAX_SOCKETS_OPEN 7

/* Maximum binary socket write size is 1kB. */

#define MODEM_MAX_BINARY_SOCKET_WRITE_BYTES 1024

/* Maximum binary socket read size is 1kB. */

#define MODEM_MAX_BINARY_SOCKET_READ_BYTES 1024

/* Limit for UDP packet payload size. */

#define UBMODEM_USRSOCK_UDP_MAX_PACKET_PAYLOAD 1024

/* How many times to retry configure setting. */

#define UBMODEM_CONFIGURE_RETRIES 5

#define MODEM_DEBUGASSERT(modem, a) ({ \
    if (!(a)) \
      { \
        __ubmodem_assert_debug_print(modem); \
        DEBUGASSERT(a); \
      } \
  })

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

#ifndef offset_of
#define offset_of(type, member) ((intptr_t)(&(((type *)0)->member)))
#endif

#ifndef container_of
#define container_of(ptr, type, member) \
        ((type *)((intptr_t)(ptr) - offset_of(type, member)))
#endif

/* Preprocessor magic to convert __LINE__ to string. */

#ifndef STRINGIFY
#  define STRINGIFY(x) #x
#endif
#ifndef TOSTRING
#  define TOSTRING(x) STRINGIFY(x)
#endif

/* Verbose debugging */

#ifdef CONFIG_UBMODEM_DEBUG_VERBOSE
#  define ubdbg(...) dbg(__VA_ARGS__)
#else
#  define ubdbg(...) do;while(0)
#endif

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

/* Modem state machine */

enum modem_state_e
{
  MODEM_STATE_WAITING = 0,
  MODEM_STATE_IN_SUBSTATE,
  MODEM_STATE_DO_POWER_ON,
  MODEM_STATE_DO_POWER_OFF,
  MODEM_STATE_DO_RESET,
  MODEM_STATE_SETUP_CMD_PROMPT,
  MODEM_STATE_SETUP_SIM,
  MODEM_STATE_SETUP_NETWORK,
  MODEM_STATE_DISCONNECT_NETWORK,
  MODEM_STATE_OPEN_GPRS_CONNECTION,
  MODEM_STATE_CLOSE_GPRS_CONNECTION,
  MODEM_STATE_DO_SOCKET_WORK,
  MODEM_STATE_DO_TASK_WORK,
  MODEM_STATE_FAKE_STUCK_HARDWARE,
};

/* Network substate machine */

enum ubmodem_network_state_e
{
  NETWORK_SETUP_DISCONNECTING = 1,
  NETWORK_SETUP_ENABLING_CREG,
  NETWORK_SETUP_ENABLING_RF,
  NETWORK_SETUP_STARTING_NETWORK_REGISTRATION,
  NETWORK_SETUP_WAITING_NETWORK_REGISTRATION,
  NETWORK_SETUP_RETRYING_NETWORK_REGISTRATION
};

/* Modem models */

enum ubmodem_model_e
{
  UBMODEM_MODEL_UNKNOWN = 0,
  UBMODEM_MODEL_SARA_G_UNKNOWN,
  UBMODEM_MODEL_SARA_G300,
  UBMODEM_MODEL_SARA_G310,
  UBMODEM_MODEL_SARA_G340,
  UBMODEM_MODEL_SARA_G350,
  UBMODEM_MODEL_SARA_U_UNKNOWN,
  UBMODEM_MODEL_SARA_U260,
  UBMODEM_MODEL_SARA_U270,
  UBMODEM_MODEL_SARA_U280,
};

/* CellLocate ULOCAID support status */

enum ubmodem_cell_locate_aid_support_e
{
  UBMODEM_CELL_LOCATE_SUPPORT_UNKNOWN = 0,
  UBMODEM_CELL_LOCATE_SUPPORT_OK,
  UBMODEM_CELL_LOCATE_SUPPORT_NOK,
};

/* Timer callback function type. */

typedef int (* ubmodem_timer_fn_t)(struct ubmodem_s *modem, int timer_id,
                                   void * const arg);

/* Task function type. */

typedef int (* ubmodem_task_fn_t)(struct ubmodem_s *modem, void *priv);

/* Raw command task function type. */

typedef void (*modem_command_task_callback_t)(const struct at_cmd_def_s *cmd,
                                              const struct at_resp_info_s *info,
                                              const uint8_t *resp_stream,
                                              size_t stream_len,
                                              void *priv);

/* Configuration command function type. */

typedef void (* ubmodem_send_config_func_t)(struct ubmodem_s *modem);

/* Configuration command group function type. */

typedef const ubmodem_send_config_func_t *(* ubmodem_configs_func_t)(
    struct ubmodem_s *modem, size_t *ncmds);

/* Check CMEE setting result function type. */

typedef void (*modem_check_cmee_func_t)(struct ubmodem_s *modem, bool cmee_ok,
                                        int cmee_setting, void *priv);

/* Modem module data structure */

struct ubmodem_s {
  /* Modem state */

  enum ubmodem_func_level_e level:4;
  enum ubmodem_func_level_e target_level:4;
  enum ubmodem_func_level_e force_level:4;
  enum ubmodem_func_level_e intermediate_level:4;

  enum modem_state_e state:5;

  /* Flags */

  bool initialized:1;       /* Is module initialized? */
  bool is_nonblocking:1;    /* Is serial_fd currently setup as non-blocking? */
  bool is_powered_off:1;    /* Has modem being powered off? (using AT+CPWROFF) */
  bool is_vcc_off:1;        /* Is modem VCC turned off? */
  bool in_state_machine:1;  /* Running main state machine function? */
  bool wake_timer_active:1; /* Main state wake-up timer has been registered. */
  bool creg_urc_registered:1;
                            /* +CREG URC registered. */
  bool uupsdd_urc_registered:1;
                            /* +UUPSDD URC registered. */
#ifndef CONFIG_UBMODEM_DISABLE_CELLLOCATE
  bool cell_locate_urc_registered:1;
                            /* CellLocate URC registered. */
#endif
#ifdef CONFIG_UBMODEM_USRSOCK
  bool socket_urcs_registered:1;
                            /* Socket URCs registered. */
#endif
#ifdef CONFIG_UBMODEM_VOICE
  bool voice_urcs_registered:1;
                            /* Voice-call URCs registered. */
#endif
  enum ubmodem_cell_locate_aid_support_e support_cell_locate_aid:2;
                            /* +ULOCAID support flag. */

  /* HW control (serial port initialization, gpio & power control) */

  struct
  {
    const struct ubmodem_hw_ops_s *ops;
    void *priv;
  } hw;

  /* Gathered modem info */

  enum ubmodem_model_e model;
  char order_code[20];

  /* Serial port file descriptor for modem I/O */

  int serial_fd;

  /* Network registration timer */

  int creg_timer_id;

  /* CellLocate timeout timer */

#ifndef CONFIG_UBMODEM_DISABLE_CELLLOCATE
  int cell_locate_timer_id;
#endif

  /* Timer for full power off */

  int poweroff_timer;

  /* Data for sub-state machines */

  void (*substate_cleanup_fn)(struct ubmodem_s *modem);
  union
  {
    struct modem_sub_power_on_s
    {
      bool is_reset;
      int step;
    } power_on;

    struct modem_sub_setup_cmd_prompt_s
    {
      int try_count;
      int configure_group_pos;
      int configure_cmd_pos;
      int configure_retries;
    } setup_cmd_prompt;

    struct modem_sub_setup_sim_s
    {
      int try_count;
      bool first_check;
    } setup_sim;

    struct modem_sub_setup_network_s
    {
      const char *fail_reason;
      bool keep_creg_urc:1;
      enum ubmodem_network_state_e network_state:3;
      int8_t received_creg_while_retrying:8;
      struct timespec net_reg_start_ts;
    } setup_network;

    struct modem_sub_gprs_s
    {
      union
      {
        int retry;
        struct
        {
          int pos;
          char value[64];
        } apn_conf;
        struct
        {
          int pos;
          struct ubmodem_event_ip_address_s ipcfg;
        } ipconfig;
      };
    } gprs;
  } sub;
  unsigned int cmd_prompt_ate0_attempts;

  /* Callback functions for events/operations */

  uint32_t active_events;
  sq_queue_t event_listeners;
  uint32_t publishing_event;

#ifdef CONFIG_UBMODEM_USRSOCK
  /* Internal TCP/IP stack to NuttX through /dev/usrsock */

  struct
  {
    int usrsockfd;
    int usockid_counter;
    sq_queue_t list;
    sq_queue_t removed_list;
    struct ubmodem_event_ip_address_s ipcfg;
    uint8_t poll_off_count; /* usrsock poll enabled when count is zero. */
    uint8_t poll_off_list[MODEM_MAX_SOCKETS_OPEN + 1];
  } sockets;
#endif

#ifdef CONFIG_UBMODEM_VOICE
  /* Voice-call handling */

  struct modem_voice_call_s
  {
    bool got_clip:1;
    bool ringing:1;
    bool active:1;
    bool ring_reported:1;
    bool disconnect_reported:1;
    bool pm_activity_enabled:1;
    bool probe_task_queued:1;
    unsigned int probe_fails:3;
    int probe_timerid:20;
    struct ubmodem_voice_call_ringing_s clip;
  } voice;

  /* Audio handling */

  struct modem_audio_s
  {
    bool in_enabled:1;
    bool out_enabled:1;
  } audio;
#endif

#ifdef CONFIG_UBMODEM_FTP_ENABLED
  /* FTP download handling */

  void *ftp_current_operation;
#endif

  /* Timer for timed state changes */

  sq_queue_t timers;
  uint16_t timer_id_cnt;

  /* Tasks queued for modem state machine */

  sq_queue_t tasks;

  /* Configuration callback */

  struct
  {
    ubmodem_config_fn_t func;
    void *priv;
  } config;

  /* Delayed command handling */

  struct
  {
    const struct at_cmd_def_s *cmd;
    modem_response_callback_t callback;
    void *callback_priv;
    char *cmd_buf;
    bool active;
  } delayed_cmd;

  /* Parser data */

  struct at_parser_s parser;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: __ubmodem_parser_assert_debug_print
 *
 * Description:
 *   Debug output for modem asserts
 *
 * Input Parameters:
 *   modem : Modem private structure.
 *
 ****************************************************************************/

void __ubmodem_assert_debug_print(struct ubmodem_s *modem);

/****************************************************************************
 * Name: __ubmodem_do_cell_locate_work
 *
 * Description:
 *   Perform CellLocate work.
 *
 * Input Parameters:
 *   modem : Modem private structure.
 *
 ****************************************************************************/

void __ubmodem_do_cell_locate_work(struct ubmodem_s *modem);

/****************************************************************************
 * Name: __ubmodem_cell_locate_cleanup
 *
 * Description:
 *   Clean-up and free CellLocate state.
 *
 * Input Parameters:
 *   modem : Modem private structure.
 *   got_celllocate_event : True if CellLocate succeeded. Generate
 *                          CellLocate failed event if this is set to 'false'.
 *
 ****************************************************************************/

void __ubmodem_cell_locate_cleanup(struct ubmodem_s *modem,
                                   bool got_celllocate_event);

/****************************************************************************
 * Name: __ubmodem_ftp_download_cleanup
 *
 * Description:
 *   Clean-up and free FTP-download state.
 *
 * Input Parameters:
 *   modem : Modem private structure.
 *
 ****************************************************************************/

#ifdef CONFIG_UBMODEM_FTP_ENABLED

void __ubmodem_ftp_download_cleanup(struct ubmodem_s *modem);

#else

static inline void __ubmodem_ftp_download_cleanup(struct ubmodem_s *modem)
{
  /*_*/
}

#endif /* CONFIG_UBMODEM_FTP_ENABLED */

/****************************************************************************
 * Name: __ubmodem_socket_cleanup_all
 *
 * Description:
 *   Remove and free all sockets.
 *
 * Input Parameters:
 *   modem : Modem private structure.
 *
 ****************************************************************************/

void __ubmodem_socket_cleanup_all(struct ubmodem_s *modem);

/****************************************************************************
 * Name: __ubmodem_wake_waiting_state
 *
 * Description:
 *   Wake up modem main state machine from waiting state.
 *
 * Input Parameters:
 *   modem      : Modem data
 *   force_wake : Forced wake-up even if module not marked to waiting state.
 *
 ****************************************************************************/

void __ubmodem_wake_waiting_state(struct ubmodem_s *modem,
                              bool was_waiting);

/****************************************************************************
 * Name: __ubmodem_substate_start_do_power_on
 *
 * Description:
 *   Start modem power-on sequence/sub-state machine.
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void __ubmodem_substate_start_do_power_on(struct ubmodem_s *modem);

/****************************************************************************
 * Name: __ubmodem_substate_start_do_reset
 *
 * Description:
 *   Start modem reset sequence/sub-state machine.
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void __ubmodem_substate_start_do_reset(struct ubmodem_s *modem);

/****************************************************************************
 * Name: __ubmodem_substate_start_do_power_off
 *
 * Description:
 *   Start modem power-off sequence/sub-state machine.
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void __ubmodem_substate_start_do_power_off(struct ubmodem_s *modem);

/****************************************************************************
 * Name: __ubmodem_substate_start_setup_cmd_prompt
 *
 * Description:
 *   Start modem AT command prompt setup sequence.
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void __ubmodem_substate_start_setup_cmd_prompt(struct ubmodem_s *modem);

/****************************************************************************
 * Name: __ubmodem_substate_start_setup_sim
 *
 * Description:
 *   Start modem SIM setup sequence.
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void __ubmodem_substate_start_setup_sim(struct ubmodem_s *modem);

/****************************************************************************
 * Name: __ubmodem_substate_start_setup_network
 *
 * Description:
 *   Start modem network registration sequence.
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void __ubmodem_substate_start_setup_network(struct ubmodem_s *modem);

/****************************************************************************
 * Name: __ubmodem_substate_start_disconnect_network
 *
 * Description:
 *   Start modem network unregistration sequence.
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void __ubmodem_substate_start_disconnect_network(struct ubmodem_s *modem);

/****************************************************************************
 * Name: __ubmodem_cancel_full_poweroff
 *
 * Description:
 *   Cancel timer for full modem power-off.
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void __ubmodem_cancel_full_poweroff(struct ubmodem_s *modem);

/****************************************************************************
 * Name: __ubmodem_network_cleanup
 *
 * Description:
 *   Clean-up network connection state.
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void __ubmodem_network_cleanup(struct ubmodem_s *modem);

/****************************************************************************
 * Name: __ubmodem_substate_start_open_gprs
 *
 * Description:
 *   Open GPRS connection sequence.
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void __ubmodem_substate_start_open_gprs(struct ubmodem_s *modem);

/****************************************************************************
 * Name: __ubmodem_substate_start_open_gprs
 *
 * Description:
 *   Close GPRS connection sequence.
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void __ubmodem_substate_start_close_gprs(struct ubmodem_s *modem);

/****************************************************************************
 * Name: __ubmodem_grps_cleanup
 *
 * Description:
 *   Clean-up GPRS connection state.
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void __ubmodem_gprs_cleanup(struct ubmodem_s *modem);

/****************************************************************************
 * Name: __ubmodem_publish_event_slow
 *
 * Description:
 *   Publish modem event to callback functions (slow-path).
 *
 * Input Parameters:
 *   modem    : Modem data
 *   type     : Event type
 *   data     : Data buffer for event
 *   datalen  : Size of buffer
 *
 ****************************************************************************/

void __ubmodem_publish_event_slow(struct ubmodem_s *modem,
                           enum ubmodem_event_flags_e type, const void *data,
                           size_t datalen);

/****************************************************************************
 * Name: __ubmodem_publish_event
 *
 * Description:
 *   Publish modem event to callback functions.
 *
 * Input Parameters:
 *   modem    : Modem data
 *   type     : Event type
 *   data     : Data buffer for event
 *   datalen  : Size of buffer
 *
 ****************************************************************************/

static inline void __ubmodem_publish_event(struct ubmodem_s *modem,
                                       enum ubmodem_event_flags_e type,
                                       const void *data, size_t datalen)
{
  /* Skip if no listener is interested in this type of event. */

  if ((type & modem->active_events) == 0)
    return;

  /* Slow-path call. */

  __ubmodem_publish_event_slow(modem, type, data, datalen);
}

/****************************************************************************
 * Name: __ubmodem_reached_level
 *
 * Description:
 *   Inform that modem state has reached new functionality level
 *
 * Input Parameters:
 *   modem    : Modem data
 *   new_level: New functionality level
 *
 ****************************************************************************/

void __ubmodem_reached_level(struct ubmodem_s *modem,
                         enum ubmodem_func_level_e new_level);

/****************************************************************************
 * Name: __ubmodem_level_transition_failed
 *
 * Description:
 *   Inform that modem could not reach targeted functionality level
 *
 * Input Parameters:
 *   modem    : Modem data
 *   new_level: New functionality level
 *
 ****************************************************************************/

void __ubmodem_level_transition_failed(struct ubmodem_s *modem,
                                   const char *reason_fmt, ...);

/****************************************************************************
 * Name: __ubmodem_change_state
 *
 * Description:
 *   Set new state for main state machine.
 *
 * Input Parameters:
 *   modem    : Modem data
 *   new_state: New state
 *
 ****************************************************************************/

void __ubmodem_change_state(struct ubmodem_s *modem,
                        enum modem_state_e new_state);

/****************************************************************************
 * Name: __ubmodem_retry_current_level
 *
 * Description:
 *   Attempt to go to new level through intermediate level,
 *   current => intermediate => current.
 *
 * Input Parameters:
 *   level   : Functional level requested for modem.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void __ubmodem_retry_current_level(struct ubmodem_s *modem,
                                   enum ubmodem_func_level_e intermediate_level);

/****************************************************************************
 * Name: __ubmodem_set_nonblocking
 *
 * Description:
 *   Setup modem file descriptor non-blocking or blocking
 *
 * Input Parameters:
 *   modem  : Modem data
 *   set    : Set non-blocking if true; set blocking if false
 *
 * Returned Values:
 *   OK: If success
 *   ERROR: If failed
 *
 ****************************************************************************/

int __ubmodem_set_nonblocking(struct ubmodem_s *modem, bool set);

/****************************************************************************
 * Name: __ubmodem_config_get_value
 *
 * Description:
 *   Get configuration values for modem.
 *
 * Input Parameters:
 *   variable : Name of configuration item to read
 *   buf      : Buffer for output value
 *   buflen   : Size of buffer
 *
 * Return value:
 *   true: Found variable, value stored to 'buf'.
 *   false: Did not find variable by this name.
 *
 ****************************************************************************/

bool __ubmodem_config_get_value(struct ubmodem_s *modem, const char *variable,
                            char *buf, size_t buflen);

/****************************************************************************
 * Name: __ubmodem_common_failed_command
 *
 * Description:
 *   Common fail-path for failed commands in sub-state machines
 *
 * Input Parameters:
 *   modem    : Modem data
 *   cmd      : Definition for failed command
 *   info     : Response information from parse
 *   reason   : Reason string for failure.
 *
 ****************************************************************************/

void __ubmodem_common_failed_command(struct ubmodem_s *modem,
                                 const struct at_cmd_def_s *cmd,
                                 const struct at_resp_info_s *info,
                                 const char *reason);

/****************************************************************************
 * Name: __ubmodem_set_timer
 *
 * Description:
 *   Setup timer with callback.
 *
 ****************************************************************************/

#ifdef CONFIG_UBMODEM_DEBUG_VERBOSE
#define __ubmodem_set_timer(modem, timeout_msec, timer_cb, cb_priv) \
        ___ubmodem_set_timer(modem, timeout_msec, timer_cb, cb_priv, \
                              __FILE__ ":" TOSTRING(__LINE__))
#else
#define __ubmodem_set_timer(modem, timeout_msec, timer_cb, cb_priv) \
        ___ubmodem_set_timer(modem, timeout_msec, timer_cb, cb_priv, NULL)
#endif

int ___ubmodem_set_timer(struct ubmodem_s *modem,
                         unsigned int timeout_msec, ubmodem_timer_fn_t timer_cb,
                         void *cb_priv, const char *regname);

/****************************************************************************
 * Name: __ubmodem_remove_timer
 *
 * Description:
 *   Remove timer.
 *
 ****************************************************************************/

void __ubmodem_remove_timer(struct ubmodem_s *modem, uint16_t id);

/****************************************************************************
 * Name: __ubmodem_add_task
 *
 * Description:
 *   Queue task for modem. Tasks are run when modem state machine would
 *   otherwise enter waiting state.
 *
 * Input Parameters:
 *   modem : Modem private structure.
 *   func  : Task function.
 *   priv  : Private data for task function.
 *
 * Return value:
 *   OK if queued successfully, ERROR if out of memory.
 *
 ****************************************************************************/

#ifdef CONFIG_UBMODEM_DEBUG_VERBOSE
#define __ubmodem_add_task(modem, func, priv) \
        ___ubmodem_add_task(modem, func, priv, __FILE__ ":" TOSTRING(__LINE__))
#else
#define __ubmodem_add_task(modem, func, priv) \
        ___ubmodem_add_task(modem, func, priv, NULL)
#endif

int ___ubmodem_add_task(struct ubmodem_s *modem, ubmodem_task_fn_t func,
                        void *priv, const char *regname);

/****************************************************************************
 * Name: ubmodem_start_raw_command
 ****************************************************************************/

int ubmodem_send_raw_command(struct ubmodem_s *modem,
                             const struct at_cmd_def_s *cmd,
                             const modem_command_task_callback_t callback,
                             void *callback_priv, const void *buf,
                             size_t buflen);

/****************************************************************************
 * Name: __ubmodem_cmdprompt_configure_next
 *
 * Description:
 *   Continue modem configuration.
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void __ubmodem_cmdprompt_configure_next(struct ubmodem_s *modem);

/****************************************************************************
 * Name: __ubmodem_cmdprompt_generic_config_handler
 *
 * Description:
 *   Generic AT response handler for simple configuration commands
 *
 ****************************************************************************/

void
__ubmodem_cmdprompt_generic_config_handler(struct ubmodem_s *modem,
                                           const struct at_cmd_def_s *cmd,
                                           const struct at_resp_info_s *info,
                                           const uint8_t *resp_stream,
                                           size_t stream_len, void *priv);

/****************************************************************************
 * Name: __ubmodem_celllocate_get_config_cmds
 *
 * Description:
 *   Get configuration command list for CellLocate.
 *
 * Input Parameters:
 *   modem    : Modem data
 *   ncmds    : Number of commands in array
 *
 * Return value:
 *   Pointer to command array
 *
 ****************************************************************************/

const ubmodem_send_config_func_t *
__ubmodem_celllocate_get_config_cmds(struct ubmodem_s *modem, size_t *ncmds);

#ifdef CONFIG_UBMODEM_VOICE

/****************************************************************************
 * Name: __ubmodem_voice_get_config_commands
 *
 * Description:
 *   Get configuration command list for voice-call setup.
 *
 * Input Parameters:
 *   modem    : Modem data
 *   ncmds    : Number of commands in array
 *
 * Return value:
 *   Pointer to command array
 *
 ****************************************************************************/

const ubmodem_send_config_func_t *
__ubmodem_voice_get_config_commands(struct ubmodem_s *modem, size_t *ncmds);

/****************************************************************************
 * Name: __ubmodem_voice_get_additional_config_commands
 *
 * Description:
 *   Get configuration command list for voice-call audio path.
 *
 * Input Parameters:
 *   modem    : Modem data
 *   ncmds    : Number of commands in array
 *
 * Return value:
 *   Pointer to command array
 *
 ****************************************************************************/

const ubmodem_send_config_func_t *
__ubmodem_voice_get_audiopath_config_commands(struct ubmodem_s *modem,
                                               size_t *ncmds);

/****************************************************************************
 * Name: __ubmodem_voice_control_setup
 *
 * Description:
 *   Setup voice-call control
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void __ubmodem_voice_control_setup(struct ubmodem_s *modem);

/****************************************************************************
 * Name: __ubmodem_voice_control_cleanup
 *
 * Description:
 *   Cleanup and disable voice-call control
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void __ubmodem_voice_control_cleanup(struct ubmodem_s *modem);

#endif /* CONFIG_UBMODEM_VOICE */

/****************************************************************************
 * Name: __ubmodem_recover_stuck_hardware
 *
 * Description:
 *   Recover/reset stuck modem hardware
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

int __ubmodem_recover_stuck_hardware(struct ubmodem_s *modem);

/****************************************************************************
 * Name: __ubmodem_reinitialize_gprs
 *
 * Description:
 *   Reinitialize GPRS connection
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

int __ubmodem_reinitialize_gprs(struct ubmodem_s *modem);

/****************************************************************************
 * Name: __ubmodem_check_cmee_status
 *
 * Description:
 *   Check current CMEE setting
 *
 * Input Parameters:
 *   callback_fn    : callback result function
 *   callback_priv  : callback private data
 *
 * Returned Values:
 *   OK: If success
 *   ERROR: If failed
 *
 ****************************************************************************/

int __ubmodem_check_cmee_status(struct ubmodem_s *modem,
                                modem_check_cmee_func_t callback_fn,
                                void *callback_priv);

/****************************************************************************
 * Public Inline Functions
 ****************************************************************************/

static inline bool __ubmodem_is_model_sara_u(struct ubmodem_s *modem)
{
  switch (modem->model)
    {
      case UBMODEM_MODEL_SARA_U260:
      case UBMODEM_MODEL_SARA_U270:
      case UBMODEM_MODEL_SARA_U280:
      case UBMODEM_MODEL_SARA_U_UNKNOWN:
        return true;
      default:
        return false;
    }
}

static inline bool __ubmodem_is_model_sara_g(struct ubmodem_s *modem)
{
  switch (modem->model)
    {
      case UBMODEM_MODEL_SARA_G300:
      case UBMODEM_MODEL_SARA_G310:
      case UBMODEM_MODEL_SARA_G340:
      case UBMODEM_MODEL_SARA_G350:
      case UBMODEM_MODEL_SARA_G_UNKNOWN:
        return true;
      default:
        return false;
    }
}

#endif /* __SYSTEM_UBMODEM_UBMODEM_INTERNAL_H_ */
