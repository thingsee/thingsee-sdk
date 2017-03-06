/****************************************************************************
 * apps/system/ubmodem/ubmodem_ftp_download.c
 *
 *   Copyright (C) 2015-2017 Haltian Ltd. All rights reserved.
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

#include <sys/types.h>
#include <assert.h>
#include <debug.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <apps/system/ubmodem.h>
#include <apps/system/conman.h>
#include <apps/netutils/netlib.h>

#include "ubmodem_internal.h"
#include "ubmodem_hw.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MODEM_SEND_FTP_CONFIG_RETRIES    60
#define MODEM_SEND_FTP_CONFIG_RETRY_SECS 1

#define FTP_MODEM_PROBE_SECS             10

#ifndef CONFIG_UBMODEM_FTP_CLIENT_TIMEOUT
#define MODEM_FTP_CLIENT_TIMEOUT 120
#else
#define MODEM_FTP_CLIENT_TIMEOUT CONFIG_UBMODEM_FTP_CLIENT_TIMEOUT
#endif

#define STR(x) #x
#define AT_TIMEOUT_COMMAND(timeout)	"=5," STR(timeout)

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

enum download_state_e
{
  DOWNLOAD_STATE_INIT = 0,
  DOWNLOAD_STATE_CONNECTING,
  DOWNLOAD_STATE_RETRIEVING,
  DOWNLOAD_STATE_DISCONNECTING,
  DOWNLOAD_STATE_COMPLETED,
};

enum ftp_cmd_flag
{
  FTP_COMMAND_FLAG_NONE         = 0,
  FTP_COMMAND_FLAG_WAIT_FOR_URC = 1 << 0,
  FTP_COMMAND_FLAG_IGNORE_ERROR = 1 << 1,
};

struct ftp_cmd_s
{
  const struct at_cmd_def_s *cmd;
  const char *cmd_args_fmt;
  int8_t cmd_arg_indices[2];
  uint32_t cmd_flags;
};

struct ftp_download_priv_s
{
  char *filepath_src_dir;
  char *filepath_src_file;
  void *ftp_priv;
  enum download_state_e download_state;
  int8_t cmds_pos;
  int8_t cmds_retries;
  int cmds_size;
  const struct ftp_cmd_s *cmds;
  bool wait_for_urc;
  bool ignore_error;
  bool retrieve_success;
  int probe_timerid;

  /*
   * cmds_args[0] = hostname or ip-address;
   * cmds_args[1] = "0" if cmds_args[0] is ip-address, "1" if hostname.
   * cmds_args[2] = username;
   * cmds_args[3] = password;
   * cmds_args[4] = filepath_src_dir;
   * cmds_args[5] = filepath_src_file;
   * cmds_args[6] = filepath_dst;
   */
  const void *cmds_args[7];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct at_cmd_def_s cmd_ATpUFTP =
{
  .name             = "+UFTP",
  .resp_format      = NULL,
  .resp_num         = 0,
  .timeout_dsec     = 10 * 10,
};

static const struct at_cmd_def_s cmd_ATpUFTPC =
{
  .name             = "+UFTPC",
  .resp_format      = NULL,
  .resp_num         = 0,
  .timeout_dsec     = 10 * 10,
};

static const struct at_cmd_def_s urc_ATpUUFTPCR =
{
  .name             = "+UUFTPCR",
  .resp_format      =
      (const uint8_t[]){
        RESP_FMT_INT8,
        RESP_FMT_INT8,
      },
  .resp_num         = 2,
};

static const struct at_cmd_def_s cmd_ATpUDELFILE =
{
  .name             = "+UDELFILE",
  .resp_format      = NULL,
  .resp_num         = 0,
  .timeout_dsec     = 10 * 10,
};

static const struct ftp_cmd_s ftp_connect_cmds[] =
{
  /*FTP server IP address*/
  { &cmd_ATpUFTP, "=%s,\"%s\"", { 1, 0 }, FTP_COMMAND_FLAG_NONE },

  /*FTP username*/
  { &cmd_ATpUFTP, "=2,\"%s\"", { 2 }, FTP_COMMAND_FLAG_NONE },

  /*FTP password*/
  { &cmd_ATpUFTP, "=3,\"%s\"", { 3 }, FTP_COMMAND_FLAG_NONE },

  /*FTP client timeout*/
  { &cmd_ATpUFTP, AT_TIMEOUT_COMMAND(MODEM_FTP_CLIENT_TIMEOUT), {}, FTP_COMMAND_FLAG_NONE },

  /*Enable passive mode*/
  { &cmd_ATpUFTP, "=6,1", {}, FTP_COMMAND_FLAG_NONE },

  /*Connect to server*/
  { &cmd_ATpUFTPC, "=1", {}, FTP_COMMAND_FLAG_WAIT_FOR_URC },
};

static const struct ftp_cmd_s ftp_retrieve_cmds[] =
{
  /*Change dir*/
  { &cmd_ATpUFTPC, "=8,\"%s\"", { 4 }, FTP_COMMAND_FLAG_WAIT_FOR_URC },

  /*Delete local file if already exists*/
  { &cmd_ATpUDELFILE, "=\"%s\"", { 6 }, FTP_COMMAND_FLAG_IGNORE_ERROR },

  /*Retrieve file*/
  { &cmd_ATpUFTPC, "=4,\"%s\",\"%s\"", { 5, 6 }, FTP_COMMAND_FLAG_WAIT_FOR_URC },
};

static const struct ftp_cmd_s ftp_disconnect_cmds[] =
{
  /*Disconnect from server*/
  { &cmd_ATpUFTPC, "=0", {}, FTP_COMMAND_FLAG_WAIT_FOR_URC |
       FTP_COMMAND_FLAG_IGNORE_ERROR },
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int execute_next_command(struct ubmodem_s *modem, void *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void cleanup_download(struct ubmodem_s *modem,
                    struct ftp_download_priv_s *ftp_priv,
                    int err)
{
  struct ubmodem_event_ftp_download_status_s data = {};

  dbg("#### FTP cleaning up download:  err %d ####\n", err);

  if (ftp_priv->probe_timerid > -1)
    {
      __ubmodem_remove_timer(modem, ftp_priv->probe_timerid);
      ftp_priv->probe_timerid = -1;
    }

  __ubparser_unregister_response_handler(&modem->parser, urc_ATpUUFTPCR.name);

  data.file_downloaded = (err == OK);
  data.priv = ftp_priv->ftp_priv;

  __ubmodem_publish_event(modem, UBMODEM_EVENT_FLAG_FTP_DOWNLOAD_STATUS, &data,
                          sizeof(data));

  free(ftp_priv->filepath_src_dir);
  free(ftp_priv->filepath_src_file);
  free(ftp_priv);

  modem->ftp_current_operation = NULL;

  /* FTP-download completed, inform power-management of low activity decrease. */

  ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_LOW, false);
}

static void set_download_state(struct ubmodem_s *modem,
                               struct ftp_download_priv_s *ftp_priv,
                               int state)
{
  int err = OK;

  dbg("#### FTP download state transition '%d' -> '%d' ####\n",
      ftp_priv->download_state, state);

  if (ftp_priv->download_state != state)
    {
      ftp_priv->download_state = state;

      switch (ftp_priv->download_state)
      {
        case DOWNLOAD_STATE_CONNECTING:
          ftp_priv->cmds = ftp_connect_cmds;
          ftp_priv->cmds_size = ARRAY_SIZE(ftp_connect_cmds);
          ftp_priv->cmds_pos = 0;
          ftp_priv->cmds_retries = 0;
          err = __ubmodem_add_task(modem, execute_next_command, ftp_priv);
          break;

        case DOWNLOAD_STATE_RETRIEVING:
          ftp_priv->retrieve_success = false;
          ftp_priv->cmds = ftp_retrieve_cmds;
          ftp_priv->cmds_size = ARRAY_SIZE(ftp_retrieve_cmds);
          ftp_priv->cmds_pos = 0;
          ftp_priv->cmds_retries = 0;
          err = __ubmodem_add_task(modem, execute_next_command, ftp_priv);
          break;

        case DOWNLOAD_STATE_DISCONNECTING:
          ftp_priv->cmds = ftp_disconnect_cmds;
          ftp_priv->cmds_size = ARRAY_SIZE(ftp_disconnect_cmds);
          ftp_priv->cmds_pos = 0;
          ftp_priv->cmds_retries = 0;
          err = __ubmodem_add_task(modem, execute_next_command, ftp_priv);
          break;

        case DOWNLOAD_STATE_COMPLETED:
          /* Download completed */

          cleanup_download(modem, ftp_priv,
                           ftp_priv->retrieve_success ? OK : ERROR);
          return;

        default:
          break;
      }
    }

  if (err != OK)
    {
      cleanup_download(modem, ftp_priv, ERROR);
    }
}

static void urc_ATpUUFTPCR_handler(struct ubmodem_s *modem,
                                    const struct at_cmd_def_s *cmd,
                                    const struct at_resp_info_s *info,
                                    const uint8_t *stream, size_t stream_len,
                                    void *priv)
{
  struct ftp_download_priv_s *ftp_priv = priv;
  int err;
  int8_t command = 0;
  int8_t result = 0;

  dbg("#### FTP status %d, state %d, stream_len %d ####\n", info->status,
      ftp_priv->download_state, stream_len);

  if (ftp_priv->download_state != DOWNLOAD_STATE_CONNECTING &&
      ftp_priv->download_state != DOWNLOAD_STATE_RETRIEVING &&
      ftp_priv->download_state != DOWNLOAD_STATE_DISCONNECTING)
    {
      dbg("#### FTP handler called in unexpected state ####\n");
      return;
    }

  err = (info->status == RESP_STATUS_URC) ? OK : ERROR;
  if (err == OK)
    {
      err = __ubmodem_stream_get_int8(&stream, &stream_len, &command) ? OK : ERROR;
      if (err == OK)
        {
          err = __ubmodem_stream_get_int8(&stream, &stream_len, &result) ? OK : ERROR;
          if (err == OK)
            {
              err = (result == 1) ? OK : ERROR;
            }
        }
    }
  dbg("#### FTP response -> err %d, command %d, result %d ####\n", err, command, result);

  if (err == OK || ftp_priv->ignore_error)
    {
      err = __ubmodem_add_task(modem, execute_next_command, ftp_priv);
      MODEM_DEBUGASSERT(modem, err >= 0);
    }
  else
    {
      if (ftp_priv->download_state == DOWNLOAD_STATE_RETRIEVING)
        {
          /* Retrieving failed, close connection. */

          ftp_priv->retrieve_success = false;

          set_download_state(modem, ftp_priv, DOWNLOAD_STATE_DISCONNECTING);
        }
      else
        {
          cleanup_download(modem, ftp_priv, ERROR);
        }

      /* No state change as this is URC handler. */
    }
}

static int retry_configure_timer_handler(struct ubmodem_s *modem,
                                         const int timer_id,
                                         void * const arg)
{
  struct ftp_download_priv_s *ftp_priv = arg;

  if (execute_next_command(modem, ftp_priv) == ERROR)
    {
      __ubmodem_change_state(modem, MODEM_STATE_WAITING);
    }
  return OK;
}

static void execute_commands_handler(struct ubmodem_s *modem,
                                  const struct at_cmd_def_s *cmd,
                                  const struct at_resp_info_s *info,
                                  const uint8_t *resp_stream,
                                  size_t stream_len, void *priv)
{
  struct ftp_download_priv_s *ftp_priv = priv;
  int err;

  dbg("#### FTP status %d, state %d, cmd_pos %d, wait_for_urc? %d, ignore_error? %d ####\n",
      info->status, ftp_priv->download_state, ftp_priv->cmds_pos,
      ftp_priv->wait_for_urc, ftp_priv->ignore_error);

  if (resp_status_is_error_or_timeout(info->status) && !ftp_priv->ignore_error)
    {
      if (ftp_priv->cmds_pos == ftp_priv->cmds_size &&
          ftp_priv->download_state == DOWNLOAD_STATE_RETRIEVING)
        {
          /* Download has succeeded, but disconnecting from FTP server failed.
           * Report success anyway (we got the file). */

          set_download_state(modem, ftp_priv, DOWNLOAD_STATE_COMPLETED);
        }
      else if (ftp_priv->cmds_pos == ftp_priv->cmds_size &&
               ftp_priv->cmds_retries++ < MODEM_SEND_FTP_CONFIG_RETRIES &&
               modem->level >= UBMODEM_LEVEL_GPRS)
        {
          /* Reading message service address failed. This happens when
           * network connection has just been created and modem has not
           * fully initialized network connection. Wait for short period and
           * retry. */

          /*  Retry configuration. */

          ftp_priv->cmds_pos = 0;

          err = __ubmodem_set_timer(modem,
                                    MODEM_SEND_FTP_CONFIG_RETRY_SECS * 1000,
                                    &retry_configure_timer_handler,
                                    ftp_priv);
          if (err == ERROR)
            {
              /* Error here? Add assert? Or just try bailout? */

              MODEM_DEBUGASSERT(modem, false);

              (void)retry_configure_timer_handler(modem, -1, ftp_priv);
            }

          return;
        }
      else
        {
          /* Failed to setup FTP download? */

          cleanup_download(modem, ftp_priv, ERROR);
        }

      /* We are in task state, release modem state machine before returning. */

      __ubmodem_change_state(modem, MODEM_STATE_WAITING);
      return;
    }

  if (ftp_priv->cmds_pos < ftp_priv->cmds_size)
    {
      if (!ftp_priv->wait_for_urc)
        {
          /* Perform next configuration. */

          if (execute_next_command(modem, ftp_priv) == ERROR)
            {
              __ubmodem_change_state(modem, MODEM_STATE_WAITING);
              return;
            }
        }
      else
        {
          /* Free modem state machine for other work and wait for URC. */

          __ubmodem_change_state(modem, MODEM_STATE_WAITING);
          return;
        }
    }
  else
    {
      __ubmodem_change_state(modem, MODEM_STATE_WAITING);
      return;
    }
}

static int execute_next_command(struct ubmodem_s *modem, void *priv)
{
  struct ftp_download_priv_s *ftp_priv = priv;
  int err;

  if (modem->ftp_current_operation != priv)
    {
      /* Request cancelled and already freed. */

      return ERROR;
    }

  if (modem->level < UBMODEM_LEVEL_GPRS)
    {
      goto err_out;
    }

  if (ftp_priv->cmds_pos < ftp_priv->cmds_size)
    {
      const struct at_cmd_def_s *cmd = ftp_priv->cmds[ftp_priv->cmds_pos].cmd;
      const char *cmd_args_fmt = ftp_priv->cmds[ftp_priv->cmds_pos].cmd_args_fmt;
      const void *cmd_arg1 = ftp_priv->cmds_args[ftp_priv->cmds[ftp_priv->cmds_pos].cmd_arg_indices[0]];
      const void *cmd_arg2 = ftp_priv->cmds_args[ftp_priv->cmds[ftp_priv->cmds_pos].cmd_arg_indices[1]];

      dbg("#### FTP command %d state %d ####\n", ftp_priv->cmds_pos, ftp_priv->download_state);

      ftp_priv->wait_for_urc = ((ftp_priv->cmds[ftp_priv->cmds_pos].cmd_flags &
                                FTP_COMMAND_FLAG_WAIT_FOR_URC) == FTP_COMMAND_FLAG_WAIT_FOR_URC);
      ftp_priv->ignore_error = ((ftp_priv->cmds[ftp_priv->cmds_pos].cmd_flags &
                                FTP_COMMAND_FLAG_IGNORE_ERROR) == FTP_COMMAND_FLAG_IGNORE_ERROR);

      ftp_priv->cmds_pos++;

      err = __ubmodem_send_cmd(modem, cmd, execute_commands_handler, ftp_priv,
                               cmd_args_fmt, cmd_arg1, cmd_arg2);
      MODEM_DEBUGASSERT(modem, err == OK);

      /* In task handling state, expecting command response => No state
       * change. */

      return OK;
    }
  else
    {
      switch (ftp_priv->download_state)
      {
        case DOWNLOAD_STATE_CONNECTING:
          /* Connecting succeeded. */

          set_download_state(modem, ftp_priv, DOWNLOAD_STATE_RETRIEVING);
          break;

        case DOWNLOAD_STATE_RETRIEVING:
          /* Retrieving succeeded. */

          ftp_priv->retrieve_success = true;

          set_download_state(modem, ftp_priv, DOWNLOAD_STATE_DISCONNECTING);
          break;

        case DOWNLOAD_STATE_DISCONNECTING:
          /* Disconnecting succeeded. */

          set_download_state(modem, ftp_priv, DOWNLOAD_STATE_COMPLETED);
          break;

        default:
          break;
      }

      /* Task completed. Release state machine and return OK. */

      __ubmodem_change_state(modem, MODEM_STATE_WAITING);

      return OK;
    }

err_out:
  /* This is task start-up function, returning error causes modem state machine
   * to be released. */

  cleanup_download(modem, ftp_priv, ERROR);

  return ERROR;
}

static void probe_check_cmee_result(struct ubmodem_s *modem, bool cmee_ok,
                                    int cmee_setting, void *priv)
{
  int err;

  if (!cmee_ok)
    {
      /* Modem HW is in unknown state, attempt to recover. */

      err = __ubmodem_recover_stuck_hardware(modem);
      MODEM_DEBUGASSERT(modem, err != ERROR);
    }

  /* Update main state machine. */

  __ubmodem_change_state(modem, MODEM_STATE_WAITING);
}

static int probe_check_cmee(struct ubmodem_s *modem, void *priv)
{
  int err;

  if (modem->ftp_current_operation != priv)
    {
      return ERROR;
    }

  if (modem->level < UBMODEM_LEVEL_CMD_PROMPT)
    {
      return ERROR;
    }

  /* Check CMEE setting to see if modem has reseted or not. */

  err = __ubmodem_check_cmee_status(modem, probe_check_cmee_result, NULL);
  MODEM_DEBUGASSERT(modem, err == OK);

  return OK;
}

static int probe_fn(struct ubmodem_s *modem, const int timer_id,
                    void * const arg)
{
  struct ftp_download_priv_s *ftp_priv = modem->ftp_current_operation;

  ftp_priv->probe_timerid = -1;

  /* Start CMEE probe task. */

  __ubmodem_add_task(modem, probe_check_cmee, ftp_priv);

  /* Reregister timer. */

  ftp_priv->probe_timerid = __ubmodem_set_timer(modem,
                                                FTP_MODEM_PROBE_SECS * 1000,
                                                probe_fn,
                                                modem);
  MODEM_DEBUGASSERT(modem, ftp_priv->probe_timerid >= 0);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
                              struct ubmodem_ftp_download_s *ftp, void *priv)
{
  struct ftp_download_priv_s *ftp_priv;
  char *filepath_src_dir;
  char *filepath_src_file;
  size_t hostname_len;
  size_t username_len;
  size_t password_len;
  size_t filepath_dst_len;
  uint8_t ipaddrtmp[4];
  char *hostname;
  char *username;
  char *password;
  char *filepath_dst;
  bool is_ipaddr;
  int err;

  DEBUGASSERT(modem);
  DEBUGASSERT(ftp);
  DEBUGASSERT(ftp->hostname);
  DEBUGASSERT(ftp->username);
  DEBUGASSERT(ftp->password);
  DEBUGASSERT(ftp->filepath_src);
  DEBUGASSERT(ftp->filepath_dst);

  /* Modem's FTP interface allows one on-going operation. Multiple requests
   * could be queued, but for now, let's return error asap. */

  if (modem->ftp_current_operation)
    {
      err = EBUSY;
      goto err_out;
    }

  /* Network connectivity required for FTP download. */

  if (modem->level < UBMODEM_LEVEL_GPRS)
    {
      err = ENONET;
      goto err_out;
    }

  hostname_len = strlen(ftp->hostname) + 1;
  username_len = strlen(ftp->username) + 1;
  password_len = strlen(ftp->password) + 1;
  filepath_dst_len = strlen(ftp->filepath_dst) + 1;

  /* Prepare FTP download task. */

  ftp_priv = calloc(1, sizeof(*ftp_priv) + hostname_len + username_len +
                       password_len + filepath_dst_len);
  if (ftp_priv == NULL)
    {
      err = ENOMEM;
      goto err_out;
    }

  ftp_priv->ftp_priv = priv;

  hostname = (char *)ftp_priv + sizeof(*ftp_priv);
  username = hostname + hostname_len;
  password = username + username_len;
  filepath_dst = password + password_len;

  memcpy(hostname, ftp->hostname, hostname_len);
  memcpy(username, ftp->username, username_len);
  memcpy(password, ftp->password, password_len);
  memcpy(filepath_dst, ftp->filepath_dst, filepath_dst_len);

  /* Splitting ftp_priv->ftp.filepath_src into (directory, file) parts */

  {
    const char *p, *last_sep = NULL;
    filepath_src_dir = NULL;
    filepath_src_file = NULL;

    for (p = ftp->filepath_src; *p; p++)
      {
        if (*p == '/') last_sep = p;
      }
    if (last_sep != NULL)
      {
        int len_dir = last_sep - ftp->filepath_src;
        int len_file = strlen(ftp->filepath_src) - len_dir - 1;
        if (len_dir > 0)
          {
            filepath_src_dir = malloc(len_dir+1);
          }
        if (filepath_src_dir != NULL)
          {
            strncpy(filepath_src_dir, ftp->filepath_src, len_dir);
            filepath_src_dir[len_dir] = 0;
          }
        if (len_file > 0)
          {
            filepath_src_file = malloc(len_file+1);
          }
        if (filepath_src_file != NULL)
          {
            strncpy(filepath_src_file, last_sep+1, len_file);
            filepath_src_file[len_file] = 0;
          }
      }
    else
      {
        const char *default_dir = ".";
        filepath_src_dir = strdup(default_dir);
        filepath_src_file = strdup(ftp->filepath_src);
      }
  }

  /* Is hostname actually ip-address? */

  is_ipaddr = netlib_ipaddrconv(hostname, ipaddrtmp);

  ftp_priv->cmds_args[0] = hostname;
  ftp_priv->cmds_args[1] = is_ipaddr ? "0" : "1";
  ftp_priv->cmds_args[2] = username;
  ftp_priv->cmds_args[3] = password;
  ftp_priv->cmds_args[4] = ftp_priv->filepath_src_dir = filepath_src_dir;
  ftp_priv->cmds_args[5] = ftp_priv->filepath_src_file = filepath_src_file;
  ftp_priv->cmds_args[6] = filepath_dst;

  dbg("#### FTP filepath '%s'-> ('%s', '%s') ####\n", ftp->filepath_src,
      filepath_src_dir, filepath_src_file);

  modem->ftp_current_operation = ftp_priv;
  __ubparser_register_response_handler(&modem->parser, &urc_ATpUUFTPCR,
                                   urc_ATpUUFTPCR_handler, ftp_priv, true);

  ftp_priv->probe_timerid = __ubmodem_set_timer(modem,
                                                FTP_MODEM_PROBE_SECS * 1000,
                                                probe_fn,
                                                modem);
  MODEM_DEBUGASSERT(modem, ftp_priv->probe_timerid >= 0);

  /* New FTP-download, inform power-management of low activity increase. */

  ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_LOW, true);

  /* Finally, change download state. If this fails, it does PM low activity decrease. */

  set_download_state(modem, ftp_priv, DOWNLOAD_STATE_CONNECTING);

  return OK;

err_out:
  errno = err;
  return ERROR;
}

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

void __ubmodem_ftp_download_cleanup(struct ubmodem_s *modem)
{
  if (!modem->ftp_current_operation)
    {
      return;
    }

  cleanup_download(modem, modem->ftp_current_operation, ERROR);
}
