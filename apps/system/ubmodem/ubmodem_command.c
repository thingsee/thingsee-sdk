/****************************************************************************
 * apps/system/ubmodem/ubmodem_command.c
 *
 *   Copyright (C) 2014-2017 Haltian Ltd. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <assert.h>
#include <debug.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include <apps/system/ubmodem.h>

#include "ubmodem_command.h"
#include "ubmodem_parser.h"
#include "ubmodem_internal.h"
#include "ubmodem_hw.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int modem_send_cmd_vfmt(struct ubmodem_s *modem,
                               const struct at_cmd_def_s *cmd,
                               const modem_response_callback_t callback,
                               void *callback_priv, const char *fmt,
                               va_list vargs)
{
  const char *writebuf;
  size_t writelen;
  size_t pos;
  ssize_t nwritten;
  size_t total;
  int err;

  /* Prepare parser for new command. */

  __ubparser_register_response_handler(&modem->parser, cmd, callback,
                                       callback_priv, false);

  /* Calculate message length. */

  total = 0;
  total += snprintf(NULL, 0, "AT%s", cmd->name);
  total += vsnprintf(NULL, 0, fmt, vargs);
  total += 1;

  if (total > 0)
  {
    char tmp[total]; /* Output command is small enough to fit stack. */

    /* Construct message */

    pos = 0;
    pos += snprintf(&tmp[pos], total - pos, "AT%s", cmd->name);
    pos += vsnprintf(&tmp[pos], total - pos, fmt, vargs);

    /* Give trace output if active. */

    if (modem->active_events & UBMODEM_EVENT_FLAG_TRACE_CMD_TO_MODEM)
      {
        __ubmodem_publish_event(modem, UBMODEM_EVENT_FLAG_TRACE_CMD_TO_MODEM,
                                tmp, pos);
      }

    tmp[pos++] = '\r';

    /* Set serial as non-blocking. */

    err = __ubmodem_set_nonblocking(modem, true);
    if (err != OK)
      return ERROR;

    /* Write buffer to modem. */

    writebuf = tmp;
    writelen = pos;
    do
      {
        nwritten = write(modem->serial_fd, writebuf, writelen);
        if (nwritten == ERROR)
          {
            int error = get_errno();

            if (error != EAGAIN)
              {
                dbg("modem write error, errno: %d\n", error);

                return ERROR;
              }
            nwritten = 0;
          }
        writebuf += nwritten;
        writelen -= nwritten;
      }
    while (writelen > 0);

    /* Give trace output if active. */

    if (modem->active_events & UBMODEM_EVENT_FLAG_TRACE_DATA_TO_MODEM)
      {
        __ubmodem_publish_event(modem, UBMODEM_EVENT_FLAG_TRACE_DATA_TO_MODEM,
                                tmp, pos);
      }
  }

  /* Setup timeout for command response. */

  err = __ubparser_setup_command_timeout(&modem->parser);
  assert(err == OK);

  return OK;
}

static int modem_send_cmd_fmt(struct ubmodem_s *modem,
                              const struct at_cmd_def_s *cmd,
                              const modem_response_callback_t callback,
                              void *callback_priv, const char *fmt, ...)
{
  va_list vargs;
  int err;

  va_start(vargs, fmt);
  err = modem_send_cmd_vfmt(modem, cmd, callback, callback_priv, fmt, vargs);
  va_end(vargs);

  return err;
}

static int modem_delayed_command_callback(struct ubmodem_s *modem, int timer_id,
                                          void * const arg)
{
  const struct at_cmd_def_s *cmd = modem->delayed_cmd.cmd;
  const modem_response_callback_t callback = modem->delayed_cmd.callback;
  void *callback_priv = modem->delayed_cmd.callback_priv;
  char *buf = modem->delayed_cmd.cmd_buf;
  int err;

  MODEM_DEBUGASSERT(modem, modem->delayed_cmd.active);
  MODEM_DEBUGASSERT(modem, modem->delayed_cmd.cmd_buf);

  ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_HIGH, false);

  modem->delayed_cmd.active = false;
  modem->delayed_cmd.cmd_buf = NULL;

  err = modem_send_cmd_fmt(modem, cmd, callback, callback_priv, "%s", buf);

  free(buf);
  return err;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __ubmodem_send_cmd
 *
 * Description:
 *   Send AT command to modem AT prompt, prepare parser for handling result
 *   code and other responses.
 *
 * Input Parameters:
 *   cmd           : AT command definition for parser (response format, etc).
 *   callback      : Callback function for results
 *   callback_priv : Callback private data
 *   cmd_args_fmt  : AT command argument output format (*printf)
 *   ...           : *printf arguments
 *
 * Returned Values:
 *   OK if success.
 *   ERROR if failed.
 *
 ****************************************************************************/

int __ubmodem_send_cmd(struct ubmodem_s *modem,
                   const struct at_cmd_def_s *cmd,
                   const modem_response_callback_t callback,
                   void *callback_priv, const char *cmd_args_fmt, ...)
{
  ssize_t err;
  va_list va_args;
  struct timespec curr_time;
  int64_t timediff = 0;
  long nsec_diff;
  int ret;

  MODEM_DEBUGASSERT(modem, !modem->delayed_cmd.active);
  MODEM_DEBUGASSERT(modem, !modem->delayed_cmd.cmd_buf);

  /* Check difference between curr_time and parser.response.time_prev. */

  clock_gettime(CLOCK_MONOTONIC, &curr_time);

  if (curr_time.tv_sec < modem->parser.response.time_prev.tv_sec)
    {
      /* Should not happen. */

      ubdbg("Trouble with CLOCK_MONOTONIC\n");
    }
  else
    {
      timediff = curr_time.tv_sec;
      timediff -= modem->parser.response.time_prev.tv_sec;
      timediff *= 1000;

      nsec_diff = curr_time.tv_nsec - modem->parser.response.time_prev.tv_nsec;
      timediff += nsec_diff / (1000 * 1000);
    }

  /* Check if we need to delay new command. */

  if (timediff < MODEM_CMD_NEW_DELAY_MSEC)
    {
      /* Setup delayed command timer. */

      modem->delayed_cmd.active = true;

      modem->delayed_cmd.cmd = cmd;
      modem->delayed_cmd.callback = callback;
      modem->delayed_cmd.callback_priv = callback_priv;

      va_start(va_args, cmd_args_fmt);
      err = vasprintf(&modem->delayed_cmd.cmd_buf, cmd_args_fmt, va_args);
      va_end(va_args);
      if (err >= 0)
        {
          ret = __ubmodem_set_timer(modem, MODEM_CMD_NEW_DELAY_MSEC - timediff,
                                  &modem_delayed_command_callback, modem);
          if (ret == ERROR)
            return ERROR;

          ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_HIGH, true);

          return OK;
        }
      else
        {
          /* Could not allocate buffer? Try sending command without delay. */
        }
    }

  /* Send command now. */

  va_start(va_args, cmd_args_fmt);
  err = modem_send_cmd_vfmt(modem, cmd, callback, callback_priv,
                            cmd_args_fmt, va_args);
  va_end(va_args);

  return err;
}

/****************************************************************************
 * Name: __ubmodem_send_raw
 *
 * Description:
 *   Send raw data to modem AT prompt, prepare parser for handling result
 *   code and other responses.
 *
 * Input Parameters:
 *   cmd           : AT command definition for parser (response format, etc).
 *   callback      : Callback function for results
 *   callback_priv : Callback private data
 *   buf           : Buffer to feed to modem
 *   buflen        : Length of buffer
 *
 * Returned Values:
 *   OK if success.
 *   ERROR if failed.
 *
 ****************************************************************************/

int __ubmodem_send_raw(struct ubmodem_s *modem,
                   const struct at_cmd_def_s *cmd,
                   const modem_response_callback_t callback,
                   void *callback_priv, const void *buf, size_t buflen)
{

  ssize_t nwritten;
  const char *writebuf;
  size_t writelen;
  int err;

  /* Prepare parser for new command. */

  __ubparser_register_response_handler(&modem->parser, cmd, callback,
                                       callback_priv, false);

  /* Set serial as non-blocking. */

  err = __ubmodem_set_nonblocking(modem, true);
  if (err != OK)
    return ERROR;

  /* Write buffer to modem. */

  writebuf = buf;
  writelen = buflen;
  do
    {
      nwritten = write(modem->serial_fd, writebuf, writelen);
      if (nwritten == ERROR)
        {
          int error = get_errno();

          if (error != EAGAIN)
            {
              dbg("modem write error, errno: %d\n", error);

              return ERROR;
            }
          nwritten = 0;
        }
      writebuf += nwritten;
      writelen -= nwritten;
    }
  while (writelen > 0);

  /* Give trace output if active. */

  if (modem->active_events & UBMODEM_EVENT_FLAG_TRACE_DATA_TO_MODEM)
    __ubmodem_publish_event(modem, UBMODEM_EVENT_FLAG_TRACE_DATA_TO_MODEM,
                        buf, buflen);

  /* Setup timeout for command response. */

  err = __ubparser_setup_command_timeout(&modem->parser);
  assert(err == OK);

  return OK;
}
