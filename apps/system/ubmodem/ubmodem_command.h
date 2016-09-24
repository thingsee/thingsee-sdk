/****************************************************************************
 * apps/system/ubmodem/ubmodem_command.h
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

#ifndef __SYSTEM_UBMODEM_UBMODEM_COMMAND_H_
#define __STSTEM_UBMODEM_UBMODEM_COMMAND_H_

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
#include "ubmodem_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Timeouts for different command classes */

#define MODEM_CMD_DEFAULT_TIMEOUT              (5 * 10) /* 5 sec */
#define MODEM_CMD_NETWORK_TIMEOUT              ((int)(1.5 * 60 + 10) * 10) /* ~1.5 min */
#define MODEM_CMD_NETWORK_REGISTRATION_TIMEOUT (MODEM_CMD_NETWORK_TIMEOUT + 5 * 10)
#define MODEM_CMD_SOCKET_TIMEOUT               (10 * 10) /* 10 sec */
#define MODEM_CMD_SOCKET_CONNECT_TIMEOUT       (20 * 10) /* 20 sec */
#define MODEM_CMD_POWEROFF_TIMEOUT             (40 * 10) /* 40 sec */
#define MODEM_CMD_SIM_MGMT_TIMEOUT             (10 * 10) /* 10 sec */
#define MODEM_CMD_LOCATION_TIMEOUT             (10 * 10) /* 10 sec */
#define MODEM_CMD_DELETE_MESSAGE_TIMEOUT       (55 * 10) /* 55 sec */

#define MODEM_CMD_NEW_DELAY_MSEC               (20) /* msec */
#define MODEM_DATA_PROMPT_DELAY_MSEC           (50) /* msec */
#define MODEM_TX_BUF_FULL_RECHECK_MSEC         (333) /* msec */

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

struct ubmodem_s;

/****************************************************************************
 * Public Data exports
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: __ubmodem_send_cmd
 *
 * Description:
 *   Send AT command to modem AT prompt, prepare parser for handling result
 *   code and other responses.
 *
 * Input Parameters:
 *   modem         : Modem module structure
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
                   void *callback_priv, const char *cmd_args_fmt, ...);

/****************************************************************************
 * Name: __ubmodem_send_raw
 *
 * Description:
 *   Send raw data to modem AT prompt, prepare parser for handling result
 *   code and other responses.
 *
 * Input Parameters:
 *   modem         : Modem module structure
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
                   void *callback_priv, const void *buf, size_t buflen);

#endif /* __SYSTEM_UBMODEM_UBMODEM_INTERNAL_H_ */

