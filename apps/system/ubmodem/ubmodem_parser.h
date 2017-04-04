/****************************************************************************
 * apps/system/ubmodem/ubmodem_parser.h
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

#ifndef __SYSTEM_UBMODEM_UBMODEM_PARSER_H_
#define __SYSTEM_UBMODEM_UBMODEM_PARSER_H_

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
#include <strings.h>

#include <apps/system/ubmodem.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_UBMODEM_VOICE
#  define NUM_VOICE_URC_HANDLERS 3
#else
#  define NUM_VOICE_URC_HANDLERS 0
#endif

#ifdef CONFIG_UBMODEM_USRSOCK
#  define NUM_USRSOCK_URC_HANDLERS 3
#else
#  define NUM_USRSOCK_URC_HANDLERS 0
#endif

#ifdef CONFIG_UBMODEM_FTP_ENABLED
#  define NUM_FTP_URC_HANDLERS 1
#else
#  define NUM_FTP_URC_HANDLERS 0
#endif

#define NUM_COMMAND_HANDLERS (4 + NUM_USRSOCK_URC_HANDLERS + \
                              NUM_VOICE_URC_HANDLERS + NUM_FTP_URC_HANDLERS)
#define MAX_RESPONSE_STREAM_LEN (1024 + 32)

/* Allow unaligned memory accesses when supported by HW. */

#undef HAS_UNALIGNED_LE_MEMACCESS
#if defined(__GNUC__) && __GNUC__ > 3
#  if defined(__ARM_FEATURE_UNALIGNED) && __ARM_FEATURE_UNALIGNED > 0 && \
      defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__) && \
      __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#    define HAS_UNALIGNED_LE_MEMACCESS 1
#  elif defined(__i386__) || defined (__x86_64__)
#    define HAS_UNALIGNED_LE_MEMACCESS 1
#  endif
#endif

/* Ideally read buffer size should match serial buffer RX buffer size. */

#ifndef CONFIG_UBMODEM_READ_BUFFER_SIZE
#  define CONFIG_UBMODEM_READ_BUFFER_SIZE 64
#endif

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

/* AT response format types */

enum at_resp_format_e
{
  RESP_FMT_INT8 = 0,            /* 8-bit integer */
  RESP_FMT_INT16,               /* 16-bit integer */
  RESP_FMT_INT32,               /* 32-bit integer */
  RESP_FMT_STRING,              /* String without quotes */
  RESP_FMT_STRING_TO_EOL,       /* String without quotes to end-of-line */
  RESP_FMT_QUOTED_STRING,       /* Quoted string */
  RESP_FMT_DATALEN,             /* Length for data-buffer */
  RESP_FMT_QUOTED_DATABUF,      /* Quoted data buffer string, any character
                                   allowed, buffer length given in advance
                                   with RESP_FMT_DATALEN parameter. */
};

/* AT command type definition structure */

struct at_cmd_def_s
{
  const char *name;             /* Name for AT command */
  const uint8_t *resp_format;   /* Response format array, see
                                   'enum at_resp_format_e' */
  unsigned int resp_num:5;      /* Number of entries in format array */
  unsigned int timeout_dsec:16; /* Timeout for AT command in deciseconds */
  bool flag_multiple_resp:1;    /* Response contains multiple lines */
  bool flag_data_prompt:1;      /* Response is data prompt, "@" */
  bool flag_pdu_prompt:1;       /* Response is text/PDU prompt, ">" */
  bool flag_errorcode:1;        /* Response is CME or CMS error */
  bool flag_plain:1;            /* Response is plain string line without
                                   response name. */
  bool flag_pm_low_activity:1;  /* Command is marked with low PM activity
                                   to allow deep-sleep while waiting for
                                   result. */
};

/* AT response callback status codes */

enum at_resp_status_e
{
  RESP_STATUS_OK = 1,           /* Received "OK" for command */
  RESP_STATUS_URC,              /* Unsolicited result code */
  RESP_STATUS_LINE,             /* One response line in multiple line
                                   response */
  RESP_STATUS_DATAPROMPT,       /* Data prompt activated by command */
  RESP_STATUS_ERROR,            /* Received "ERROR" for command */
  RESP_STATUS_CME_ERROR,        /* Received "+CME ERROR: <code>" for command */
  RESP_STATUS_CMS_ERROR,        /* Received "+CMS ERROR: <code>" for command */
  RESP_STATUS_TIMEOUT,          /* Command timed-out */
};
/* AT response information */

struct at_resp_info_s
{
  enum at_resp_status_e status:4; /* Status/type code for the response */
  int errorcode:16;               /* Error-code for error statuses */
  int num_params:12;              /* Number of parameters read and binarized */
};

#ifdef HAS_UNALIGNED_LE_MEMACCESS
typedef uint32_t uint32_alias_t __attribute__((may_alias, aligned(1)));
typedef uint16_t uint16_alias_t __attribute__((may_alias, aligned(1)));
#endif

/****************************************************************************
 * Name: modem_response_callback_t
 *
 * Description:
 *   Command response callback function type
 *
 * Input Parameters:
 *   cmd         - Command definition structure, name / format style / etc
 *   info        - Response information per callback call.
 *   resp_stream - Parameters in binarized format.
 *   stream_len  - Length of binarized parameter stream.
 *   priv        - Private data for callback.
 *
 ****************************************************************************/

typedef void (*modem_response_callback_t)(struct ubmodem_s *modem,
                                          const struct at_cmd_def_s *cmd,
                                          const struct at_resp_info_s *info,
                                          const uint8_t *resp_stream,
                                          size_t stream_len,
                                          void *priv);

/* Parser master states */

enum at_parser_state_e
{
  PARSER_STATE_RESET,                   /* Initial state, no response handler
                                         */
  PARSER_STATE_READ_RESPONSE_NAME,      /* Response type detection */
  PARSER_STATE_SKIP_TO_EOL,             /* Skip to end-of-line */
  PARSER_STATE_START_READ_RARAM,        /* Prepare reading new parameter */
  PARSER_STATE_DO_READ_PARAM,           /* Parameter reading sub-state-machine;
                                           Uses 'enum at_resp_format_e' and
                                           'parser->param' for sub-state. */
};

/* Response handler/callback for active and unsolicited result codes. */

struct at_response_handler_s
{
  bool active;                          /* Active handler? */
  bool unsolicited;                     /* URC handler? */
  const struct at_cmd_def_s *cmd;       /* Command definition */
  modem_response_callback_t callback;   /* Callback function */
  void *callback_priv;                  /* Callback private data */
} packed_struct;

struct modem_response_stream_s
{
  uint8_t buf[MAX_RESPONSE_STREAM_LEN]; /* Response stream buffer */
  uint16_t pos;                         /* Current position */
} packed_struct;

struct at_parser_s
{
  enum at_parser_state_e state;         /* Main state for parser */
  char statebuf[32];                    /* Temporary state-buffer */
  uint8_t statebufpos;

  /* Parameter sub-state */

  struct
  {
    uint8_t *stream_strlen_pos;
    uint16_t strlen;
    uint8_t fmt;
    uint8_t quotes;
    bool found;
  } param;

  struct modem_response_stream_s stream;

  /* Response information */

  struct {
    struct at_resp_info_s info;
    uint16_t datalen;
    struct timespec time_prev;
  } response;

  /* Command handler registry */

  const struct at_response_handler_s *handler;
  struct at_response_handler_s commands[NUM_COMMAND_HANDLERS];
  int cmd_timeout_id;
  bool cmd_timeout_armed;

  /* Read buffer */

  char readbuf[CONFIG_UBMODEM_READ_BUFFER_SIZE];

#ifdef CONFIG_UBMODEM_PARSER_DEBUG
  /* Debugging helpers */

  char last_received[128 + 3];
  uint8_t last_received_pos;
#endif
};

/****************************************************************************
 * Public Data exports
 ****************************************************************************/

/****************************************************************************
 * Public Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: resp_status_is_error
 *
 * Description:
 *   Check if response status indicates error.
 *
 * Input Parameters:
 *   status - Response status code
 *
 * Returned Values:
 *   True if status is error code.
 *
 ****************************************************************************/

static inline bool resp_status_is_error(enum at_resp_status_e status)
{
  return status == RESP_STATUS_ERROR ||
         status == RESP_STATUS_CME_ERROR ||
         status == RESP_STATUS_CMS_ERROR;
}

/****************************************************************************
 * Name: resp_status_is_error_or_timeout
 *
 * Description:
 *   Check if response status indicates error or timeout
 *
 * Input Parameters:
 *   status - Response status code
 *
 * Returned Values:
 *   True if status is error code or timeout.
 *
 ****************************************************************************/

static inline bool resp_status_is_error_or_timeout(enum at_resp_status_e status)
{
  return resp_status_is_error(status) || status == RESP_STATUS_TIMEOUT;
}

/****************************************************************************
 * Name: __ubmodem_stream_get_int8
 *
 * Description:
 *   Extract 8-bit integer parameter from parameter binary stream, move
 *   stream pointer forward by one byte.
 *
 * Input Parameters:
 *   stream     - Pointer to stream pointer   (in/out)
 *   stream_len - Pointer to stream length    (in/out)
 *   outval     - Pointer to output variable  (out)
 *
 * Returned Values:
 *   True if value was successfully extracted from stream.
 *
 ****************************************************************************/

static inline inline_function bool
__ubmodem_stream_get_int8(const uint8_t **stream, size_t *stream_len,
                          int8_t *outval)
{
  const uint8_t *pos = *stream;
  uint8_t val;

  if (*stream_len < sizeof(*outval))
    return false;

  val = *pos++;

  *stream = pos;
  *stream_len -= sizeof(*outval);
  *outval = val;

  return true;
}

/****************************************************************************
 * Name: __ubmodem_stream_get_int16
 *
 * Description:
 *   Extract 16-bit integer parameter from parameter binary stream, move
 *   stream pointer forward by two bytes.
 *
 * Input Parameters:
 *   stream     - Pointer to stream pointer   (in/out)
 *   stream_len - Pointer to stream length    (in/out)
 *   outval     - Pointer to output variable  (out)
 *
 * Returned Values:
 *   True if value was successfully extracted from stream.
 *
 ****************************************************************************/

static inline inline_function bool
__ubmodem_stream_get_int16(const uint8_t **stream, size_t *stream_len,
                           int16_t *outval)
{
  const uint8_t *pos = *stream;
  uint16_t val;

  if (*stream_len < sizeof(*outval))
    return false;

#ifdef HAS_UNALIGNED_LE_MEMACCESS
  val = *(uint16_alias_t *)pos;
  pos += sizeof(int16_t);
#else
  val = *pos++;
  val |= *pos++ << 8;
#endif

  *stream = pos;
  *stream_len -= sizeof(*outval);
  *outval = val;

  return true;
}

/****************************************************************************
 * Name: __ubmodem_stream_get_int32
 *
 * Description:
 *   Extract 32-bit integer parameter from parameter binary stream, move
 *   stream pointer forward by four bytes.
 *
 * Input Parameters:
 *   stream     - Pointer to stream pointer   (in/out)
 *   stream_len - Pointer to stream length    (in/out)
 *   outval     - Pointer to output variable  (out)
 *
 * Returned Values:
 *   True if value was successfully extracted from stream.
 *
 ****************************************************************************/

static inline inline_function bool
__ubmodem_stream_get_int32(const uint8_t **stream, size_t *stream_len,
                           int32_t *outval)
{
  const uint8_t *pos = *stream;
  uint32_t val;

  if (*stream_len < sizeof(*outval))
    return false;

#ifdef HAS_UNALIGNED_LE_MEMACCESS
  val = *(uint32_alias_t *)pos;
  pos += sizeof(int32_t);
#else
  val = *pos++;
  val |= *pos++ << 8;
  val |= *pos++ << 16;
  val |= *pos++ << 24;
#endif

  *stream = pos;
  *stream_len -= sizeof(*outval);
  *outval = val;

  return true;
}

/****************************************************************************
 * Name: __ubmodem_stream_get_string
 *
 * Description:
 *   Extract 16-bit string length and get pointer to string buffer, move
 *   stream pointer forward by (2 + string_length + 1) bytes.
 *
 * Input Parameters:
 *   stream     - Pointer to stream pointer  (in/out)
 *   stream_len - Pointer to stream length   (in/out)
 *   string     - Pointer to string pointer  (out)
 *   string_len - Pointer to string length   (out)
 *
 * Returned Values:
 *   True if string was successfully extracted from stream.
 *
 ****************************************************************************/

static inline inline_function bool
__ubmodem_stream_get_string(const uint8_t **stream, size_t *stream_len,
                        const char **string, uint16_t *string_len)
{
  uint16_t slen = 0;

  /* Get string length. */

  if (!__ubmodem_stream_get_int16(stream, stream_len, (void *)&slen))
    return false;

  if (string_len)
    *string_len = slen;

  /* Check that stream is large enough (string length and null-char) */

  if (*stream_len < slen + 1)
    return false;

  *string = (const void *)*stream;

  *stream += slen + 1;
  *stream_len -= slen + 1;

  return true;
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: __ubmodem_stream_max_readbuf_len
 *
 * Description:
 *   Get maximum length of data buffer for 'cmd' command definition.
 *
 * Input Parameters:
 *   cmd     - Command definition to check
 *
 * Returned Values:
 *   >= 1: Maximum data buffer of data buffer
 *   0: Error
 *
 ****************************************************************************/

size_t __ubmodem_stream_max_readbuf_len(const struct at_cmd_def_s *cmd);

/****************************************************************************
 * Name: __ubparser_setup_command_timeout
 *
 * Description:
 *   Setup and arm timeout timer for currently active command.
 *
 * Input Parameters:
 *   parser   : Parser structure
 *
 * Returned Values:
 *   ERROR if timer setup fails, OK if timer armed.
 *
 ****************************************************************************/

int __ubparser_setup_command_timeout(struct at_parser_s *parser);

/****************************************************************************
 * Name: __ubparser_unregister_response_handler
 *
 * Description:
 *   Unregister response code handler by name
 *
 * Input Parameters:
 *   parser   : Parser structure
 *   name     : Name of command
 *
 ****************************************************************************/

void __ubparser_unregister_response_handler(struct at_parser_s *parser,
                                            const char *name);

/****************************************************************************
 * Name: __ubparser_register_response_handler
 *
 * Description:
 *   Register response code handler callback
 *
 * Input Parameters:
 *   parser         : Parser structure
 *   cmd            : Command definition
 *   callback       : Callback function for handler
 *   callback_priv  : Private data pointer for callback
 *   unsolicited    : Is handler for unsolicited result code?
 *
 ****************************************************************************/

void __ubparser_register_response_handler(struct at_parser_s *parser,
                                      const struct at_cmd_def_s *cmd,
                                      const modem_response_callback_t callback,
                                      void *callback_priv, bool unsolicited);

/****************************************************************************
 * Name: __ubparse_buffer
 *
 * Description:
 *   Pass buffer byte-by-byte to parser state machine.
 *
 * Input Parameters:
 *   parser   : Parser structure
 *   buf      : Buffer containing data from modem
 *   buflen   : Length of buffer
 *
 ****************************************************************************/

void __ubparse_buffer(struct at_parser_s *parser, const void *buf,
                      size_t buflen);

/****************************************************************************
 * Name: __ubmodem_parser_selftest
 *
 * Description:
 *   Run parser self-tests.
 *
 ****************************************************************************/

void __ubmodem_parser_selftest(void);

#endif /* __SYSTEM_UBMODEM_UBMODEM_PARSER_H_ */
