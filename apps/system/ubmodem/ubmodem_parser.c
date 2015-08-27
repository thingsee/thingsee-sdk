/****************************************************************************
 * apps/system/ubmodem/ubmodem_parser.c
 *
 *   Copyright (C) 2014-2015 Haltian Ltd. All rights reserved.
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
#include <fcntl.h>
#include <unistd.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <poll.h>
#include <errno.h>

#include <apps/system/ubmodem.h>

#include "ubmodem_parser.h"
#include "ubmodem_command.h"
#include "ubmodem_internal.h"

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

static struct at_response_handler_s *
parser_find_response_handler_by_name(struct at_parser_s *parser,
                                     const char *name);
static struct at_response_handler_s *
parser_find_response_handler_for_active_command(struct at_parser_s *parser);
static bool parse_parameter_byte(struct at_parser_s *parser, char cur_char,
                                 bool end_of_line);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static struct ubmodem_s *parser_to_ubmodem(struct at_parser_s *parser)
{
  return container_of(parser, struct ubmodem_s, parser);
}

/****************************************************************************
 * Name: is_visible_at_ascii
 *
 * Description:
 *   Check if 'c' is in visible ASCII range.
 *
 ****************************************************************************/

static bool is_visible_at_ascii(char c)
{
  /* Hayes command set visible ASCII character range from 32 to 126. */

  return (c >= 32) && (c <= 126);
}

/****************************************************************************
 * Name: null_terminate_buf
 *
 * Description:
 *   Add null-termination character at end of buffer.
 *
 * Input Parameters:
 *   buf    : Buffer to modify
 *   maxlen : Maximum length of buffer
 *   curlen : Current used space in buffer
 *
 ****************************************************************************/

static void null_terminate_buf(void *buf, size_t maxlen, size_t curlen)
{
  size_t nullpos;

  if (curlen < maxlen)
    nullpos = curlen;
  else
    nullpos = maxlen - 1;

  ((char*)buf)[nullpos] = '\0';
}

/****************************************************************************
 * Name: modem_stream_reset
 *
 * Description:
 *   Reset binarized parameter stream
 *
 * Input Parameters:
 *   stream  : Stream structure to reset
 *
 ****************************************************************************/

static void modem_stream_reset(struct modem_response_stream_s *stream)
{
  stream->pos = 0;
}

/****************************************************************************
 * Name: modem_stream_put_int8
 *
 * Description:
 *   Add 8-bit integer to parameter stream.
 *
 * Input Parameters:
 *   stream  : Stream structure
 *   val     : 8-bit integer value
 *
 * Returned Values:
 *   True if enough space left and value added to stream, false if no space.
 *
 ****************************************************************************/

static bool modem_stream_put_int8(struct modem_response_stream_s *stream,
                                  int8_t val)
{
  if (stream->pos + sizeof(val) > sizeof(stream->buf))
    return false;

  stream->buf[stream->pos++] = val;
  return true;
}

/****************************************************************************
 * Name: modem_stream_put_int16
 *
 * Description:
 *   Add 12-bit integer to parameter stream.
 *
 * Input Parameters:
 *   stream  : Stream structure
 *   val     : 16-bit integer value
 *
 * Returned Values:
 *   True if enough space left and value added to stream, false if no space.
 *
 ****************************************************************************/

static bool modem_stream_put_int16(struct modem_response_stream_s *stream,
                                   int16_t val)
{
  if (stream->pos + sizeof(val) > sizeof(stream->buf))
    return false;

#ifdef HAS_UNALIGNED_LE_MEMACCESS
  *(int16_t*)&stream->buf[stream->pos] = val;
  stream->pos += sizeof(int16_t);
#else
  stream->buf[stream->pos++] = val & 0xff; val >>= 8;
  stream->buf[stream->pos++] = val & 0xff;
#endif

  return true;
}

/****************************************************************************
 * Name: modem_stream_put_int32
 *
 * Description:
 *   Add 32-bit integer to parameter stream.
 *
 * Input Parameters:
 *   stream  : Stream structure
 *   val     : 32-bit integer value
 *
 * Returned Values:
 *   True if enough space left and value added to stream, false if no space.
 *
 ****************************************************************************/

static bool modem_stream_put_int32(struct modem_response_stream_s *stream,
                                   int32_t val)
{
  if (stream->pos + sizeof(val) > sizeof(stream->buf))
    return false;

#ifdef HAS_UNALIGNED_LE_MEMACCESS
  *(int32_t*)&stream->buf[stream->pos] = val;
  stream->pos += sizeof(int32_t);
#else
  stream->buf[stream->pos++] = val & 0xff; val >>= 8;
  stream->buf[stream->pos++] = val & 0xff; val >>= 8;
  stream->buf[stream->pos++] = val & 0xff; val >>= 8;
  stream->buf[stream->pos++] = val & 0xff;
#endif
  return true;
}

/****************************************************************************
 * Name: modem_stream_get_strlenpos
 *
 * Description:
 *   Allocate space for string length (16-bit integer) and get pointer to
 *   this element.
 *
 * Input Parameters:
 *   stream  : Stream structure
 *   bufpos  : Pointer to buffer pointer, used for output
 *
 * Returned Values:
 *   True if enough space left and space allocated from stream, false if
 *   no space.
 *
 ****************************************************************************/

static bool modem_stream_get_strlenpos(struct modem_response_stream_s *stream,
                                       uint8_t **bufpos)
{
  if (stream->pos + sizeof(uint16_t) > sizeof(stream->buf))
    return false;

  *bufpos = &stream->buf[stream->pos];

#ifdef HAS_UNALIGNED_LE_MEMACCESS
  *(int16_t*)&stream->buf[stream->pos] = -1;
  stream->pos += sizeof(int16_t);
#else
  stream->buf[stream->pos++] = -1;
  stream->buf[stream->pos++] = -1;
#endif
  return true;
}

/****************************************************************************
 * Name: modem_do_callback
 *
 * Description:
 *   Perform modem parser callback and disable timeout timer for current
 *   command
 *
 * Input Parameters:
 *   parser    : Parser structure
 *   handler   : Parser handler/callback to call
 *   stream    : Binarized parameter stream
 *   streamlen : Length of stream
 *
 ****************************************************************************/

static void modem_do_callback(struct at_parser_s *parser,
                              const struct at_response_handler_s *handler,
                              const void *stream, size_t streamlen)
{
  struct ubmodem_s *modem = parser_to_ubmodem(parser);

  if (!handler->unsolicited && parser->cmd_timeout_armed)
    {
      /* Unregister command timeout. */

      __ubmodem_remove_timer(modem, parser->cmd_timeout_id);

      parser->cmd_timeout_id = -1;
      parser->cmd_timeout_armed = false;
    }

  /* Call handler callback. */

  handler->callback(modem, handler->cmd, &parser->response.info, stream,
                    streamlen, handler->callback_priv);
}

/****************************************************************************
 * Name: parser_resp_trace
 ****************************************************************************/

static void parser_resp_trace(struct at_parser_s *parser,
                              const char *resp_str)
{
  struct ubmodem_s *modem = parser_to_ubmodem(parser);

  if (modem->active_events & UBMODEM_EVENT_FLAG_TRACE_RESP_FROM_MODEM)
    {
      __ubmodem_publish_event(modem, UBMODEM_EVENT_FLAG_TRACE_RESP_FROM_MODEM,
                              resp_str, strlen(resp_str));
    }
}

/****************************************************************************
 * Name: parser_found_response_name
 *
 * Description:
 *   Check if found response name can be recognized and select correct
 *   action and handler for the detected response.
 *
 * Input Parameters:
 *   parser    : Parser structure
 *   resp_name : Parsed response name
 *
 * Returned Values:
 *   True if response line is expected to contain parameters.
 *
 ****************************************************************************/

static bool parser_found_response_name(struct at_parser_s *parser,
                                       const char *resp_name)
{
  static const uint8_t error_format[] = { RESP_FMT_INT32 };
  static const struct at_cmd_def_s error_def = {
    .name = "",
    .resp_format = error_format,
    .resp_num = 1,
    .flag_errorcode = true,
  };
  static const struct at_response_handler_s internal_error_handler = {
    .cmd = &error_def,
  };
  struct at_response_handler_s *active, *urc, *plain;

  plain = NULL;

  /* Clear handler */

  parser->handler = NULL;

  /* Check if we have active command running */

  active = parser_find_response_handler_for_active_command(parser);
  if (active)
    {
      /* Check valid responses for active command */

      if (strcmp(resp_name, active->cmd->name) == 0)
        {
          parser_resp_trace(parser, active->cmd->name);

          /* This is response parameter line for active command. */

          /* Reset parameter state and stream. */

          parser->response.info.num_params = 0;
          modem_stream_reset(&parser->stream);

          /* Use currently active cmd for response format. */

          parser->handler = active;
        }
      else if (strcmp(resp_name, "OK") == 0)
        {
          parser_resp_trace(parser, "OK");

          /* Fill "complete OK" response type. */

          parser->response.info.status = RESP_STATUS_OK;
          parser->response.info.errorcode = 0;

          if (active->cmd->flag_multiple_resp)
            modem_stream_reset(&parser->stream);

          /* Unregister handler before callback, so that callback can
           * register new.
           */

          __ubparser_unregister_response_handler(parser, active->cmd->name);

          /* Store last completed time. */

          clock_gettime(CLOCK_MONOTONIC, &parser->response.time_prev);

          /* Do callback. */

          modem_do_callback(parser, active, parser->stream.buf,
                            parser->stream.pos);

          /* Reset stream. */

          modem_stream_reset(&parser->stream);

          return false;
        }
      else if ((active->cmd->flag_data_prompt && strcmp(resp_name, "@") == 0) ||
               (active->cmd->flag_pdu_prompt && strcmp(resp_name, ">") == 0))
        {
          parser_resp_trace(parser, resp_name);

          /* Fill "data prompt" response type. */

          parser->response.info.status = RESP_STATUS_DATAPROMPT;
          parser->response.info.errorcode = 0;

          /* Unregister handler before callback, so that callback can
           * register new.
           */

          __ubparser_unregister_response_handler(parser, active->cmd->name);

          /* NOTE: Don't update &parser->response.time_prev here. */

          /* Do callback. */

          modem_do_callback(parser, active, NULL, 0);

          /* Reset stream. */

          modem_stream_reset(&parser->stream);

          return false;
        }
      else if (strcmp(resp_name, "ERROR") == 0)
        {
          parser_resp_trace(parser, "ERROR");

          /* Fill "complete ERROR" response type. */

          parser->response.info.status = RESP_STATUS_ERROR;
          parser->response.info.errorcode = -1;

          /* Unregister handler before callback, so that callback can
           * register new.
           */

          __ubparser_unregister_response_handler(parser, active->cmd->name);

          /* Store last completed time. */

          clock_gettime(CLOCK_MONOTONIC, &parser->response.time_prev);

          /* Do callback. */

          modem_do_callback(parser, active, NULL, 0);

          /* Reset stream. */

          modem_stream_reset(&parser->stream);

          return false;
        }
      else if (strcmp(resp_name, "+CME ERROR") == 0)
        {
          parser_resp_trace(parser, "+CME ERROR");

          /* Fill "complete CME ERROR" response type. */

          parser->response.info.status = RESP_STATUS_CME_ERROR;
          parser->response.info.errorcode = -1;

          /* Use internal "+CME ERROR" handler for parsing error code. */

          parser->handler = &internal_error_handler;
        }
      else if (strcmp(resp_name, "+CMS ERROR") == 0)
        {
          parser_resp_trace(parser, "+CMS ERROR");

          /* Fill "complete CMS ERROR" response type. */

          parser->response.info.status = RESP_STATUS_CMS_ERROR;
          parser->response.info.errorcode = -1;

          /* Use internal "+CMS ERROR" handler for parsing error code. */

          parser->handler = &internal_error_handler;
        }
      else if (active->cmd->flag_plain)
        {
          plain = active;
        }
    }

  if (!parser->handler)
    {
      /* Check if response name is known URC. */

      urc = parser_find_response_handler_by_name(parser, resp_name);

      if (urc)
        {
          parser_resp_trace(parser, urc->cmd->name);

          /* Handle the unsolicited response code */

          parser->handler = urc;
        }
    }

  if (!parser->handler && plain)
    {
      size_t i, slen;

      /* Plain response line handler. */

      /* This is response parameter line for active command. */

      /* Reset parameter state and stream. */

      parser->response.info.num_params = 0;
      modem_stream_reset(&parser->stream);

      /* Use currently active cmd for response format. */

      parser->handler = plain;

      /* Start by feeding current buffer to parameter parser. */

      parser->param.fmt =
          parser->handler->cmd->resp_format[parser->response.info.num_params];

      for (i = 0, slen = strlen(resp_name); i < slen; i++)
        {
          parse_parameter_byte(parser, resp_name[i], false);
        }

      return true;
    }

  if (!parser->handler)
    {
      /* Unknown response type. */

      return false;
    }

  /* Reset parameter state and stream. */

  parser->response.info.num_params = 0;
  modem_stream_reset(&parser->stream);

  return true;
}

/****************************************************************************
 * Name: modem_parser_handle_errorcode
 *
 * Description:
 *   Handle received error-code and call handler callback
 *
 * Input Parameters:
 *   parser    : Parser structure
 *
 ****************************************************************************/

static void modem_parser_handle_errorcode(struct at_parser_s *parser)
{
  const uint8_t *stream = parser->stream.buf;
  size_t stream_len = parser->stream.pos;
  struct at_response_handler_s *active;
  int32_t errorcode;
  char errstr[32];

  /* Check if we have active command running */

  active = parser_find_response_handler_for_active_command(parser);
  DEBUGASSERT(active);

  /* Get error code from stream. */

  if (!__ubmodem_stream_get_int32(&stream, &stream_len, &errorcode))
    errorcode = -1;

  parser->response.info.errorcode = errorcode;
  parser->response.info.num_params = 0;

  snprintf(errstr, sizeof(errstr), "errorcode = %d", errorcode);
  parser_resp_trace(parser, errstr);

  /* Unregister handler before callback, so that callback can
   * register new.
   */

  __ubparser_unregister_response_handler(parser, active->cmd->name);

  /* Store last completed time. */

  clock_gettime(CLOCK_MONOTONIC, &parser->response.time_prev);

  /* Pass result to callback. */

  modem_do_callback(parser, active, NULL, 0);

  /* Reset parser. */

  parser->handler = NULL;
  parser->state = PARSER_STATE_RESET;

  /* Reset stream. */

  modem_stream_reset(&parser->stream);
}

/****************************************************************************
 * Name: modem_parser_handle_response_line
 *
 * Description:
 *   Handle received response line of multi-line response type.
 *
 * Input Parameters:
 *   parser    : Parser structure
 *
 ****************************************************************************/

static void modem_parser_handle_response_line(struct at_parser_s *parser)
{
  const struct at_response_handler_s *active = parser->handler;

  /* Store last completed time. */

  clock_gettime(CLOCK_MONOTONIC, &parser->response.time_prev);

  /* Pass result to callback. */

  parser->response.info.status = RESP_STATUS_LINE;

  modem_do_callback(parser, active, parser->stream.buf, parser->stream.pos);

  /* Reset parser. */

  parser->response.info.num_params = 0;
  parser->response.info.errorcode = 0;

  parser->handler = NULL;
  parser->state = PARSER_STATE_RESET;

  /* Reset stream. */

  modem_stream_reset(&parser->stream);
}

/****************************************************************************
 * Name: modem_parser_handle_unsolicited
 *
 * Description:
 *   Handler unsolicited response code (URC), call corresponding callback.
 *
 * Input Parameters:
 *   parser    : Parser structure
 *
 ****************************************************************************/

static void modem_parser_handle_unsolicited(struct at_parser_s *parser)
{
  const struct at_response_handler_s *urc = parser->handler;

  /* Store last completed time. */

  clock_gettime(CLOCK_MONOTONIC, &parser->response.time_prev);

  /* Pass result to callback. */

  parser->response.info.status = RESP_STATUS_URC;
  parser->response.info.errorcode = 0;

  modem_do_callback(parser, urc, parser->stream.buf, parser->stream.pos);

  /* Reset parser. */

  parser->response.info.num_params = 0;

  parser->handler = NULL;
  parser->state = PARSER_STATE_RESET;

  /* Reset stream. */

  modem_stream_reset(&parser->stream);
}

/****************************************************************************
 * Name: parse_parameter_byte
 *
 * Description:
 *   Parameter parsing sub-state machine. Transform ASCII parameters to
 *   binary stream.
 *
 * Input Parameters:
 *   parser      : Parser structure
 *   cur_char    : Current input character
 *   end_of_line : Is current input character a end-of-line character?
 *
 * Returned Values:
 *   True if main state machine needs to reprocess the 'cur_char'.
 *
 ****************************************************************************/

static bool parse_parameter_byte(struct at_parser_s *parser, char cur_char,
                                 bool end_of_line)
{
  long val;
  uint8_t fmt = parser->param.fmt;
  bool ok;
  bool do_restart = false;
  bool end_of_param;

  switch ((enum at_resp_format_e)fmt)
    {
    case RESP_FMT_INT8:
    case RESP_FMT_INT16:
    case RESP_FMT_INT32:
    case RESP_FMT_DATALEN:
      /*
       * Parameter is format '<signed integer>', and ended by ',' or EOL.
       */

      if (cur_char == ',' || end_of_line)
        {
          /*
           * End of parameter.
           *
           * Fill parameter to output stream and proceed with next parameter.
           */

          parser->response.info.num_params++;

          /* Convert parameter string to integer. */

          null_terminate_buf(parser->statebuf, sizeof(parser->statebuf),
                             parser->statebufpos);
          val = strtol(parser->statebuf, NULL, 10);

          /* Store value to response stream. */

          switch (fmt)
            {
            default:
              ok = false;
              break;
            case RESP_FMT_INT8:
              ok = modem_stream_put_int8(&parser->stream, val);
              break;
            case RESP_FMT_INT16:
              ok = modem_stream_put_int16(&parser->stream, val);
              break;
            case RESP_FMT_INT32:
              ok = modem_stream_put_int32(&parser->stream, val);
              break;
            case RESP_FMT_DATALEN:
              /*
               * Data length parameter has zero length. Length is stored with
               * RESP_FMT_QUOTED_DATABUF.
               */

              parser->response.datalen = val;
              ok = true;
              break;
            }

          DEBUGASSERT(ok);

          parser->state = PARSER_STATE_START_READ_RARAM;

          /* Restart needed to handle end-of-line. */

          if (end_of_line)
            do_restart = true;

          break;
        }

      /* Add integer characters to temporary buffer. */

      if (cur_char == '-' || cur_char == '+' ||
          (cur_char >= '0' && cur_char <= '9'))
        {
          if (parser->statebufpos < sizeof(parser->statebuf))
            parser->statebuf[parser->statebufpos++] = cur_char;
        }
      break;

    case RESP_FMT_STRING:
      /*
       * Parameter format is plain <string>, ended by ',' or end-of-line.
       */
    case RESP_FMT_STRING_TO_EOL:
      /*
       * Parameter format is plain <string>, ended by end-of-line.
       */

      if (parser->param.stream_strlen_pos == NULL)
        {
          /*
           * Output for string contains length at head as 'uint16_t'.
           * Take the offset for string length and fill in dummy value as
           * place holder.
           */

          ok = modem_stream_get_strlenpos(&parser->stream,
                                          &parser->param.stream_strlen_pos);

          parser->param.strlen = 0;

          DEBUGASSERT(ok);
          DEBUGASSERT(parser->param.stream_strlen_pos);
        }

      if (end_of_line || (fmt == RESP_FMT_STRING && cur_char == ','))
        {
          /*
           * End of parameter.
           */

          parser->response.info.num_params++;

          /* Add null-char for convenience. */

          ok = modem_stream_put_int8(&parser->stream, '\0');
          DEBUGASSERT(ok);

          /* Update string length to stream. */

          parser->param.stream_strlen_pos[0] = parser->param.strlen & 0xff;
          parser->param.stream_strlen_pos[1] =
              (parser->param.strlen >> 8) & 0xff;

          parser->param.stream_strlen_pos = NULL;
          parser->state = PARSER_STATE_START_READ_RARAM;

          /* Restart needed to handle end-of-line. */

          if (end_of_line)
            do_restart = true;

          break;
        }

      /* Skip spaces from start */

      if (cur_char == ' ' && parser->param.strlen == 0)
        break;

      /* Fill characters to output stream. */

      ok = modem_stream_put_int8(&parser->stream, cur_char);
      if (ok)
        parser->param.strlen++;
      DEBUGASSERT(ok);

      break;

    case RESP_FMT_QUOTED_STRING:
      /*
       * Parameter format is quoted string: "<string>"
       *
       * Might have spaces before initial quote.
       * Ended by '",' or end-of-line.
       */
    case RESP_FMT_QUOTED_DATABUF:
      /*
       * Parameter format is quoted string: "<string>"
       *
       * Length of string is stored from previous RESP_FMT_DATALEN parameter
       * to 'parser->response.datalen'. String contains bytes in range
       * 0x00-0xff. Ended after <datalen> bytes, with trailing '",'.
       */

      if (parser->param.stream_strlen_pos == NULL)
        {
          /*
           * Output for string contains length at head as 'uint16_t'.
           * Take the offset for string length and fill in dummy value as place
           * holder.
           */

          ok = modem_stream_get_strlenpos(&parser->stream,
                                          &parser->param.stream_strlen_pos);

          parser->param.strlen = 0;
          parser->param.quotes = 0;

          DEBUGASSERT(ok);
          DEBUGASSERT(parser->param.stream_strlen_pos);
        }

      if (parser->param.quotes == 2)
        {
          /* Skip trailing spaces. */

          if (cur_char == ' ')
            return false;
        }

      if (fmt == RESP_FMT_QUOTED_STRING)
        {
          /* End-of-line or ',' after or before quoted string */

          end_of_param = end_of_line ||
              (parser->param.quotes != 1 && cur_char == ',');
        }
      else
        {
          /* [End-of-line or ','] after or before quoted data buffer*/

          end_of_param = parser->param.quotes != 1 &&
              (end_of_line || cur_char == ',');
        }

      if (end_of_param)
        {
          /*
           * End of parameter.
           */

          parser->response.info.num_params++;

          /* Add null-char for convenience. */

          ok = modem_stream_put_int8(&parser->stream, '\0');
          DEBUGASSERT(ok);

          /* Update string length to stream. */

          parser->param.stream_strlen_pos[0] = parser->param.strlen & 0xff;
          parser->param.stream_strlen_pos[1] =
              (parser->param.strlen >> 8) & 0xff;

          parser->param.stream_strlen_pos = NULL;
          parser->state = PARSER_STATE_START_READ_RARAM;

          /* Restart needed to handle end-of-line. */

          if (end_of_line)
            do_restart = true;

          break;
        }

      /* Handle parameter beginning */

      if (parser->param.quotes == 0)
        {
          DEBUGASSERT(parser->param.strlen == 0);

          /* Skip spaces from start */

          if (cur_char == ' ')
            break;

          /* Expecting quote for beginning */

          if (cur_char != '\"')
            break;

          parser->param.quotes = 1;
          break;
        }

      DEBUGASSERT(parser->param.quotes == 1);

      if (fmt == RESP_FMT_QUOTED_STRING)
        {
          /* Check for trailing quote. */

          if (cur_char == '\"')
            {
              parser->param.quotes++;
              break;
            }
        }
      else
        {
          /* Check read all of data buffer already */

          if (parser->response.datalen == 0)
            {
              if (cur_char == '\"')
                parser->param.quotes++;
              break;
            }

          parser->response.datalen--;
        }

      /* Fill characters to output stream. */

      ok = modem_stream_put_int8(&parser->stream, cur_char);
      if (ok)
        parser->param.strlen++;

      break;
    }

  if (end_of_line)
    {
      /* Special case handling, error-codes and responses with multiple
       * lines. */

      if (parser->handler->cmd->flag_errorcode)
        {
          modem_parser_handle_errorcode(parser);

          do_restart = false;
        }
      else if (parser->handler->unsolicited)
        {
          modem_parser_handle_unsolicited(parser);

          do_restart = false;
        }
      else if (parser->handler->cmd->flag_multiple_resp)
        {
          modem_parser_handle_response_line(parser);

          do_restart = false;
        }
    }

  return do_restart;
}

/****************************************************************************
 * Name: parse_byte
 *
 * Description:
 *   Modem AT response parsing state machine. One byte ('cur_char') at time is
 *   passed to state machine. Main motivation for state machine based approach
 *   is that some commands pass binary data parameters (socket read), which
 *   would cause issues for per-line based parsing.
 *
 * Input Parameters:
 *   parser      : Parser structure
 *   cur_char    : Current input character
 *
 ****************************************************************************/

static void parse_byte(struct at_parser_s *parser, char cur_char)
{
  bool end_of_line;
  bool statebuffull;

  end_of_line = (cur_char == '\r' || cur_char == '\n');

#ifdef CONFIG_UBMODEM_PARSER_DEBUG
  parser->last_received[parser->last_received_pos++] = cur_char;
  if (parser->last_received_pos == sizeof(parser->last_received))
    parser->last_received_pos = 0;
#endif

restart:
  switch (parser->state)
    {
    case PARSER_STATE_RESET:
      /* Quick exit if empty line. */

      if (end_of_line)
        break;

      /* Initial state for parser, should be beginning of line. */

      parser->statebufpos = 0;
      parser->handler = NULL;
      parser->state = PARSER_STATE_READ_RESPONSE_NAME;

      /* Pass-through (No break) */

    case PARSER_STATE_READ_RESPONSE_NAME:
      /*
       * Reading response name.
       * - Response line is ended with "\r\n".
       * - Response name can be ended with "\r\n" or ":".
       * - If name is ended with ":", line has additional response arguments.
       */

      DEBUGASSERT(parser->statebufpos <= sizeof(parser->statebuf));

      statebuffull = (parser->statebufpos == sizeof(parser->statebuf) - 1);

      /* Is end-of-line, end-of-name or data prompt */

      if (end_of_line || cur_char == ':' || statebuffull ||
          ((cur_char == '@' || cur_char == '>') && parser->statebufpos == 0))
        {
          if (cur_char == '@' || cur_char == '>')
            {
              parser->statebuf[0] = cur_char;
              parser->statebufpos = 1;
            }
          else if (parser->statebufpos == 0)
            {
              /* Empty line.. reset line parser. */

              parser->state = PARSER_STATE_RESET;
              break;
            }

          /* Handle found response name. */

          null_terminate_buf(parser->statebuf, sizeof(parser->statebuf),
                             parser->statebufpos);

          if (parser_found_response_name(parser, parser->statebuf))
            {
              /*
               * This command was expected to be received, continue
               * parsing arguments.
               */

              DEBUGASSERT(parser->handler);

              if (parser->handler->cmd->flag_plain)
                {
                  /* Handle current character as part of parameter. */

                  parser->state = PARSER_STATE_DO_READ_PARAM;
                  goto restart;
                }
              else
                {
                  parser->state = PARSER_STATE_START_READ_RARAM;
                }
            }
          else
            {
              /*
               * Unknown response or completed command, skip to end-of-line.
               */

              if (end_of_line || (cur_char == '@' || cur_char == '>'))
                parser->state = PARSER_STATE_RESET;
              else
                parser->state = PARSER_STATE_SKIP_TO_EOL;

            }

          break;
        }

      /* Only printable, non-whitespace, characters can be part of response
       * name. */

      if (is_visible_at_ascii(cur_char))
        {
          /* Add character to response name buffer. */

          if (parser->statebufpos < sizeof(parser->statebuf))
            {
              parser->statebuf[parser->statebufpos++] = cur_char;
            }
        }

      break;

    case PARSER_STATE_START_READ_RARAM:
      /*
       * Start reading next parameter based on command response format.
       */

      DEBUGASSERT(parser->handler);

      parser->statebufpos = 0;
      memset(&parser->param, 0, sizeof(parser->param));

      /* Check if response format has this many parameters */

      if (parser->response.info.num_params >= parser->handler->cmd->resp_num)
        {
          /* Reached maximum for parameters, skip to end-of-line. */

          parser->state = PARSER_STATE_SKIP_TO_EOL;
          goto restart;
        }

      /* Check parameter format and start parsing accordingly. */

      parser->param.fmt =
          parser->handler->cmd->resp_format[parser->response.info.num_params];
      parser->state = PARSER_STATE_DO_READ_PARAM;

      /* Pass-through since we are at parameter character already.
       * (No break) */

    case PARSER_STATE_DO_READ_PARAM:
      /*
       * Parse parameter based on parameter format.
       */

      if (parse_parameter_byte(parser, cur_char, end_of_line))
        goto restart;

      break;

    case PARSER_STATE_SKIP_TO_EOL:
      /*
       * Skip to end-of-line.
       */

      /* Is end-of-line? */

      if (end_of_line)
        {
          /* End-of-line, reset line parser. */

          parser->state = PARSER_STATE_RESET;
          break;
        }
      else
        {
          /* Skip other character. */

          break;
        }
    }
}

/****************************************************************************
 * Name: parser_find_response_handler_by_name
 *
 * Description:
 *   ...
 *
 * Input Parameters:
 *   parser   : Parser structure
 *   name     : Name of command
 *
 ****************************************************************************/

static struct at_response_handler_s *
parser_find_response_handler_by_name(struct at_parser_s *parser,
                                     const char *name)
{
  uint32_t i;

  /* Find matching command. */

  for (i = 0; i < ARRAY_SIZE(parser->commands); i++)
    {
      if (!parser->commands[i].active)
        continue;

      DEBUGASSERT(parser->commands[i].cmd != NULL);
      DEBUGASSERT(parser->commands[i].cmd->name != NULL);

      if (parser->commands[i].cmd->name == name ||
          strcasecmp(parser->commands[i].cmd->name, name) == 0)
        {
          /* Found match. */

          return &parser->commands[i];
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: parser_find_response_handler_for_active_command
 *
 * Description:
 *   ...
 *
 * Input Parameters:
 *   parser   : Parser structure
 *
 ****************************************************************************/

static struct at_response_handler_s *
parser_find_response_handler_for_active_command(struct at_parser_s *parser)
{
  uint32_t i;

  /* Find matching command. */

  for (i = 0; i < ARRAY_SIZE(parser->commands); i++)
    {
      if (!parser->commands[i].active)
        continue;

      DEBUGASSERT(parser->commands[i].cmd != NULL);
      DEBUGASSERT(parser->commands[i].cmd->name != NULL);

      if (parser->commands[i].unsolicited)
        continue;

      /* Found match. */

      return &parser->commands[i];
    }

  return NULL;
}

/****************************************************************************
 * Name: modem_command_timeout_callback
 *
 * Description:
 *   Called if active command does not receive response from modem in time.
 *
 * Input Parameters:
 *   timer_id   : ID for timer
 *   arg        : Pointer to parser structure
 *
 ****************************************************************************/

static int modem_command_timeout_callback(struct ubmodem_s *modem,
                                          const int timer_id, void * const arg)
{
  struct at_parser_s *parser = arg;
  struct at_response_handler_s *active;
  struct at_resp_info_s info = {};

  DEBUGASSERT(parser->cmd_timeout_armed && parser->cmd_timeout_id == timer_id);

  /* Only active commands timeout, make sure that this is not leftover timer. */

  active = parser_find_response_handler_for_active_command(parser);
  DEBUGASSERT(active);

  /* Call time-out. */

  info.status = RESP_STATUS_TIMEOUT;
  info.errorcode = -1;
  info.num_params = 0;

  /* Unregister handler before callback, so that callback can
   * register new.
   */

  __ubparser_unregister_response_handler(parser, active->cmd->name);

  /* Mark command timer unarmed. */

  parser->cmd_timeout_armed = false;
  parser->cmd_timeout_id = -1;

  /* Pass timeout to callback. */

  active->callback(modem, active->cmd, &info, NULL, 0, active->callback_priv);

  /* Reset parser. */

  parser->state = PARSER_STATE_RESET;

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

void __ubparse_buffer(struct at_parser_s *parser, const void *buf, size_t buflen)
{
  const uint8_t *pbuf = buf;

  while (buflen--)
    parse_byte(parser, *pbuf++);
}

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

int __ubparser_setup_command_timeout(struct at_parser_s *parser)
{
  int ret;
  uint32_t timeout_msec;
  uint32_t timeout_dsec;
  struct at_response_handler_s *active;
  struct ubmodem_s *modem = parser_to_ubmodem(parser);

  DEBUGASSERT(parser->cmd_timeout_armed == false);

  /* Only active commands timeout, make sure that this is not leftover
   * timer. */

  active = parser_find_response_handler_for_active_command(parser);
  DEBUGASSERT(active);

  timeout_dsec = active->cmd->timeout_dsec;
  if (timeout_dsec == 0)
    timeout_dsec = MODEM_CMD_DEFAULT_TIMEOUT;

  timeout_msec = timeout_dsec * 100;

  ret = __ubmodem_set_timer(modem, timeout_msec,
                          &modem_command_timeout_callback,
                          parser);
  if (ret == ERROR)
    return ERROR;

  parser->cmd_timeout_id = ret;
  parser->cmd_timeout_armed = true;

  return OK;
}

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

void
__ubparser_unregister_response_handler(struct at_parser_s *parser,
                                   const char *name)
{
  struct at_response_handler_s *handler;

  /* Find matching command. */

  handler = parser_find_response_handler_by_name(parser, name);
  if (!handler)
    {
      ubdbg("response handler: '%s' not found!\n", name);

      return;
    }

  /* Found match, reset entry. (Do not clear as structure might still be in
   * use).
   */

  handler->active = false;

  ubdbg("unregistered response handler '%s'.\n", name);
}

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
                                      void *callback_priv, bool unsolicited)
{
  int i;

  ubdbg("registering response handler: '%s'\n", cmd->name);

  /* Find empty slot. Note: first slot is not used for unsolicited result
   * code handlers for faster handling of regular commands.
   */

  for (i = unsolicited ? 1 : 0; i < ARRAY_SIZE(parser->commands); i++)
    if (!parser->commands[i].active)
      break;

  if (i == ARRAY_SIZE(parser->commands))
    {
      /* List full! */

      dbg("List full! Tried to add: '%s'.\n", cmd->name);

      for (i = 0; i < ARRAY_SIZE(parser->commands); i++)
        dbg("%d: '%s', active: %d, unsol: %d\n", i,
            parser->commands[i].cmd->name,
            parser->commands[i].active,
            parser->commands[i].unsolicited);

      usleep(5000*1000);

      assert(i < ARRAY_SIZE(parser->commands));
    }

  if (!((!unsolicited && i == 0) || (unsolicited && i > 0)))
    {
      dbg("err: unsol:%d, i:%d\n", unsolicited, i);
      dbg("cmd[0]: active:%d, name:%s\n",
             parser->commands[0].active,
             parser->commands[0].cmd ? parser->commands[0].cmd->name : NULL
             );
      dbg("new: unsolicited:%d, name:%s\n", unsolicited, cmd->name);

      usleep(5000*1000);

      assert(!unsolicited && i == 0);
      assert(unsolicited && i > 0);
    }

  parser->commands[i] = (struct at_response_handler_s){
    .active        = true,
    .unsolicited   = unsolicited,
    .cmd           = cmd,
    .callback      = callback,
    .callback_priv = callback_priv,
  };
}

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

size_t __ubmodem_stream_max_readbuf_len(const struct at_cmd_def_s *cmd)
{
  const struct modem_response_stream_s *dummy = NULL;
  size_t stream_maxlen = sizeof(dummy->buf);
  unsigned int i;
  bool has_databuflen = false;
  bool has_databuf = false;

  for (i = 0; i < cmd->resp_num; i++)
    {
      size_t resp_size;

      switch (cmd->resp_format[i])
        {
        case RESP_FMT_INT8:
          resp_size = sizeof(int8_t);
          break;
        case RESP_FMT_INT16:
          resp_size = sizeof(int16_t);
          break;
        case RESP_FMT_INT32:
          resp_size = sizeof(int32_t);
          break;
        case RESP_FMT_STRING:
        case RESP_FMT_STRING_TO_EOL:
        case RESP_FMT_QUOTED_STRING:
          resp_size = sizeof(int16_t);
          break;

        case RESP_FMT_QUOTED_DATABUF:
          if (!has_databuflen)
            {
              /* Invalid command definition, RESP_FMT_DATALEN before
               * RESP_FMT_QUOTED_DATABUF required! */

              return 0;
            }
          if (has_databuf)
            {
              /* Multiple RESP_FMT_QUOTED_DATABUF not allowed! */

              return 0;
            }

          resp_size = sizeof(uint16_t) + sizeof(char);
          has_databuf = true;
          break;
        default:
        case RESP_FMT_DATALEN:
          resp_size = 0;
          has_databuflen = true;
          break;
        }

      stream_maxlen -= resp_size;
    }

  if (!has_databuf)
    {
      /* No RESP_FMT_QUOTED_DATABUF in command definition! */

      return 0;
    }

  return stream_maxlen;
}

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

void __ubmodem_assert_debug_print(struct ubmodem_s *modem)
{
  dbg("Modem assert!\n");
  dbg("Modem level: %d, target: %d, state: %d\n",
      modem->level, modem->target_level, modem->state);

#ifdef CONFIG_UBMODEM_PARSER_DEBUG
  {
    char buf[sizeof(modem->parser.last_received) + 1];
    int i, j, k;

    for (k = 0, j = 0, i = modem->parser.last_received_pos;
         j < sizeof(modem->parser.last_received); j++, i++)
      {
        if (i == sizeof(modem->parser.last_received))
          i = 0;

        /* Skip any leading null chars. */

        if (modem->parser.last_received[i] == '\0' && k == 0)
          continue;

        /* Stop at trailing null char. */

        if (modem->parser.last_received[i] == '\0' && k != 0)
          break;

        buf[k++] = modem->parser.last_received[i];
      }

    if (k == 0)
      return;

    buf[k] = '\0';

    dbg("Last data parsed by modem: [%s]\n", buf);
  }
#endif
}

/****************************************************************************
 * Self-tests
 ****************************************************************************/

#ifndef MODEM_DISABLE_SELFTESTS

struct parser_selftest_expected_s {
  int16_t status;
  int16_t error_code;
  int16_t param_num;
  uint16_t stream_len;
  const char *stream;
};

struct parser_selftest_s {
  const struct at_cmd_def_s *cmd;
  const char *from_modem;
  const struct parser_selftest_expected_s *expected;
  uint8_t expected_count;
  uint8_t urc_count;
  uint8_t from_modem_before_command;
};

struct selftest_callback_priv_s {
  const struct parser_selftest_s *tv;
  uint8_t callback_count;
  bool had_error;
};

#define CMD_LINE(x) x "\r\n"

static const struct parser_selftest_s parser_test_vectors[] = {
  {
    .cmd =
      &(const struct at_cmd_def_s){
        .name = "+TEST1",
        .resp_format = (const uint8_t[]){ RESP_FMT_INT8 },
        .resp_num = 1,
      },
    .from_modem = CMD_LINE("OK"),
    .expected_count = 1,
    .expected =
      &(const struct parser_selftest_expected_s){
        .param_num = 0,
        .status = RESP_STATUS_OK,
        .stream = NULL,
      },
  }, {
    .cmd =
      &(const struct at_cmd_def_s){
        .name = "+TEST1",
        .resp_format = (const uint8_t[]){ RESP_FMT_INT8 },
        .resp_num = 1,
      },
    .from_modem = CMD_LINE("ERROR"),
    .expected_count = 1,
    .expected =
      &(const struct parser_selftest_expected_s){
        .param_num = 0,
        .status = RESP_STATUS_ERROR,
        .error_code = -1,
        .stream = NULL,
      },
  }, {
    .cmd =
      &(const struct at_cmd_def_s){
        .name = "+TEST1",
        .resp_format = (const uint8_t[]){ RESP_FMT_INT8 },
        .resp_num = 1,
      },
    .from_modem = CMD_LINE("+URC: 123456789, \"string\", -1234, 121")
                  CMD_LINE("ERROR"),
    .urc_count = 1,
    .expected_count = 1,
    .expected =
      &(const struct parser_selftest_expected_s){
        .param_num = 0,
        .status = RESP_STATUS_ERROR,
        .error_code = -1,
        .stream = NULL,
      },
  }, {
    .cmd =
      &(const struct at_cmd_def_s){
        .name = "+TEST1",
        .resp_format = (const uint8_t[]){ RESP_FMT_INT8 },
        .resp_num = 1,
      },
    .from_modem = CMD_LINE("+CME ERROR: 123"),
    .expected_count = 1,
    .expected =
      &(const struct parser_selftest_expected_s){
        .param_num = 0,
        .status = RESP_STATUS_CME_ERROR,
        .error_code = 123,
        .stream = NULL,
      },
  }, {
    .cmd =
      &(const struct at_cmd_def_s){
        .name = "+TEST1",
        .resp_format = (const uint8_t[]){ RESP_FMT_INT8 },
        .resp_num = 1,
      },
    .from_modem = CMD_LINE("+CMS ERROR: 321")
                  CMD_LINE("+URC: 123456789, \"string\", -1234, 121"),
    .urc_count = 1,
    .expected_count = 1,
    .expected =
      &(const struct parser_selftest_expected_s){
        .param_num = 0,
        .status = RESP_STATUS_CMS_ERROR,
        .error_code = 321,
        .stream = NULL,
      },
  }, {
    .cmd =
      &(const struct at_cmd_def_s){
        .name = "+TEST1",
        .resp_format = (const uint8_t[]){ RESP_FMT_INT8 },
        .resp_num = 1,
      },
    .from_modem = CMD_LINE("+TEST1: 0")
                  CMD_LINE("OK"),
    .expected_count = 1,
    .expected =
      &(const struct parser_selftest_expected_s){
        .status = RESP_STATUS_OK,
        .param_num = 1,
        .stream = /* int8 */ "\x00",
        .stream_len = 1,
      },
  }, {
    .cmd =
      &(const struct at_cmd_def_s){
        .name = "+TEST1",
        .resp_format = (const uint8_t[]){ RESP_FMT_INT8 },
        .resp_num = 1,
      },
    .from_modem = CMD_LINE("+URC: 123456789, \"string\", -1234, 121")
                  CMD_LINE("+TEST1: 128")
                  CMD_LINE("OK"),
    .urc_count = 1,
    .expected_count = 1,
    .expected =
      &(const struct parser_selftest_expected_s){
        .status = RESP_STATUS_OK,
        .param_num = 1,
        .stream = /* int8 */ "\x80",
        .stream_len = 1,
      },
  }, {
    .cmd =
      &(const struct at_cmd_def_s){
        .name = "+TEST2",
        .resp_format =
          (const uint8_t[]){
            RESP_FMT_INT16,
            RESP_FMT_INT8,
            RESP_FMT_INT32,
            RESP_FMT_INT16,
            RESP_FMT_INT32,
            RESP_FMT_INT32,
            RESP_FMT_INT8,
            RESP_FMT_INT16
          },
        .resp_num = 8,
      },
    .from_modem = CMD_LINE("+TEST2: 1, 2, 3, 4, 5, 6, 7, 8")
                  CMD_LINE("OK"),
    .expected_count = 1,
    .expected =
      &(const struct parser_selftest_expected_s){
        .status = RESP_STATUS_OK,
        .param_num = 8,
        .stream = /* int16 */ "\x01\x00"
                  /* int8  */ "\x02"
                  /* int32 */ "\x03\x00\x00\x00"
                  /* int16 */ "\x04\x00"
                  /* int32 */ "\x05\x00\x00\x00"
                  /* int32 */ "\x06\x00\x00\x00"
                  /* int8  */ "\x07"
                  /* int16 */ "\x08\x00",
        .stream_len = 20,
      },
  }, {
    .cmd =
      &(const struct at_cmd_def_s){
        .name = "+TEST2",
        .resp_format =
          (const uint8_t[]){
            RESP_FMT_INT16,
            RESP_FMT_INT8,
            RESP_FMT_INT32,
            RESP_FMT_INT16,
            RESP_FMT_INT32,
            RESP_FMT_INT32,
            RESP_FMT_INT8,
            RESP_FMT_INT16
          },
        .resp_num = 8,
      },
    .from_modem = CMD_LINE("+URC: 123456789, \"string\", -1234, 121")
                  CMD_LINE("+TEST2: 4096, -12,,1234,  -1  ,-1234567  ,\t 255 \t ,")
                  CMD_LINE("OK")
                  CMD_LINE("+URC: 1234567890, \"string2\", -1233, 122"),
    .urc_count = 2,
    .expected_count = 1,
    .expected =
      &(const struct parser_selftest_expected_s){
        .status = RESP_STATUS_OK,
        .param_num = 8,
        .stream = /* int16 */ "\x00\x10"
                  /* int8  */ "\xf4"
                  /* int32 */ "\x00\x00\x00\x00"
                  /* int16 */ "\xd2\x04"
                  /* int32 */ "\xff\xff\xff\xff"
                  /* int32 */ "\x79\x29\xed\xff"
                  /* int8  */ "\xff"
                  /* int16 */ "\x00\x00",
        .stream_len = 20,
      },
  }, {
    .cmd =
      &(const struct at_cmd_def_s){
        .name = "+TEST3",
        .resp_format =
          (const uint8_t[]){
            RESP_FMT_STRING,
            RESP_FMT_INT32
          },
        .resp_num = 2,
      },
    .from_modem = CMD_LINE("+TEST3: test, -1")
                  CMD_LINE("OK"),
    .expected_count = 1,
    .expected =
      &(const struct parser_selftest_expected_s){
        .status = RESP_STATUS_OK,
        .param_num = 2,
        .stream = /* string */ "\x04\x00" "test" "\x00"
                  /* int32  */ "\xff\xff\xff\xff",
        .stream_len = 11,
      },
  }, {
    .cmd =
      &(const struct at_cmd_def_s){
        .name = "+TEST3",
        .resp_format =
          (const uint8_t[]){
            RESP_FMT_STRING,
            RESP_FMT_INT32
          },
        .resp_num = 2,
      },
    .from_modem = CMD_LINE("+TEST3: , -1")
                  CMD_LINE("OK"),
    .expected_count = 1,
    .expected =
      &(const struct parser_selftest_expected_s){
        .status = RESP_STATUS_OK,
        .param_num = 2,
        .stream = /* string */ "\x00\x00" "" "\x00"
                  /* int32  */ "\xff\xff\xff\xff",
        .stream_len = 7,
      },
  }, {
    .cmd =
      &(const struct at_cmd_def_s){
        .name = "+TEST4",
        .resp_format =
          (const uint8_t[]){
            RESP_FMT_INT32,
            RESP_FMT_STRING_TO_EOL
          },
        .resp_num = 2,
      },
    .from_modem = CMD_LINE("+TEST4: +1234567890, abc,,,,,a,s,f,!,\"\",,,")
                  CMD_LINE("OK"),
    .expected_count = 1,
    .expected =
      &(const struct parser_selftest_expected_s){
        .status = RESP_STATUS_OK,
        .param_num = 2,
        .stream = /* int32  */ "\xd2\x02\x96\x49"
                  /* string */ "\x15\x00" "abc,,,,,a,s,f,!,\"\",,," "\x00",
        .stream_len = 28,
      },
  }, {
    .cmd =
      &(const struct at_cmd_def_s){
        .name = "+TEST4",
        .resp_format =
          (const uint8_t[]){
            RESP_FMT_INT32,
            RESP_FMT_STRING_TO_EOL
          },
        .resp_num = 2,
      },
    .from_modem = CMD_LINE("+TEST4:,")
                  CMD_LINE("OK"),
    .expected_count = 1,
    .expected =
      &(const struct parser_selftest_expected_s){
        .status = RESP_STATUS_OK,
        .param_num = 2,
        .stream = /* int32  */ "\x00\x00\x00\x00"
                  /* string */ "\x00\x00" ""  "\x00",
        .stream_len = 7,
      },
  }, {
    .cmd =
      &(const struct at_cmd_def_s){
        .name = "+TEST4",
        .resp_format =
          (const uint8_t[]){
            RESP_FMT_INT32,
            RESP_FMT_STRING_TO_EOL
          },
        .resp_num = 2,
      },
    .from_modem = CMD_LINE("+TEST4:2147483648,a")
                  CMD_LINE("OK"),
    .expected_count = 1,
    .expected =
      &(const struct parser_selftest_expected_s){
        .status = RESP_STATUS_OK,
        .param_num = 2,
        .stream = /* int32  */ "\x00\x00\x00\x80"
                  /* string */ "\x01\x00" "a" "\x00",
        .stream_len = 8,
      },
  }, {
    .cmd =
      &(const struct at_cmd_def_s){
        .name = "+TEST5",
        .resp_format =
          (const uint8_t[]){
            RESP_FMT_QUOTED_STRING,
            RESP_FMT_INT8,
            RESP_FMT_QUOTED_STRING,
            RESP_FMT_QUOTED_STRING,
            RESP_FMT_QUOTED_STRING
          },
        .resp_num = 5,
      },
    .from_modem = CMD_LINE("+TEST5: \"a\", -128, \"c\", \"d\", \"e\"")
                  CMD_LINE("OK"),
    .expected_count = 1,
    .expected =
      &(const struct parser_selftest_expected_s){
        .status = RESP_STATUS_OK,
        .param_num = 5,
        .stream = /* string */ "\x01\x00" "a" "\x00"
                  /* int8   */ "\x80"
                  /* string */ "\x01\x00" "c" "\x00"
                  /* string */ "\x01\x00" "d" "\x00"
                  /* string */ "\x01\x00" "e" "\x00",
        .stream_len = 17,
      },
  }, {
    .cmd =
      &(const struct at_cmd_def_s){
        .name = "+TEST5",
        .resp_format =
          (const uint8_t[]){
            RESP_FMT_QUOTED_STRING,
            RESP_FMT_INT8,
            RESP_FMT_QUOTED_STRING,
            RESP_FMT_QUOTED_STRING,
            RESP_FMT_QUOTED_STRING
          },
        .resp_num = 5,
      },
    .from_modem = CMD_LINE("+TEST5:\"\"  ,127,,\"\",  \"")
                  CMD_LINE("OK"),
    .expected_count = 1,
    .expected =
      &(const struct parser_selftest_expected_s){
        .status = RESP_STATUS_OK,
        .param_num = 5,
        .stream = /* string */ "\x00\x00" "" "\x00"
                  /* int8   */ "\x7f"
                  /* string */ "\x00\x00" "" "\x00"
                  /* string */ "\x00\x00" "" "\x00"
                  /* string */ "\x00\x00" "" "\x00",
        .stream_len = 13,
      },
  }, {
    .cmd =
      &(const struct at_cmd_def_s){
        .name = "+TEST6",
        .resp_format =
          (const uint8_t[]){
            RESP_FMT_DATALEN,
            RESP_FMT_QUOTED_DATABUF,
          },
        .resp_num = 2,
      },
    .from_modem = CMD_LINE("+TEST6: 3, \"ABC\" ")
                  CMD_LINE("OK"),
    .expected_count = 1,
    .expected =
      &(const struct parser_selftest_expected_s){
        .status = RESP_STATUS_OK,
        .param_num = 2,
        .stream = /* buflen */ ""
                  /* buf    */ "\x03\x00" "ABC" "\x00",
        .stream_len = 6,
      },
  }, {
    .cmd =
      &(const struct at_cmd_def_s){
        .name = "+TEST6",
        .resp_format =
          (const uint8_t[]){
            RESP_FMT_DATALEN,
            RESP_FMT_QUOTED_DATABUF,
          },
        .resp_num = 2,
      },
    .from_modem = CMD_LINE("+TEST6: 3, \"A\"C\"")
                  CMD_LINE("OK"),
    .expected_count = 1,
    .expected =
      &(const struct parser_selftest_expected_s){
        .status = RESP_STATUS_OK,
        .param_num = 2,
        .stream = /* buflen */ ""
                  /* buf    */ "\x03\x00" "A\"C" "\x00",
        .stream_len = 6,
      },
  }, {
    .cmd =
      &(const struct at_cmd_def_s){
        .name = "+TEST7",
        .resp_format =
          (const uint8_t[]){
            RESP_FMT_DATALEN,
            RESP_FMT_INT8,
            RESP_FMT_QUOTED_DATABUF,
            RESP_FMT_INT8,
          },
        .resp_num = 4,
      },
    .from_modem = CMD_LINE("+TEST7: 3, -128, \"abc\", -128")
                  CMD_LINE("OK"),
    .expected_count = 1,
    .expected =
      &(const struct parser_selftest_expected_s){
        .status = RESP_STATUS_OK,
        .param_num = 4,
        .stream = /* buflen */ ""
                  /* int8   */ "\x80"
                  /* buf    */ "\x03\x00" "abc" "\x00"
                  /* int8   */ "\x80",
        .stream_len = 8,
      },
  }, {
    .cmd =
      &(const struct at_cmd_def_s){
        .name = "+TEST7",
        .resp_format =
          (const uint8_t[]){
            RESP_FMT_DATALEN,
            RESP_FMT_INT8,
            RESP_FMT_QUOTED_DATABUF,
            RESP_FMT_INT8,
          },
        .resp_num = 4,
      },
    .from_modem = CMD_LINE("+TEST7: 2, -2, \"\"\"\", 3")
                  CMD_LINE("OK"),
    .expected_count = 1,
    .expected =
      &(const struct parser_selftest_expected_s){
        .status = RESP_STATUS_OK,
        .param_num = 4,
        .stream = /* buflen */ ""
                  /* int8   */ "\xfe"
                  /* buf    */ "\x02\x00" "\"\"" "\x00"
                  /* int8   */ "\x03",
        .stream_len = 7,
      },
  }, {
    .cmd =
      &(const struct at_cmd_def_s){
        .name = "+TEST7",
        .resp_format =
          (const uint8_t[]){
            RESP_FMT_DATALEN,
            RESP_FMT_INT8,
            RESP_FMT_QUOTED_DATABUF,
            RESP_FMT_INT8,
          },
        .resp_num = 4,
      },
    .from_modem = CMD_LINE("+TEST7: 35, 123, \"abc\", 3"
                           "\n\rERROR\n\r+CPOWEROFF: 0"
                           "\n\rOK\n\r\", -123")
                  CMD_LINE("OK"),
    .expected_count = 1,
    .expected =
      &(const struct parser_selftest_expected_s){
        .status = RESP_STATUS_OK,
        .param_num = 4,
        .stream = /* buflen */ ""
                  /* int8   */ "\x7b"
                  /* buf    */ "\x23\x00" "abc\", 3\n\rERROR\n\r"
                               "+CPOWEROFF: 0\n\rOK\n\r" "\x00"
                  /* int8   */ "\x85",
        .stream_len = 36 + 4,
      },
  }, {
    .cmd =
      &(const struct at_cmd_def_s){
        .name = "+TEST7",
        .resp_format =
          (const uint8_t[]){
            RESP_FMT_DATALEN,
            RESP_FMT_INT8,
            RESP_FMT_QUOTED_DATABUF,
            RESP_FMT_INT8,
          },
        .resp_num = 4,
      },
    .from_modem = CMD_LINE("+TEST7: 2, -2, \"\"\"\", 3")
                  CMD_LINE("OK"),
    .expected_count = 1,
    .expected =
      &(const struct parser_selftest_expected_s){
        .status = RESP_STATUS_OK,
        .param_num = 4,
        .stream = /* buflen */ ""
                  /* int8   */ "\xfe"
                  /* buf    */ "\x02\x00" "\"\"" "\x00"
                  /* int8   */ "\x03",
        .stream_len = 7,
      },
  }, {
    .cmd =
      &(const struct at_cmd_def_s){
        .name = "+TEST8",
        .resp_format =
          (const uint8_t[]){
            RESP_FMT_INT8,
            RESP_FMT_QUOTED_STRING
          },
        .resp_num = 2,
        .flag_multiple_resp = true,
      },
    .from_modem = CMD_LINE("+TEST8: 1, \"line 1\"")
                  CMD_LINE("+TEST8: 2, \"line 2!\"")
                  CMD_LINE("+TEST8: 3, \"line 3!!\"")
                  CMD_LINE("+TEST8: 4, \"line 4!!!\"")
                  CMD_LINE("OK"),
    .expected_count = 5,
    .expected =
      (const struct parser_selftest_expected_s []){
        {
          .status = RESP_STATUS_LINE,
          .param_num = 2,
          .stream = /* int8   */ "\x01"
                    /* string */ "\x06\x00" "line 1" "\x00",
          .stream_len = 10,
        }, {
          .status = RESP_STATUS_LINE,
          .param_num = 2,
          .stream = /* int8   */ "\x02"
                    /* string */ "\x07\x00" "line 2!" "\x00",
          .stream_len = 11,
        }, {
          .status = RESP_STATUS_LINE,
          .param_num = 2,
          .stream = /* int8   */ "\x03"
                    /* string */ "\x08\x00" "line 3!!" "\x00",
          .stream_len = 12,
        }, {
          .status = RESP_STATUS_LINE,
          .param_num = 2,
          .stream = /* int8   */ "\x04"
                    /* string */ "\x09\x00" "line 4!!!" "\x00",
          .stream_len = 13,
        }, {
          .status = RESP_STATUS_OK,
          .param_num = 0,
          .stream = NULL,
          .stream_len = 0,
        }
      },
  }, {
    .cmd =
      &(const struct at_cmd_def_s){
        .name = "+TEST9",
        .resp_format = NULL,
        .resp_num = 0,
        .flag_data_prompt = true,
      },
    .from_modem = CMD_LINE("+CME ERROR: 123"),
    .expected_count = 1,
    .expected =
      &(const struct parser_selftest_expected_s){
        .param_num = 0,
        .status = RESP_STATUS_CME_ERROR,
        .error_code = 123,
        .stream = NULL,
      },
  }, {
    .cmd =
      &(const struct at_cmd_def_s){
        .name = "+TEST9",
        .resp_format = NULL,
        .resp_num = 0,
        .flag_data_prompt = true,
      },
    .from_modem = CMD_LINE("OK"),
    .expected_count = 1,
    .expected =
      &(const struct parser_selftest_expected_s){
        .param_num = 0,
        .status = RESP_STATUS_OK,
        .error_code = 0,
        .stream = NULL,
      },
  }, {
    .cmd =
      &(const struct at_cmd_def_s){
        .name = "+TEST9",
        .resp_format = NULL,
        .resp_num = 0,
        .flag_data_prompt = true,
      },
    .from_modem = "@",
    .expected_count = 1,
    .expected =
      &(const struct parser_selftest_expected_s){
        .param_num = 0,
        .status = RESP_STATUS_DATAPROMPT,
        .stream = NULL,
      },
  }, {
     .cmd =
       &(const struct at_cmd_def_s){
         .name = "+TEST10",
         .resp_format = (const uint8_t[]){ RESP_FMT_STRING_TO_EOL },
         .resp_num = 1,
         .flag_plain = true,
       },
     .from_modem = CMD_LINE("SHORT RESPONSE")
                   CMD_LINE("OK"),
     .expected_count = 1,
     .expected =
       &(const struct parser_selftest_expected_s){
         .status = RESP_STATUS_OK,
         .param_num = 1,
         .stream = /* string */ "\x0e\x00" "SHORT RESPONSE" "\x00",
         .stream_len = 17,
       },
  }, {
     .cmd =
       &(const struct at_cmd_def_s){
         .name = "+TEST10",
         .resp_format = (const uint8_t[]){ RESP_FMT_STRING_TO_EOL },
         .resp_num = 1,
         .flag_plain = true,
       },
     .from_modem = CMD_LINE("LONG RESPONSE ----012345678901234567890"
                            "abcdefghijklmnopqrstuvwxyz")
                   CMD_LINE("OK"),
     .expected_count = 1,
     .expected =
       &(const struct parser_selftest_expected_s){
         .status = RESP_STATUS_OK,
         .param_num = 1,
         .stream = /* string */ "\x41\x00"
                                "LONG RESPONSE ----012345678901234567890"
                                "abcdefghijklmnopqrstuvwxyz"
                                "\x00",
         .stream_len = 68,
       },
  }, {
      .cmd =
      &(const struct at_cmd_def_s){
        .name = "+CGSN",
        .resp_format = (const uint8_t[]){ RESP_FMT_STRING_TO_EOL },
        .resp_num = 1,
        .flag_plain = true,
      },
    .from_modem = CMD_LINE("004999010640000")
                  CMD_LINE("OK"),
    .expected_count = 1,
    .expected =
      &(const struct parser_selftest_expected_s){
        .status = RESP_STATUS_OK,
        .param_num = 1,
        .stream = /* string */ "\x0f\x00" "004999010640000" "\x00",
        .stream_len = 18,
      },
  }, {
    .cmd =
      &(const struct at_cmd_def_s){
        .name = "+TEST10",
        .resp_format = (const uint8_t[]){ RESP_FMT_STRING_TO_EOL },
        .resp_num = 1,
        .flag_plain = true,
      },
    .from_modem = CMD_LINE("+URC: 123456789, \"string\", -1234, 121")
                  CMD_LINE("SHORT RESPONSE")
                  CMD_LINE("OK"),
    .expected_count = 1,
    .urc_count = 1,
    .expected =
      &(const struct parser_selftest_expected_s){
        .status = RESP_STATUS_OK,
        .param_num = 1,
        .stream = /* string */ "\x0e\x00" "SHORT RESPONSE" "\x00",
        .stream_len = 17,
      },
  }, {
    .cmd =
      &(const struct at_cmd_def_s){
        .name = "+TEST10",
        .resp_format = (const uint8_t[]){ RESP_FMT_STRING_TO_EOL },
        .resp_num = 1,
        .flag_plain = true,
      },
    .from_modem = CMD_LINE("test:::::::::::::::::::::::::::::::::")
                  CMD_LINE("OK"),
    .expected_count = 1,
    .expected =
      &(const struct parser_selftest_expected_s){
        .status = RESP_STATUS_OK,
        .param_num = 1,
        .stream = /* string */ "\x25\x00"
                               "test:::::::::::::::::::::::::::::::::"
                               "\x00",
        .stream_len = 40,
      },
  }, {
    .cmd =
      &(const struct at_cmd_def_s){
        .name = "+TEST11",
        .resp_format = (const uint8_t[]){ RESP_FMT_STRING_TO_EOL },
        .resp_num = 1,
        .flag_plain = true,
        .flag_multiple_resp = true,
      },
    .from_modem = CMD_LINE("123FIRST")
                  CMD_LINE("456SECOND")
                  CMD_LINE("OK"),
    .expected_count = 3,
    .expected =
      (const struct parser_selftest_expected_s []){
        {
          .status = RESP_STATUS_LINE,
          .param_num = 1,
          .stream = /* string */ "\x08\x00" "123FIRST" "\x00",
          .stream_len = 11,
        }, {
          .status = RESP_STATUS_LINE,
          .param_num = 1,
          .stream = /* string */ "\x09\x00" "456SECOND" "\x00",
          .stream_len = 12,
        }, {
          .status = RESP_STATUS_OK,
          .param_num = 0,
          .stream = NULL,
          .stream_len = 0,
        }
      },
  }, {
    /* +TEST12 set tests sending command after part of URC has been already
     * being received.*/
    .cmd =
      &(const struct at_cmd_def_s){
        .name = "+TEST12",
        .resp_format = (const uint8_t[]){ RESP_FMT_INT8 },
        .resp_num = 1,
      },
    .from_modem = CMD_LINE("+URC: 123456789, \"string\", -1234, 121")
                  CMD_LINE("+TEST12: 128")
                  CMD_LINE("OK"),
    .from_modem_before_command = 1,
    .urc_count = 1,
    .expected_count = 1,
    .expected =
      &(const struct parser_selftest_expected_s){
        .status = RESP_STATUS_OK,
        .param_num = 1,
        .stream = /* int8 */ "\x80",
        .stream_len = 1,
      },
  }, {
    .cmd =
      &(const struct at_cmd_def_s){
        .name = "+TEST12",
        .resp_format = (const uint8_t[]){ RESP_FMT_INT8 },
        .resp_num = 1,
      },
    .from_modem = CMD_LINE("+URC: 123456789, \"string\", -1234, 121")
                  CMD_LINE("+TEST12: 128")
                  CMD_LINE("OK"),
    .from_modem_before_command = 4,
    .urc_count = 1,
    .expected_count = 1,
    .expected =
      &(const struct parser_selftest_expected_s){
        .status = RESP_STATUS_OK,
        .param_num = 1,
        .stream = /* int8 */ "\x80",
        .stream_len = 1,
      },
  }, {
    .cmd =
      &(const struct at_cmd_def_s){
        .name = "+TEST12",
        .resp_format = (const uint8_t[]){ RESP_FMT_INT8 },
        .resp_num = 1,
      },
    .from_modem = CMD_LINE("+URC: 123456789, \"string\", -1234, 121")
                  CMD_LINE("+TEST12: 128")
                  CMD_LINE("OK"),
    .from_modem_before_command = 5,
    .urc_count = 1,
    .expected_count = 1,
    .expected =
      &(const struct parser_selftest_expected_s){
        .status = RESP_STATUS_OK,
        .param_num = 1,
        .stream = /* int8 */ "\x80",
        .stream_len = 1,
      },
  }, {
    .cmd =
      &(const struct at_cmd_def_s){
        .name = "+TEST12",
        .resp_format = (const uint8_t[]){ RESP_FMT_INT8 },
        .resp_num = 1,
      },
    .from_modem = CMD_LINE("+URC: 123456789, \"string\", -1234, 121")
                  CMD_LINE("+TEST12: 128")
                  CMD_LINE("OK"),
    .from_modem_before_command = 19,
    .urc_count = 1,
    .expected_count = 1,
    .expected =
      &(const struct parser_selftest_expected_s){
        .status = RESP_STATUS_OK,
        .param_num = 1,
        .stream = /* int8 */ "\x80",
        .stream_len = 1,
      },
  }, {
    .cmd =
      &(const struct at_cmd_def_s){
        .name = "+TEST12",
        .resp_format = (const uint8_t[]){ RESP_FMT_INT8 },
        .resp_num = 1,
      },
    .from_modem = CMD_LINE("+URC: 123456789, \"string\", -1234, 121")
                  CMD_LINE("ERROR"),
    .from_modem_before_command = 19,
    .urc_count = 1,
    .expected_count = 1,
    .expected =
      &(const struct parser_selftest_expected_s){
        .param_num = 0,
        .status = RESP_STATUS_ERROR,
        .error_code = -1,
        .stream = NULL,
      },
  }, {
    .cmd =
      &(const struct at_cmd_def_s){
        .name = "+TEST12",
        .resp_format = (const uint8_t[]){ RESP_FMT_INT8 },
        .resp_num = 1,
      },
    .from_modem = CMD_LINE("+URC: 123456789, \"string\", -1234, 121")
                  CMD_LINE("+CME ERROR: 123"),
    .from_modem_before_command = 19,
    .urc_count = 1,
    .expected_count = 1,
    .expected =
      &(const struct parser_selftest_expected_s){
        .param_num = 0,
        .status = RESP_STATUS_CME_ERROR,
        .error_code = 123,
        .stream = NULL,
      },
  },
};

static const struct at_cmd_def_s urc_cmd = {
  .name = "+URC",
  .resp_format =
    (const uint8_t[]){
      RESP_FMT_INT32,
      RESP_FMT_QUOTED_STRING,
      RESP_FMT_INT16,
      RESP_FMT_INT8,
    },
  .resp_num = 4,
};

static void
parser_selftest_callback(struct ubmodem_s *modem,
                         const struct at_cmd_def_s *cmd,
                         const struct at_resp_info_s *info,
                         const uint8_t *resp_stream,
                         size_t stream_len, void *priv)
{
  struct selftest_callback_priv_s *cbpriv = priv;
  const struct parser_selftest_s *tv = cbpriv->tv;
  int tv_idx = tv - parser_test_vectors;
  const struct parser_selftest_expected_s *expected;
  bool error = false;

  if (cbpriv->callback_count >= tv->expected_count)
    {
      cbpriv->callback_count++;

      dbg("%s(): tv#%d: %s; got %d, expected %d\n",
             __func__, tv_idx, "unexpected number of callbacks",
             cbpriv->callback_count, tv->expected_count);

      cbpriv->had_error = true;
      return;
    }

  expected = &tv->expected[cbpriv->callback_count];

  if (info->status != expected->status)
    {
      dbg("%s(): tv#%d: %s; got %d, expected %d\n",
             __func__, tv_idx, "unexpected status",
             info->status, expected->status);
      error = true;
    }

  if (info->errorcode != expected->error_code)
    {
      dbg("%s(): tv#%d: %s; got %d, expected %d\n",
             __func__, tv_idx, "unexpected error code",
             info->errorcode, expected->error_code);
      error = true;
    }

  if (info->num_params != expected->param_num)
    {
      dbg("%s(): tv#%d: %s; got %d, expected %d\n",
             __func__, tv_idx, "unexpected number of parameters",
             info->num_params, expected->param_num);
      error = true;
    }

  if (stream_len != expected->stream_len)
    {
      dbg("%s(): tv#%d: %s; got %d, expected %d\n",
             __func__, tv_idx, "unexpected response stream length",
             (int)stream_len, (int)expected->stream_len);
      error = true;
    }

  if (stream_len && stream_len == expected->stream_len)
    {
      size_t i;

      if (memcmp(expected->stream, resp_stream, stream_len) != 0)
        {
          dbg("%s(): tv#%d: response stream does not match the expected!\n",
                 __func__, tv_idx);
          error = true;

          dbg("-------got: ");
          for (i = 0; i < stream_len; i++)
            dbg("%02X%c", resp_stream[i], i + 1 == stream_len ? '\n' : ':');

          dbg("--expected: ");
          for (i = 0; i < stream_len; i++)
            dbg("%02X%c", expected->stream[i], i + 1 == stream_len ? '\n' : ':');
        }
    }

  cbpriv->callback_count++;
  if (error)
    {
      cbpriv->had_error = true;
      dbg("%s(): tv#%d: command='%s', from_modem=[%s]\n",
          __func__, tv_idx, tv->cmd->name, tv->from_modem);
    }
}

static void
parser_selftest_urc_callback(struct ubmodem_s *modem,
                             const struct at_cmd_def_s *cmd,
                             const struct at_resp_info_s *info,
                             const uint8_t *resp_stream,
                             size_t stream_len, void *priv)
{
  int *urc_called = priv;
  int32_t first = 0;
  int16_t middle = 0;
  int8_t last = 0;
  const char *str_ptr;
  uint16_t str_len;

  if (*urc_called >= 0)
    (*urc_called)++;

  if (info->status != RESP_STATUS_URC)
    {
      dbg("%s(): %s; got %d, expected %d\n",
             __func__, "unexpected status",
             info->status, RESP_STATUS_URC);
      goto err;
    }

  if (info->errorcode != 0)
    {
      dbg("%s(): %s; got %d, expected %d\n",
             __func__, "unexpected error code",
             info->errorcode, 0);
      goto err;
    }

  if (info->num_params != 4)
    {
      dbg("%s(): %s; got %d, expected %d\n",
             __func__, "unexpected number of parameters",
             info->num_params, 4);
      goto err;
    }

  /* Parse stream. */

  if (!__ubmodem_stream_get_int32(&resp_stream, &stream_len, &first))
    goto err;

  if (first < 0xffff && first >= 0)
    goto err;

  if (!__ubmodem_stream_get_string(&resp_stream, &stream_len, &str_ptr, &str_len))
    goto err;

  if (strnlen(str_ptr, str_len) != str_len)
    goto err;

  if (str_ptr[str_len] != '\0')
    goto err;

  if (!__ubmodem_stream_get_int16(&resp_stream, &stream_len, &middle))
    goto err;

  if (middle >= 0)
    goto err;

  if (!__ubmodem_stream_get_int8(&resp_stream, &stream_len, &last))
    goto err;

  if (last <= 0)
    goto err;

  if (stream_len != 0)
    goto err;

  return;

err:
  *urc_called = -1;
}

static void do_parser_selftest(struct at_parser_s *parser)
{
  struct selftest_callback_priv_s privdata;
  const struct parser_selftest_s *tv;
  unsigned int i;
  int urc_called = 0;
  size_t from_modem_len;

  /* Register unsolicited result code (URC) handler */

  __ubparser_register_response_handler(parser, &urc_cmd,
                                   parser_selftest_urc_callback,
                                   &urc_called, true);

  for (i = 0; i < ARRAY_SIZE(parser_test_vectors); i++)
    {
      tv = &parser_test_vectors[i];
      privdata =
        (struct selftest_callback_priv_s) {
          .tv = tv,
          .callback_count = 0,
          .had_error = false,
        };
      urc_called = 0;
      from_modem_len = strlen(tv->from_modem);

      DEBUGASSERT(tv->from_modem_before_command < from_modem_len);

      if (tv->from_modem_before_command > 0)
        {
          /* Write early bytes emulated modem response to parser. */

          __ubparse_buffer(parser, tv->from_modem,
                           tv->from_modem_before_command);
        }

      /* Register the command handler. */

      __ubparser_register_response_handler(parser, tv->cmd,
                                       parser_selftest_callback,
                                       &privdata, false);

      if (from_modem_len > tv->from_modem_before_command)
        {
          /* Write emulated modem response to parser. */

          __ubparse_buffer(parser,
                           tv->from_modem + tv->from_modem_before_command,
                           from_modem_len - tv->from_modem_before_command);
        }

      if (privdata.callback_count != tv->expected_count || privdata.had_error)
        {
          dbg("Modem parser self-test #%d failed!\n", i);
        }
      if (urc_called < 0 || urc_called != tv->urc_count)
        {
          dbg("Modem parser self-test #%d failed for URC!\n", i);
        }
    }
}

void __ubmodem_parser_selftest(void)
{
  struct at_parser_s *parser;
  static bool done = false;

  if (done)
    return;

  done = true;

  parser = zalloc(sizeof(*parser));
  DEBUGASSERT(parser);

  do_parser_selftest(parser);

  free(parser);
}

#else

void __ubmodem_parser_selftest(void)
{
  return;
}

#endif
