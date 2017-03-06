/****************************************************************************
 * apps/system/ubmodem/ubmodem_pdu_util.c
 *
 *   Copyright (C) 2015-2017 Haltian Ltd. All rights reserved.
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
 **************************************************************************/

/****************************************************************************
 * 'decode' is from "http://bjoern.hoehrmann.de/utf-8/decoder/dfa/",
 * with MIT license:
 *
 * Copyright (c) 2008-2009 Bjoern Hoehrmann <bjoern@hoehrmann.de>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>
#include <assert.h>
#include <errno.h>
#include <arpa/inet.h>

#include "ubmodem_pdu_util.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define UTF8_ACCEPT 0
#define UTF8_REJECT 1

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t utf8d[] =
{
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, /* 00..1f */
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, /* 20..3f */
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, /* 40..5f */
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, /* 60..7f */
  1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9, /* 80..9f */
  7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7, /* a0..bf */
  8,8,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2, /* c0..df */
  0xa,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x4,0x3,0x3, /* e0..ef */
  0xb,0x6,0x6,0x6,0x5,0x8,0x8,0x8,0x8,0x8,0x8,0x8,0x8,0x8,0x8,0x8, /* f0..ff */
  0x0,0x1,0x2,0x3,0x5,0x8,0x7,0x1,0x1,0x1,0x4,0x6,0x1,0x1,0x1,0x1, /* s0..s0 */
  1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,0,1,0,1,1,1,1,1,1, /* s1..s2 */
  1,2,1,1,1,1,1,2,1,2,1,1,1,1,1,1,1,1,1,1,1,1,1,2,1,1,1,1,1,1,1,1, /* s3..s4 */
  1,2,1,1,1,1,1,1,1,2,1,1,1,1,1,1,1,1,1,1,1,1,1,3,1,3,1,1,1,1,1,1, /* s5..s6 */
  1,3,1,1,1,1,1,3,1,3,1,1,1,1,1,1,1,3,1,1,1,1,1,1,1,1,1,1,1,1,1,1, /* s7..s8 */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: decode
 *
 * Description:
 *   UTF-8 is a variable length character encoding. To decode a character one
 *   or more bytes have to be read from a string. The decode function
 *   implements a single step in this process. It takes two parameters
 *   maintaining state and a byte, and returns the state achieved after
 *   processing the byte. Specifically, it returns the value UTF8_ACCEPT (0)
 *   if enough bytes have been read for a character, UTF8_REJECT (1) if the
 *   byte is not allowed to occur at its position, and some other positive
 *   value if more bytes have to be read.
 *
 *   When decoding the first byte of a string, the caller must set the state
 *   variable to UTF8_ACCEPT. If, after decoding one or more bytes the state
 *   UTF8_ACCEPT is reached again, then the decoded Unicode character value
 *   is available through the codep parameter. If the state UTF8_REJECT is
 *   entered, that state will never be exited unless the caller intervenes.
 *
 ****************************************************************************/

static uint32_t decode(uint32_t state, uint32_t *codep, uint32_t byte)
{
  uint32_t type = utf8d[byte];

  *codep = (state != UTF8_ACCEPT) ?
    (byte & 0x3fu) | (*codep << 6) :
    (0xff >> type) & (byte);

  state = utf8d[256 + state * 16 + type];
  return state;
}

/****************************************************************************
 * Name: tohex
 *
 * Description:
 *   Convert input nibble to ASCII hex representation.
 *
 ****************************************************************************/

static inline int tohex(int value)
{
  return value + ((value > 9) ? ('A' - 10) : '0');
}

/****************************************************************************
 * Name: push_pdu_char
 *
 * Description:
 *   Output 8-bit value as ASCII hex to target string buffer.
 *
 ****************************************************************************/

static inline int push_pdu_char(char *pdubuf, int pos, uint8_t value)
{
  if (pdubuf)
    {
      pdubuf[pos + 0] = tohex((value >> 4) & 0xF);
      pdubuf[pos + 1] = tohex(value & 0xF);
    }

  return pos + 2;
}

/****************************************************************************
 * Name: fromhex
 *
 * Description:
 *   Parse two-byte input ASCII hex to 8-bit value.
 *
 ****************************************************************************/

static inline uint8_t fromhex(const char hex[2])
{
  uint8_t num[2];

  num[0] = hex[0] - ((hex[0] > '9') ? ('A' - 10) : '0');
  num[1] = hex[1] - ((hex[1] > '9') ? ('A' - 10) : '0');

  return (num[0] << 4) + num[1];
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __utf8_decode
 *
 * Description:
 *   Decode one codepoint from input UTF-8 string and update input offset.
 *
 ****************************************************************************/

int32_t __utf8_decode(const char *buf, size_t *offset, size_t buflen)
{
  const uint8_t *in;
  size_t inlen;
  uint32_t codep = 0;
  uint32_t state = UTF8_ACCEPT;

  if (*offset >= buflen)
    {
      return 0;
    }

  inlen = buflen - *offset;
  in = (const uint8_t *)(buf + *offset);

  /* Decode one code-point from utf-8 string. */

  while (inlen > 0)
    {
      state = decode(state, &codep, *in);
      inlen--;
      in++;

      switch (state)
        {
        case UTF8_ACCEPT:
          /* Check for overlong encodings. */
          *offset = (buflen - inlen);
          return codep;
        case UTF8_REJECT:
          *offset += 1;
          return -1;
        }
    }

  /* Truncated UTF-8 string? */

  *offset = (buflen - inlen);
  return -1;
}

/****************************************************************************
 * Name: __ucs2_encode
 *
 * Description:
 *   Encode one codepoint to 16-bit UCS-2 character. Unsupported codepoints
 *   are replaced with replacement character � (0xFFFD).
 *
 ****************************************************************************/

uint16_t __ucs2_encode(int32_t codep)
{
  /* UCS-2 can encode ranges 0x0000..0xd7ff and 0xE000..0xFFFF. Inject
   * replacement character � (0xFFFD) for unsupported characters.
   */

  if (codep >= 0x0000 && codep <= 0xd7ff)
    {
      return (uint16_t)codep;
    }
  else if (codep >= 0xe000 && codep <= 0xffff)
    {
      return (uint16_t)codep;
    }
  else
    {
      return 0xfffd;
    }
}

/****************************************************************************
 * Name: __utf8_num_code_points
 *
 * Description:
 *   Count number of codepoints for input UTF-8 string.
 *
 ****************************************************************************/

size_t __utf8_num_code_points(const char *buf, size_t buflen,
                              bool stop_at_null)
{
  size_t pos = 0;
  size_t count = 0;

  /* Get code-point length of UTF-8 string. */

  do
    {
      int32_t codep;

      if (pos == buflen)
        break;

      codep = __utf8_decode(buf, &pos, buflen);
      if (stop_at_null && codep == 0)
        break;

      count++;
    }
  while (true);

  return count;
}

/****************************************************************************
 * Name: __utf8_to_ucs2
 *
 * Description:
 *   Convert input UTF-8 string to UCS-2 string.
 *
 ****************************************************************************/

size_t __utf8_to_ucs2(uint16_t *out, size_t outcount,
                      const char *inbuf, size_t *inoffset, size_t inbuflen)
{
  size_t count = 0;

  if (*inoffset >= inbuflen)
    return 0;

  while (outcount > 0)
    {
      int32_t codep;

      codep = __utf8_decode(inbuf, inoffset, inbuflen);
      if (codep == 0)
        break;

      *out = __ucs2_encode(codep);
      out++;
      outcount--;
      count++;
    }

  return count;
}

/****************************************************************************
 * Name: __ubmodem_utf8_to_pdu
 *
 * Description:
 *   Convert input UTF-8 string to Unicode PDU user data.
 *
 * Input Parameters:
 *   *message : The UTF-8 message to convert
 *   **pduout : Pointer to an array where the pdu-bytes will be stored in, this
 *              function allocates the necessary memory for the array
 *
 * Returned valued:
 *   Returns the number of bytes stored in the pdu-array, or -ENOMEM in case of
 *   out-of-memory error or -EINVAL in case of invalid input.
 *
 *  Note:  Don't forget to free() the pdu-array when you're finished with it.
 ****************************************************************************/

ssize_t __ubmodem_utf8_to_pdu(const char *message, uint8_t **pduout)
{
  size_t mesglen = strlen(message);
  size_t mesgchars = __utf8_num_code_points(message, mesglen, true);
  uint16_t *ucs2;
  size_t ucs2len;
  size_t pos = 0;
  size_t i;

  /* Decode UTF-8 to UCS2. */

  ucs2 = malloc(sizeof(uint16_t) * mesgchars);
  if (!ucs2)
    {
      return -ENOMEM;
    }

  ucs2len = __utf8_to_ucs2(ucs2, mesgchars, message, &pos, mesglen);
  if (ucs2len != mesgchars || pos != mesglen)
    {
      /* Parsing error. */

      free(ucs2);
      return -EINVAL;
    }

  /* Convert from host-endian to big-endian. */

  pos = 0;
  for (i = 0; i < ucs2len; i++)
    {
      ucs2[i] = htons(ucs2[i]);
    }

  *pduout = (void *)ucs2;
  return ucs2len * 2;
}

/****************************************************************************
 * Name: __ubmodem_address_to_pdu
 *
 * Description:
 *   Convert an phone address from text format to PDU format
 *
 * Input Parameters:
 *   *address : The address to convert
 *   **pdu    : Pointer to an array where the pdu-bytes will be stored in, this
 *              function allocates the necessary memory for the array
 *
 * Returned valued:
 *   Returns the number of bytes stored in the pdu-array, or -ENOMEM in case of
 *   out-of-memory error.
 *
 *  Note:  Don't forget to free() the pdu-array when you're finished with it.
 ****************************************************************************/

int __ubmodem_address_to_pdu(const char *address, uint8_t **pdu)
{
  bool is_inter = false;
  size_t addrlen = strlen(address);
  uint8_t prev = 0;
  int currnum = 0;
  int numcount = 0;
  int pdulen;
  size_t i;
  int pos;

  if (addrlen == 0)
    {
      *pdu = NULL;
      return 0;
    }

  if (addrlen > 0 && address[0] == '+')
    is_inter = true;

  /* Check address validity. */

  for (i = is_inter; i < addrlen; i++)
    {
      if (isspace(address[i]) || address[i] == '-')
        {
          /* Skip white-space. */

          continue;
        }

      if (address[i] < '0' || address[i] > '9')
        {
          /* Invalid number. */

          return -1;
        }

      numcount++;
    }

  /* Allocate enough memory for pdu. */

  pdulen = 1 + (numcount + 1) / 2;
  *pdu = malloc(pdulen);
  if (*pdu == NULL)
    {
      return -ENOMEM;
    }

  /* Save address type. */

  pos = 0;
  (*pdu)[pos++] = is_inter ? 0x91 : 0x81;

  /* Convert address to PDU format. */

  for (i = is_inter; i < addrlen; i++)
    {
      if (isspace(address[i]) || address[i] == '-')
        {
          /* Skip white-space. */

          continue;
        }

      if ((currnum & 1) == 0)
        {
          prev = address[i] - '0';
        }
      else
        {
          (*pdu)[pos++] = ((address[i] - '0') << 4) | prev;
        }

      currnum++;
    }

  if ((numcount & 1) == 1)
    {
      (*pdu)[pos++] = (0xF << 4) | prev;
    }

  assert(pos == pdulen);

  return pdulen;
}

/****************************************************************************
 * Name: __ubmodem_pdu_multipart_iterator_init
 *
 * Description:
 *   Initialize iterator structure for concatenated SMS creation.
 *
 * Input Parameters:
 *   *iter      : Iterator structure
 *   msg_pdulen : Length of PDU message part
 *   ref_num    : Reference number used for concatenated SMS.
 *
 * Returned valued:
 *   Returns 'true' if initialized successfully, 'false' if message length
 *   is too long to handle.
 ****************************************************************************/

bool __ubmodem_pdu_multipart_iterator_init(struct multipdu_iter_s *iter,
                                           size_t msg_pdulen, uint8_t ref_num)
{
  size_t part_count;

  if (msg_pdulen <= 70 * 2)
    {
      /* Single SMS mode can transfer 70 characters using USC2 encoding. */

      part_count = 1;
    }
  else
    {
      /* Additional concatenated SMS header takes 6 bytes, thus can send 67
       * characters per message. */

      part_count = msg_pdulen / (67 * 2) + !!(msg_pdulen % (67 * 2));
    }

  if (part_count > 0xff)
    {
      /* Too long message for concatenated SMS. */

      return false;
    }

  iter->msg_pos = 0;
  iter->ref_num = ref_num;
  iter->part_pos = 0;
  iter->part_count = part_count;

  return true;
}

/****************************************************************************
 * Name: __ubmodem_prepare_pdu_string
 *
 * Description:
 *   Prepare next PDU for concatenated SMS.
 *
 * Input Parameters:
 *   *recv_pdu   : Phone number PDU
 *   recv_pdulen : Length of Phone number PDU
 *   *msg_pdu    : Message PDU
 *   msg_pdulen  : Length of Message PDU
 *   *iter       : Concatenated SMS iterator structure
 *   **pduout    : Pointer to an array where the pdu-bytes will be stored in,
 *                 this function allocates the necessary memory for the array
 *
 * Returned valued:
 *   Returns length of resulting PDU and negative value in case of error.
 *
 *  Note:  Don't forget to free() the pdu-array when you're finished with it.
 ****************************************************************************/

ssize_t __ubmodem_prepare_pdu_string(const uint8_t *recv_pdu,
                                     size_t recv_pdulen,
                                     const uint8_t *msg_pdu,
                                     size_t msg_pdulen,
                                     struct multipdu_iter_s *iter,
                                     char **pduout)
{
  size_t pos;
  size_t pdulen = 0;
  size_t i;
  char *pdu = NULL;
  bool multipart = iter->part_count > 1;
  uint8_t pduflags = 0x01; /* SMS-SUBMIT */

  /* Adjust message PDU depending on current concatenated state. */

  msg_pdu += iter->msg_pos;
  msg_pdulen -= iter->msg_pos;
  if (multipart)
    {
      if (msg_pdulen > 67 * 2)
        {
          msg_pdulen = 67 * 2;
        }
    }

  if (msg_pdulen == 0)
    {
      /* Already done? */

      *pduout = NULL;
      return 0;
    }

  if (iter->part_pos == iter->part_count)
    {
      /* Already done? */

      *pduout = NULL;
      return 0;
    }

  if (multipart)
    pduflags |= 0x40; /* User Data Header present */

  do
    {
      /* Fill in data for PDU. */

      pos = 0;
      pos = push_pdu_char(pdu, pos, 0x00);         /* Length of SMSC info */
      pos = push_pdu_char(pdu, pos, pduflags);     /* First octet */
      pos = push_pdu_char(pdu, pos, 0x00);         /* TP-Message-Reference */
      pos = push_pdu_char(pdu, pos, (recv_pdulen - 1) * 2); /* Address-Length */
      for (i = 0; i < recv_pdulen; i++)
        {
          pos = push_pdu_char(pdu, pos, recv_pdu[i]); /* Receiver address */
        }
      pos = push_pdu_char(pdu, pos, 0x00);         /* TP-PID */
      pos = push_pdu_char(pdu, pos, 0x08);         /* TP-DCS: UCS2 data coding */

      if (multipart)
        {
          /* TP-User-Data-Length (header + data). */

          pos = push_pdu_char(pdu, pos, msg_pdulen + 6);

          /* Insert 'concatenated message' user data header. */

          pos = push_pdu_char(pdu, pos, 0x05);     /* Length of UDH. */
          pos = push_pdu_char(pdu, pos, 0x00);     /* IEI, concatenated message. */
          pos = push_pdu_char(pdu, pos, 0x03);     /* Length of IEI. */

          /* Reference number. */

          pos = push_pdu_char(pdu, pos, iter->ref_num);

          /* Total count of parts. */

          pos = push_pdu_char(pdu, pos, iter->part_count);

          /* Current part. */

          pos = push_pdu_char(pdu, pos, iter->part_pos + 1);
        }
      else
        {
          pos = push_pdu_char(pdu, pos, msg_pdulen);   /* TP-User-Data-Length */
        }

      for (i = 0; i < msg_pdulen; i++)
        {
          pos = push_pdu_char(pdu, pos, msg_pdu[i]); /* SMS message */
        }

      if (pdu == NULL)
        {
          /* Allocate buffer for PDU. */

          pdulen = pos;
          pdu = malloc(pdulen + 1);
          if (pdu == NULL)
            {
              return -ENOMEM;
            }
        }
      else
        {
          break;
        }
    }
  while (true);
  pdu[pos] = '\0';

  iter->msg_pos += msg_pdulen;
  iter->part_pos += 1;

  *pduout = pdu;
  return pdulen;
}

/****************************************************************************
 * Name: __ubmodem_get_pdu_smsc_len
 *
 * Description:
 *   Get length of SMSC header for PDU.
 ****************************************************************************/

ssize_t __ubmodem_get_pdu_smsc_len(const char *pduhex)
{
  size_t smsclen = 1 + fromhex(&pduhex[0]);
  return smsclen;
}
