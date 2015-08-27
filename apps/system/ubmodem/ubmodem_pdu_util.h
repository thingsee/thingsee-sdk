/****************************************************************************
 * apps/system/ubmodem/tests/ubmodem_pdu_util.h
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
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

#ifndef __TESTS_UBMODEM_PDU_UTIL_H_
#define __TESTS_UBMODEM_PDU_UTIL_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <assert.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

struct multipdu_iter_s
{
  size_t msg_pos;
  uint8_t part_pos;
  uint8_t part_count;
  uint8_t ref_num;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: __utf8_decode
 *
 * Description:
 *   Decode one codepoint from input UTF-8 string and update input offset.
 *
 ****************************************************************************/

int32_t __utf8_decode(const char *buf, size_t *offset, size_t buflen);


/****************************************************************************
 * Name: __ucs2_encode
 *
 * Description:
 *   Encode one codepoint to 16-bit UCS-2 character. Unsupported codepoints
 *   are replaced with replacement character � (0xFFFD).
 *
 ****************************************************************************/

uint16_t __ucs2_encode(int32_t codep);


/****************************************************************************
 * Name: __utf8_num_code_points
 *
 * Description:
 *   Count number of codepoints for input UTF-8 string.
 *
 ****************************************************************************/

size_t __utf8_num_code_points(const char *buf, size_t buflen,
                              bool stop_at_null);

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

size_t __utf8_to_ucs2(uint16_t *out, size_t outcount,
                      const char *inbuf, size_t *inoffset, size_t inbuflen);

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

ssize_t __ubmodem_utf8_to_pdu(const char *message, uint8_t **pduout);

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
 *   Returns the number of bytes stored in the pdu-array, or -1 in case of
 *   out-of-memory error.
 *
 *  Note:  Don't forget to free() the pdu-array when you're finished with it.
 ****************************************************************************/

int __ubmodem_address_to_pdu(const char *address, uint8_t **pdu);

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
                                           size_t msg_pdulen, uint8_t ref_num);

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
                                     char **pduout);

/****************************************************************************
 * Name: __ubmodem_get_pdu_smsc_len
 *
 * Description:
 *   Get length of SMSC header for PDU.
 ****************************************************************************/

ssize_t __ubmodem_get_pdu_smsc_len(const char *pduhex);

#endif /* __TESTS_UBMODEM_PDU_UTIL_H_ */

