/****************************************************************************
 * Copyright (C) 2015 Haltian Ltd. All rights reserved.
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
 * Author: Dmitry Nikolaev <dmitry.nikolaev@haltian.com>
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "qr_simple_ecc.h"
#include "qr_simple_8_bit.h"


#ifdef CONFIG_THINGSEE_DISPLAY_TRACES
#  define lcd_dbg(x, ...)    dbg(x, ##__VA_ARGS__)
#  define lcd_lldbg(x, ...)  lldbg(x, ##__VA_ARGS__)
#else
#  define lcd_dbg(x, ...)
#  define lcd_lldbg(x, ...)
#endif


#define QR_SIMPLE_8_BIT                 0x0004

#define QR_SIMPLE_MAX_SIZE              44
#define QR_SIMPLE_ECC_NBR               10

/* Since we are using here version 2 with ECC's level L
 * the total number of codewords is 44 and the error correction
 * code words number is 10. */
#define QR_SIMPLE_ENCODED_BYTES_NBR     (QR_SIMPLE_MAX_SIZE - QR_SIMPLE_ECC_NBR)      /* 44 - 10 */


static uint16_t qr_simple_calc_inds(const uint8_t *data)
{
    const uint16_t ind_mask = 0x0FF;        /* 8-bits mask for character count indicator */
    const uint8_t mode_mask = 0x0F;         /* 4-bits mask for mode indicator */
    uint16_t res;

    res = (QR_SIMPLE_8_BIT & mode_mask) << 12;
    lcd_dbg("Masked mode: 0x%04X\n", res);
    res |= (strlen((const char *)data) & ind_mask) << 4;
    lcd_dbg("Masked counter + masked mode: 0x%04X\n", res);

    return res;
}

static void qr_simple_append_data(const uint8_t *data, uint8_t *output)
{
    size_t length;

    length = strlen((const char *)data);

    memcpy(output, (const int8_t *)data, length);
}

static void qr_simple_shift_bits(uint8_t *output, uint32_t bytes_nbr)
{
    uint8_t i;

    /* We need to shift all bytes (from the second one) to the left by four bits
     * Do not forget that first two bytes have been already shifted earlier */
    for (i = 1; i < bytes_nbr; i++) {
        *(output + i) |= (*(output + i + 1) >> 4);
        *(output + i + 1) <<= 4;
    }
}

static void qr_simple_add_padwords(uint8_t *output, uint8_t bytes_nbr)
{
    uint8_t i;
    const uint8_t pad_words[] = { 0xEC, 0x11 };

    for (i = bytes_nbr; i < QR_SIMPLE_ENCODED_BYTES_NBR; i++) {
        *(output + i) = pad_words[0];
        i++;
        *(output + i) = pad_words[1];
    }
}

static void qr_simple_calculate_ecc(uint8_t *output)
{
    uint8_t i;
    uint8_t *ecc_buf;

    qr_simple_calculate_polynomial(output);
    ecc_buf = (uint8_t *)qr_simple_get_ecc();

    memcpy((output + QR_SIMPLE_ENCODED_BYTES_NBR), ecc_buf, QR_SIMPLE_ECC_NBR);

    for (i = 0; i < QR_SIMPLE_MAX_SIZE; i++) {
        lcd_dbg("Output byte: %d index = %d\n", output[i], i);
    }
}

static void qr_simple_divide_data(const uint8_t *data, uint16_t inds, uint8_t *output)
{
    const uint16_t msb_mask = 0xFF00;
    const uint16_t lsb_mask = 0x00FF;
    uint8_t bytes_nbr;

    output[0] = (inds & msb_mask) >> 8;
    output[1] = (inds & lsb_mask);

    bytes_nbr = strlen((const char *)data) + 2;

    qr_simple_append_data(data, output + 2);

    qr_simple_shift_bits(output, bytes_nbr);
    qr_simple_add_padwords(output, bytes_nbr);
    qr_simple_calculate_ecc(output);
}

/* It should return ERROR on nx_bitmap fail
 * TO DO: implement later and clean up code */
int qr_simple_8bit_convert(const uint8_t *data, uint8_t *output)
{
    int ret = OK;
    uint16_t inds;

    inds = qr_simple_calc_inds(data);
    qr_simple_divide_data(data, inds, output);

    return ret;
}
