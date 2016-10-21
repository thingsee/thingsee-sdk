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
#include <sys/types.h>

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>

#include "qr_simple_ecc.h"
#include "qr_simple_fill_matrix.h"
#include "qr_simple_8_bit.h"

#include "qr_main.h"


// TO DO: move numeric encoder into its own *.c and *.h -files


#ifdef CONFIG_THINGSEE_DISPLAY_TRACES
#  define lcd_dbg(x, ...)    dbg(x, ##__VA_ARGS__)
#  define lcd_lldbg(x, ...)  lldbg(x, ##__VA_ARGS__)
#else
#  define lcd_dbg(x, ...)
#  define lcd_lldbg(x, ...)
#endif

#define QR_SIMPLE_NUMERIC               0x0001
#define QR_SIMPLE_CONVERTED_BUFF_SIZE   0x20

#define QR_SIMPLE_MAX_SIZE              44
#define QR_SIMPLE_ECC_NBR               10
#define QR_SIMPLE_BYTE_OUTPUT           0x80

/* Since we are using here version 2 with ECC's level L
 * the total number of codewords is 44 and the error correction
 * code words number is 10. */
#define QR_SIMPLE_ENCODED_BYTES_NBR     (QR_SIMPLE_MAX_SIZE - QR_SIMPLE_ECC_NBR)      /* 44 - 10 */

/* Structure is a better place to save data */
typedef struct qr_simple_encoder_s {
    NXWINDOW hwnd;
} qr_simple_encoder_t;


/* Buffer to keep convertion result */
static uint32_t g_bits[QR_SIMPLE_CONVERTED_BUFF_SIZE] = { 0 };
/* Encoder */

static qr_simple_encoder_t g_encoder;


static uint16_t qr_simple_calc_inds(const uint8_t *data)
{
    const uint16_t ind_mask = 0x03FF;       /* 10-bits mask for character count indicator */
    const uint8_t mode_mask = 0x0F;         /* 4-bits mask for mode indicator */
    uint16_t res = 0;

    res = (QR_SIMPLE_NUMERIC & mode_mask) << 12;
    lcd_dbg("Masked mode: 0x%04X\n", res);
    res |= (strlen((const char *)data) & ind_mask) << 2;
    lcd_dbg("Masked counter + masked mode: 0x%04X\n", res);

    return res;
}

static void qr_simple_append_data(uint32_t value, uint8_t index)
{
    static int16_t offset = -2;

    if (!(index % 3)) {
        if (offset == -2) {
            g_bits[index / 3] = value << 22;
        } else {
            g_bits[(index / 3) - 1] |= value >> (8 - offset);
            value &= (0x0000FF >> offset);
            g_bits[index / 3] = value << (22 + offset + 2);
        }
        offset += 2;

        if (offset >= 8) {
            offset = -2;
        }
    } else if ((index % 3) == 1) {
        g_bits[index / 3] |= value << (12 + offset);
    } else {
        g_bits[index / 3] |= value << (2 + offset);
    }

    lcd_dbg("Offset: %d\n", offset);
}

static void qr_simple_shift_bits(uint8_t *output, uint32_t bytes_nbr)
{
    uint8_t i;
    uint8_t j;

    /* Let's extract data from integers */
    for (i = 2, j = 0; i < bytes_nbr; j++) {
        *(output + i) = (g_bits[j] & 0xFF000000) >> 24;
        i++;
        *(output + i) = (g_bits[j] & 0x00FF0000) >> 16;
        i++;
        *(output + i) = (g_bits[j] & 0x0000FF00) >> 8;
        i++;
        *(output + i) = (g_bits[j] & 0x000000FF);
        i++;
    }

    /* Now we need to shift all bytes (from the second one) to the left by two bits
     * Do not forget that first two bytes have been already shifted earlier */
    for (i = 1; i < bytes_nbr; i++) {
        *(output + i) |= (*(output + i + 1) >> 6);
        *(output + i + 1) <<= 2;
    }
}

static void qr_simple_divide_0_remainder(const uint8_t *data, int length, uint8_t bits_nbr)
{
    int i = 0;
    uint32_t tmp_data = 0;
    uint8_t counter = 0;

    while (i < length) {
        tmp_data = (*(data + i) - '0') * 100;
        i++;
        tmp_data += (*(data + i) - '0') * 10;
        i++;
        tmp_data += (*(data + i) - '0');
        i++;

        lcd_dbg("Data divided: %d\n", tmp_data);
        qr_simple_append_data(tmp_data, counter);
        counter++;
    }

    if ((bits_nbr % 8)) {
        qr_simple_append_data(0, counter);
    }
}

static void qr_simple_divide_1_remainder(const uint8_t *data, int length)
{
    const uint16_t ch_mask = 0x0F;       /* 10-bits mask for character. */
    int i = 0;
    uint32_t tmp_data = 0;
    uint8_t counter = 0;

    while (i < length) {
        tmp_data = (*(data + i) - '0') * 100;
        i++;
        if (*(data + i)) {
            tmp_data += (*(data + i) - '0') * 10;
            i++;
            tmp_data += (*(data + i) - '0');
            i++;
        } else {
            tmp_data /= 100;       /* One numbers only presented */
            /* Let us to apply a 4-bits mask
             * and shift it by six to the left
             * In other words, here the terminator will be append*/
            tmp_data &= ch_mask;
            tmp_data <<= 6;
            qr_simple_append_data(tmp_data, counter);
            counter++;
            tmp_data = 0;
        }
        lcd_dbg("Data divided: %d\n", tmp_data);
        qr_simple_append_data(tmp_data, counter);
        counter++;
    }
}

static void qr_simple_divide_2_remainder(const uint8_t *data, int length)
{
    const uint16_t ch_mask = 0x7F;       /* 10-bits mask for character. */
    int i = 0;
    uint32_t tmp_data = 0;
    uint8_t counter = 0;

    while (i < length) {
        tmp_data = (*(data + i) - '0') * 100;
        i++;
        tmp_data += (*(data + i) - '0') * 10;
        i++;
        if (*(data + i)) {
            tmp_data += (*(data + i) - '0');
            i++;
        } else {
            tmp_data /= 10;       /* Two numbers only presented */
            /* Let us to apply a 7-bits mask
             * and shift it by four to the left
             * In other words, here the terminator will be append*/
            tmp_data &= ch_mask;
            tmp_data <<= 3;
            qr_simple_append_data(tmp_data, counter);
            counter++;
            tmp_data = 0;
        }
        lcd_dbg("Data divided: %d\n", tmp_data);
        qr_simple_append_data(tmp_data, counter);
        counter++;
    }
}

static void qr_simple_add_padwords(uint8_t *output, uint8_t bytes_nbr)
{
    uint8_t i = 0;
    const uint8_t pad_words[2] = { 0xEC, 0x11 };

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
    uint8_t length    = 0;
    uint8_t remainder = 0;
    uint8_t *output_head;
    /* Mode indicator + character count indicator */
    uint32_t bits_nbr = 14;
    uint8_t bytes_nbr = 0;

    output_head = output;

    output[0] = (inds & msb_mask) >> 8;
    output[1] = (inds & lsb_mask);
    output += 2;

    length = strlen((const char *)data);
    remainder = length % 3;
    bits_nbr += length / 3 * 10;

    lcd_dbg("Input data: %s. Remainder: %d\n", data, remainder);

    switch(remainder) {
    case 0:
        bits_nbr += 4;      /* 4 bits terminator */
        qr_simple_divide_0_remainder(data, length, bits_nbr);
        break;
    case 1:
        qr_simple_divide_1_remainder(data, length);
        bits_nbr += 8;      /* 4 bits + 4 bits terminator */
        break;
    case 2:
        qr_simple_divide_2_remainder(data, length);
        bits_nbr += 11;     /* 7 bits + 4 bits terminator */
        break;
    default:
        lcd_dbg("Cannot be here!\n");
        break;
    }

    output = output_head;

    bytes_nbr = bits_nbr / 8 + (!(bits_nbr % 8) ? 0 : 1);
    lcd_dbg("LENGTH: %d\n", bytes_nbr);

    qr_simple_shift_bits(output, bytes_nbr);
    qr_simple_add_padwords(output, bytes_nbr);
    qr_simple_calculate_ecc(output);
}

static int qr_simple_numeric_convert(const uint8_t *data, uint8_t *output)
{
    int ret = 0;
    uint16_t inds = 0;

    inds = qr_simple_calc_inds(data);
    qr_simple_divide_data(data, inds, output);

    return ret;
}

/* Should return Error if render fails */
int qr_main_encode(const uint8_t *str, qr_simple_encoder_algorithm_t algo)
{
    int ret = OK;
    uint8_t output[QR_SIMPLE_BYTE_OUTPUT] = { 0 };
    //TO DO: Add more algorithms: alphanumeric

    switch (algo) {
    case QR_SIMPLE_ENCODER_NUMERIC:
        qr_simple_numeric_convert(str, output);
        break;
    case QR_SIMPLE_ENCODER_8BIT:
        qr_simple_8bit_convert(str, output);
        break;
    default:
        lcd_dbg("Cannot be here\n");
        break;
    }

    qr_simple_place_modules(output, g_encoder.hwnd);

    return ret;
}

void qr_main_init(NXWINDOW hwnd)
{
    g_encoder.hwnd = hwnd;
}
