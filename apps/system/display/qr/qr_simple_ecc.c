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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "qr_simple_polynom.h"
#include "qr_simple_ecc.h"

#define QR_SIMPLE_DATA_SIZE                 34
#define QR_SIMPLE_GEN_SIZE                  11
#define QR_SIMPLE_MULT_SIZE                 11

static const uint8_t g_generator_polynomial[QR_SIMPLE_GEN_SIZE] = {
    0, 251, 67, 46, 61, 118, 70, 64, 94, 32, 45
};

static uint8_t g_mult_polynomial[QR_SIMPLE_MULT_SIZE] = { 0 };

static void qr_simple_ecc_move2left(const uint8_t *input, uint8_t index)
{
    uint8_t i = 0;

    for (i = 0; i < QR_SIMPLE_MULT_SIZE - 1; i++) {
        g_mult_polynomial[i] = g_mult_polynomial[i + 1];
    }

    /* For debugging purposes */
    i = index + QR_SIMPLE_MULT_SIZE;

    /* Our polynomial is 44 members long but only 34 contains real data.
     * Thus, 10 last ones are zeroes. */

    if (index < 23) {
        g_mult_polynomial[QR_SIMPLE_MULT_SIZE - 1] = *(input + i);
    } else {
        g_mult_polynomial[QR_SIMPLE_MULT_SIZE - 1] = 0;
    }
}

/* The input array must be at least 44 bytes long */
void qr_simple_calculate_polynomial(const uint8_t *input)
{
    const uint8_t max_value = 0xFF;
    uint8_t i, j;
    uint8_t fetched_power;
    /* Must be 16 bits long. Sometimes the value can be longer than 8 bits */
    uint16_t calculated_power;
    uint8_t fetched_value;
    uint8_t calculated_value;
    /* Powers and values to be used in while calculating a correct ECC */
    uint8_t *powers;
    uint8_t *values;

    memcpy(g_mult_polynomial, input, QR_SIMPLE_MULT_SIZE);

    powers = (uint8_t *)qr_simple_get_index_polynomial();
    values = (uint8_t *)qr_simple_get_value_polynomial();

    for (i = 0; i < QR_SIMPLE_DATA_SIZE; i++) {
        fetched_power = powers[g_mult_polynomial[0]];

        for (j = 0; j < QR_SIMPLE_GEN_SIZE; j++) {
            calculated_power = fetched_power + g_generator_polynomial[j];
            if (calculated_power >= max_value) {
                calculated_power -= max_value;
            }

            fetched_value = values[calculated_power];
            calculated_value = g_mult_polynomial[j] ^ fetched_value;
            g_mult_polynomial[j] = calculated_value;
        }

        qr_simple_ecc_move2left(input, i);
    }
}

const uint8_t *qr_simple_get_ecc(void)
{
    return (const uint8_t *)g_mult_polynomial;
}
