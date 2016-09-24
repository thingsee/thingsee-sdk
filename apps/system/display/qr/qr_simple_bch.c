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
#include <string.h>
#include <stdint.h>

#include "qr_simple_bch.h"

static uint16_t qr_simple_crc_next(uint16_t crc, uint8_t data)
{
    uint16_t eor;
    unsigned int i = 8;

    crc ^= (uint16_t)data << 2;
    do {
            /* This might produce branchless code */
            eor = crc & 0x200 ? 0x137 : 0;
            crc <<= 1;
            crc ^= eor;
    } while (--i);

    return crc;
}


static inline uint16_t qr_simple_crc_final(uint16_t crc)
{
    return crc & 0x3ff;
}

static uint16_t qr_simple_crc_calc(const uint8_t *data, size_t len)
{
    uint16_t crc = 0;

    if (len) do {
            crc = qr_simple_crc_next(crc, *data++);
    } while (--len);

    return qr_simple_crc_final(crc);
}

uint16_t qr_simple_calculate_bch(uint8_t value)
{
    uint16_t ecc = 0;

    ecc = qr_simple_crc_calc(&value, 1);

    return ((value << 10) | ecc);
}
