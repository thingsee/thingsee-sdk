/****************************************************************************
 * Copyright (C) 2014-2015 Haltian Ltd. All rights reserved.
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
#include <stdint.h>

#include <oled_image_template.h>

#if CONFIG_THINGSEE_DISPLAY_INC_IMG_EXAMPLE

static const uint8_t g_battery5_bits[] = {
    0x80, 0x07, 0xFC, 0x00, 0x80, 0x07, 0xFC, 0x00, 0x80, 0x17, 0xFE, 0x00,
    0x80, 0x3F, 0xFB, 0x80, 0x80, 0xE0, 0x00, 0xC0, 0x80, 0x8F, 0xFE, 0x60,
    0x81, 0xBF, 0xFF, 0x20, 0x81, 0x7F, 0xFF, 0xB0, 0x83, 0x7F, 0xFF, 0x90,
    0x81, 0x7F, 0xFF, 0x90, 0x81, 0x7F, 0xFF, 0x90, 0x81, 0x00, 0x00, 0x10,
    0x83, 0x7F, 0xFF, 0x90, 0x81, 0x7F, 0xFF, 0x90, 0x81, 0x7F, 0xFF, 0x90,
    0x81, 0x7F, 0xFF, 0x90, 0x83, 0x7F, 0xFF, 0x90, 0x81, 0x00, 0x00, 0x10,
    0x81, 0x7F, 0xFF, 0x90, 0x81, 0x7F, 0xFF, 0x90, 0x83, 0x7F, 0xFF, 0x90,
    0x81, 0x7F, 0xFF, 0x90, 0x81, 0x7F, 0xFF, 0x90, 0x81, 0x00, 0x00, 0x10,
    0x83, 0x7F, 0xFF, 0x90, 0x81, 0x7F, 0xFF, 0x90, 0x81, 0x7F, 0xFF, 0x90,
    0x81, 0x3F, 0xFF, 0xB0, 0x81, 0x9F, 0xFF, 0x20, 0x80, 0xC5, 0x54, 0x60,
    0x80, 0x70, 0x01, 0xC0, 0x80, 0x3F, 0xFF, 0x00

};



const uint8_t *oled_image_bat_get_img(void)
{
    return g_battery5_bits;
}

#endif
