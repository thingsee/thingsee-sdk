/******************************************************************************
 * drivers/lcd/memlcd_font.h
 * Character console font for Sharp Memory LCD.
 *
 *   Copyright (C) 2014 Haltian Ltd. All rights reserved.
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
 *    without specific prior writen permission.
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
 ******************************************************************************/

#ifdef CONFIG_MEMLCD_CHARACTER_CONSOLE_MODE

/*
 * NOTE: Included once from memlcd.c !
 */

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/* Pixel/bit size of font */

#define MEMLCD_FONT_BITHEIGHT 32
#define MEMLCD_FONT_BITWIDTH 16

/* Up-scaling of stored font */

#define MEMLCD_FONT_DATA_BITHEIGHT 8
#define MEMLCD_FONT_DATA_BITWIDTH 8

/******************************************************************************
 * Private Data
 ******************************************************************************/

/* Macro to change font-face bits LSB to MSB and invert bit values. */

#define X(b) \
  ((((b >> 7) & 1) << 0) ^ \
   (((b >> 6) & 1) << 1) ^ \
   (((b >> 5) & 1) << 2) ^ \
   (((b >> 4) & 1) << 3) ^ \
   (((b >> 3) & 1) << 4) ^ \
   (((b >> 2) & 1) << 5) ^ \
   (((b >> 1) & 1) << 6) ^ \
   (((b >> 0) & 1) << 7) ^ \
    0xff)

/* Macros for filling character space with blank font-face. */

#define INSERT_BLANK { \
    X(0b10101010), \
    X(0b01010100), \
    X(0b10101010), \
    X(0b01010100), \
    X(0b10101010), \
    X(0b01010100), \
    X(0b10101010), \
    X(0b00000000) }

#define INSERT_BLANK2 INSERT_BLANK, INSERT_BLANK
#define INSERT_BLANK4 INSERT_BLANK2, INSERT_BLANK2
#define INSERT_BLANK8 INSERT_BLANK4, INSERT_BLANK4
#define INSERT_BLANK16 INSERT_BLANK8, INSERT_BLANK8
#define INSERT_BLANK32 INSERT_BLANK16, INSERT_BLANK16
#define INSERT_BLANK64 INSERT_BLANK32, INSERT_BLANK32
#define INSERT_BLANK128 INSERT_BLANK64, INSERT_BLANK64

/* ASCII characters */

static const uint8_t fonts_ascii[256][MEMLCD_FONT_DATA_BITHEIGHT] = {
    INSERT_BLANK32, /* Control character, 0..31 */
    { /* Space */
        X(0b00000000),
        X(0b00000000),
        X(0b00000000),
        X(0b00000000),
        X(0b00000000),
        X(0b00000000),
        X(0b00000000),
        X(0b00000000)
    }, { /* ! */
        X(0b00010000),
        X(0b00010000),
        X(0b00010000),
        X(0b00010000),
        X(0b00010000),
        X(0b00000000),
        X(0b00010000),
        X(0b00000000)
    }, { /* " */
        X(0b00101000),
        X(0b00101000),
        X(0b00000000),
        X(0b00000000),
        X(0b00000000),
        X(0b00000000),
        X(0b00000000),
        X(0b00000000)
    }, { /* # */
        X(0b00000000),
        X(0b00100100),
        X(0b01111110),
        X(0b00100100),
        X(0b00100100),
        X(0b01111110),
        X(0b00100100),
        X(0b00000000)
    }, { /* $ */
        X(0b00010000),
        X(0b01111110),
        X(0b10010000),
        X(0b01111100),
        X(0b00010010),
        X(0b01111100),
        X(0b00010000),
        X(0b00000000)
    }, { /* % */
        X(0b01100010),
        X(0b10010100),
        X(0b01101000),
        X(0b00010000),
        X(0b00101100),
        X(0b01010010),
        X(0b10001100),
        X(0b00000000)
    }, { /* & */
        X(0b00011000),
        X(0b00100000),
        X(0b01000000),
        X(0b00111000),
        X(0b00101010),
        X(0b01000100),
        X(0b00111010),
        X(0b00000000)
    }, { /* ' */
        X(0b00010000),
        X(0b00010000),
        X(0b00000000),
        X(0b00000000),
        X(0b00000000),
        X(0b00000000),
        X(0b00000000),
        X(0b00000000)
    }, { /* ( */
        X(0b00001000),
        X(0b00010000),
        X(0b00010000),
        X(0b00010000),
        X(0b00010000),
        X(0b00010000),
        X(0b00001000),
        X(0b00000000)
    }, { /* ) */
        X(0b00010000),
        X(0b00001000),
        X(0b00001000),
        X(0b00001000),
        X(0b00001000),
        X(0b00001000),
        X(0b00010000),
        X(0b00000000)
    }, { /* * */
        X(0b00010000),
        X(0b01010100),
        X(0b00101000),
        X(0b11010110),
        X(0b00101000),
        X(0b01010100),
        X(0b00010000),
        X(0b00000000),
    }, { /* + */
        X(0b00000000),
        X(0b00010000),
        X(0b00010000),
        X(0b11111110),
        X(0b00010000),
        X(0b00010000),
        X(0b00000000),
        X(0b00000000)
    }, { /* , */
        X(0b00000000),
        X(0b00000000),
        X(0b00000000),
        X(0b00000000),
        X(0b00000000),
        X(0b00011000),
        X(0b00100000),
        X(0b00000000)
    }, { /* - */
        X(0b00000000),
        X(0b00000000),
        X(0b00000000),
        X(0b01111110),
        X(0b00000000),
        X(0b00000000),
        X(0b00000000),
        X(0b00000000)
    }, { /* . */
        X(0b00000000),
        X(0b00000000),
        X(0b00000000),
        X(0b00000000),
        X(0b00000000),
        X(0b00000000),
        X(0b00011000),
        X(0b00000000)
    }, { /* / */
        X(0b00000010),
        X(0b00000100),
        X(0b00001000),
        X(0b00010000),
        X(0b00100000),
        X(0b01000000),
        X(0b10000000),
        X(0b00000000)
    }, { /* 0 */
        X(0b00111000),
        X(0b01000100),
        X(0b10100010),
        X(0b10010010),
        X(0b10001010),
        X(0b01000100),
        X(0b00111000),
        X(0b00000000)
    }, { /* 1 */
        X(0b00001000),
        X(0b00011000),
        X(0b00001000),
        X(0b00001000),
        X(0b00001000),
        X(0b00001000),
        X(0b00011100),
        X(0b00000000)
    }, { /* 2 */
        X(0b00111100),
        X(0b01000010),
        X(0b00000010),
        X(0b00000100),
        X(0b00011000),
        X(0b00100000),
        X(0b01111110),
        X(0b00000000)
    }, { /* 3 */
        X(0b00111100),
        X(0b01000010),
        X(0b00000010),
        X(0b00001100),
        X(0b00000010),
        X(0b01000010),
        X(0b00111100),
        X(0b00000000)
    }, { /* 4 */
        X(0b00001000),
        X(0b00011000),
        X(0b00101000),
        X(0b01001000),
        X(0b11111110),
        X(0b00001000),
        X(0b00001000),
        X(0b00000000)
    }, { /* 5 */
        X(0b01111110),
        X(0b01000000),
        X(0b01100000),
        X(0b00011100),
        X(0b00000010),
        X(0b01000010),
        X(0b00111100),
        X(0b00000000)
    }, { /* 6 */
        X(0b00111100),
        X(0b01000000),
        X(0b01000000),
        X(0b01011100),
        X(0b01100010),
        X(0b01000010),
        X(0b00111100),
        X(0b00000000)
    }, { /* 7 */
        X(0b01111110),
        X(0b00000010),
        X(0b00000100),
        X(0b00001000),
        X(0b00010000),
        X(0b00010000),
        X(0b00010000),
        X(0b00000000)
    }, { /* 8 */
        X(0b00111100),
        X(0b01000010),
        X(0b01000010),
        X(0b00111100),
        X(0b01000010),
        X(0b01000010),
        X(0b00111100),
        X(0b00000000)
    }, { /* 9 */
        X(0b00111100),
        X(0b01000010),
        X(0b01000010),
        X(0b00111110),
        X(0b00000010),
        X(0b01000100),
        X(0b00111000),
        X(0b00000000)
    }, { /* : */
        X(0b00000000),
        X(0b00000000),
        X(0b00011000),
        X(0b00000000),
        X(0b00000000),
        X(0b00000000),
        X(0b00011000),
        X(0b00000000)
    }, { /* ; */
        X(0b00000000),
        X(0b00000000),
        X(0b00011000),
        X(0b00000000),
        X(0b00000000),
        X(0b00011000),
        X(0b00100000),
        X(0b00000000)
    }, { /* < */
        X(0b00000000),
        X(0b00000000),
        X(0b00001100),
        X(0b00110000),
        X(0b11000000),
        X(0b00110000),
        X(0b00001100),
        X(0b00000000)
    }, { /* = */
        X(0b00000000),
        X(0b00000000),
        X(0b01111100),
        X(0b00000000),
        X(0b01111100),
        X(0b00000000),
        X(0b00000000),
        X(0b00000000)
    }, { /* > */
        X(0b00000000),
        X(0b00000000),
        X(0b01100000),
        X(0b00011000),
        X(0b00000110),
        X(0b00011000),
        X(0b01100000),
        X(0b00000000)
    }, { /* ? */
        X(0b00111100),
        X(0b01000010),
        X(0b00000110),
        X(0b00011000),
        X(0b00010000),
        X(0b00000000),
        X(0b00010000),
        X(0b00000000)
    }, { /* @ */
        X(0b00111000),
        X(0b01000100),
        X(0b10000010),
        X(0b10011010),
        X(0b10011110),
        X(0b01000000),
        X(0b00111100),
        X(0b00000000)
    }, { /* A */
        X(0b00011000),
        X(0b00100100),
        X(0b01000010),
        X(0b01000010),
        X(0b01111110),
        X(0b01000010),
        X(0b01000010),
        X(0b00000000)
    }, { /* B */
        X(0b01110000),
        X(0b01001000),
        X(0b01000100),
        X(0b01111100),
        X(0b01000010),
        X(0b01000010),
        X(0b01111100),
        X(0b00000000)
    }, { /* C */
        X(0b00111100),
        X(0b01000010),
        X(0b01000000),
        X(0b01000000),
        X(0b01000000),
        X(0b01000010),
        X(0b00111100),
        X(0b00000000)
    }, { /* D */
        X(0b01111000),
        X(0b01000100),
        X(0b01000010),
        X(0b01000010),
        X(0b01000010),
        X(0b01000100),
        X(0b01111000),
        X(0b00000000)
    }, { /* E */
        X(0b01111110),
        X(0b01000000),
        X(0b01000000),
        X(0b01111100),
        X(0b01000000),
        X(0b01000000),
        X(0b01111110),
        X(0b00000000)
    }, { /* F */
        X(0b01111110),
        X(0b01000000),
        X(0b01000000),
        X(0b01111100),
        X(0b01000000),
        X(0b01000000),
        X(0b01000000),
        X(0b00000000)
    }, { /* G */
        X(0b00111100),
        X(0b01000010),
        X(0b01000000),
        X(0b01000110),
        X(0b01000010),
        X(0b01000010),
        X(0b00111100),
        X(0b00000000)
    }, { /* H */
        X(0b01000010),
        X(0b01000010),
        X(0b01000010),
        X(0b01111110),
        X(0b01000010),
        X(0b01000010),
        X(0b01000010),
        X(0b00000000)
    }, { /* I */
        X(0b00111000),
        X(0b00010000),
        X(0b00010000),
        X(0b00010000),
        X(0b00010000),
        X(0b00010000),
        X(0b00111000),
        X(0b00000000)
    }, { /* J */
        X(0b00111000),
        X(0b00001000),
        X(0b00001000),
        X(0b00001000),
        X(0b00001000),
        X(0b01001000),
        X(0b00110000),
        X(0b00000000)
    }, { /* K */
        X(0b01000010),
        X(0b01000100),
        X(0b01001000),
        X(0b01110000),
        X(0b01001100),
        X(0b01000010),
        X(0b01000010),
        X(0b00000000)
    }, { /* L */
        X(0b01000000),
        X(0b01000000),
        X(0b01000000),
        X(0b01000000),
        X(0b01000000),
        X(0b01000000),
        X(0b01111110),
        X(0b00000000)
    }, { /* M */
        X(0b10000010),
        X(0b11000110),
        X(0b10101010),
        X(0b10010010),
        X(0b10000010),
        X(0b10000010),
        X(0b10000010),
        X(0b00000000)
    }, { /* N */
        X(0b10000010),
        X(0b11000010),
        X(0b10100010),
        X(0b10010010),
        X(0b10001010),
        X(0b10000110),
        X(0b10000010),
        X(0b00000000)
    }, { /* O */
        X(0b00111000),
        X(0b01000100),
        X(0b10000010),
        X(0b10000010),
        X(0b10000010),
        X(0b01000100),
        X(0b00111000),
        X(0b00000000)
    }, { /* P */
        X(0b01111100),
        X(0b01000010),
        X(0b01000010),
        X(0b01111100),
        X(0b01000000),
        X(0b01000000),
        X(0b01000000),
        X(0b00000000)
    }, { /* Q */
        X(0b00111000),
        X(0b01000100),
        X(0b10000010),
        X(0b10000010),
        X(0b10001010),
        X(0b01000100),
        X(0b00111010),
        X(0b00000000)
    }, { /* R */
        X(0b01111000),
        X(0b01000100),
        X(0b01000100),
        X(0b01111000),
        X(0b01010000),
        X(0b01001000),
        X(0b01000100),
        X(0b00000000)
    }, { /* S */
        X(0b00111100),
        X(0b01000010),
        X(0b01000000),
        X(0b00111100),
        X(0b00000010),
        X(0b01000010),
        X(0b00111100),
        X(0b00000000)
    }, { /* T */
        X(0b11111110),
        X(0b00010000),
        X(0b00010000),
        X(0b00010000),
        X(0b00010000),
        X(0b00010000),
        X(0b00010000),
        X(0b00000000)
    }, { /* U */
        X(0b10000010),
        X(0b10000010),
        X(0b10000010),
        X(0b10000010),
        X(0b10000010),
        X(0b01000100),
        X(0b00111000),
        X(0b00000000)
    }, { /* V */
        X(0b10000010),
        X(0b10000010),
        X(0b01000100),
        X(0b01000100),
        X(0b00101000),
        X(0b00101000),
        X(0b00010000),
        X(0b00000000)
    }, { /* W */
        X(0b10000010),
        X(0b10000010),
        X(0b01000100),
        X(0b01010100),
        X(0b01010100),
        X(0b01010100),
        X(0b00101000),
        X(0b00000000)
    }, { /* X */
        X(0b10000010),
        X(0b01000100),
        X(0b00101000),
        X(0b00010000),
        X(0b00101000),
        X(0b01000100),
        X(0b10000010),
        X(0b00000000)
    }, { /* Y */
        X(0b10000010),
        X(0b10000010),
        X(0b01000100),
        X(0b00101000),
        X(0b00010000),
        X(0b00010000),
        X(0b00010000),
        X(0b00000000)
    }, { /* Z */
        X(0b11111110),
        X(0b00000100),
        X(0b00001000),
        X(0b00010000),
        X(0b00100000),
        X(0b01000000),
        X(0b11111110),
        X(0b00000000)
    }, { /* [ */
        X(0b00111000),
        X(0b00100000),
        X(0b00100000),
        X(0b00100000),
        X(0b00100000),
        X(0b00100000),
        X(0b00111000),
        X(0b00000000)
    }, { /* \ */
        X(0b10000000),
        X(0b01000000),
        X(0b00100000),
        X(0b00010000),
        X(0b00001000),
        X(0b00000100),
        X(0b00000010),
        X(0b00000000)
    }, { /* ] */
        X(0b00111000),
        X(0b00001000),
        X(0b00001000),
        X(0b00001000),
        X(0b00001000),
        X(0b00001000),
        X(0b00111000),
        X(0b00000000)
    }, { /* ^ */
        X(0b00010000),
        X(0b00101000),
        X(0b01000100),
        X(0b00000000),
        X(0b00000000),
        X(0b00000000),
        X(0b00000000),
        X(0b00000000)
    }, { /* _ */
        X(0b00000000),
        X(0b00000000),
        X(0b00000000),
        X(0b00000000),
        X(0b00000000),
        X(0b00000000),
        X(0b11111110),
        X(0b00000000)
    }, { /* ` */
        X(0b00100000),
        X(0b00010000),
        X(0b00000000),
        X(0b00000000),
        X(0b00000000),
        X(0b00000000),
        X(0b00000000),
        X(0b00000000)
    }, { /* a */
        X(0b00000000),
        X(0b00000000),
        X(0b00111100),
        X(0b00000010),
        X(0b00111110),
        X(0b01000010),
        X(0b00111110),
        X(0b00000000)
    }, { /* b */
        X(0b01000000),
        X(0b01000000),
        X(0b01000000),
        X(0b01111000),
        X(0b01000100),
        X(0b01000100),
        X(0b01111000),
        X(0b00000000)
    }, { /* c */
        X(0b00000000),
        X(0b00000000),
        X(0b00111100),
        X(0b01000000),
        X(0b01000000),
        X(0b01000000),
        X(0b00111100),
        X(0b00000000)
    }, { /* d */
        X(0b00000100),
        X(0b00000100),
        X(0b00000100),
        X(0b00111100),
        X(0b01000100),
        X(0b01000100),
        X(0b00111100),
        X(0b00000000)
    }, { /* e */
        X(0b00000000),
        X(0b00000000),
        X(0b00111100),
        X(0b01000100),
        X(0b01111000),
        X(0b01000000),
        X(0b00111100),
        X(0b00000000)
    }, { /* f */
        X(0b00010000),
        X(0b00100000),
        X(0b01110000),
        X(0b00100000),
        X(0b00100000),
        X(0b00100000),
        X(0b00100000),
        X(0b00000000)
    }, { /* g */
        X(0b00000000),
        X(0b00000000),
        X(0b00111100),
        X(0b01000100),
        X(0b00111100),
        X(0b00000100),
        X(0b00111000),
        X(0b00000000)
    }, { /* h */
        X(0b00100000),
        X(0b00100000),
        X(0b00100000),
        X(0b00111000),
        X(0b00100100),
        X(0b00100100),
        X(0b00100100),
        X(0b00000000)
    }, { /* i */
        X(0b00010000),
        X(0b00000000),
        X(0b00110000),
        X(0b00010000),
        X(0b00010000),
        X(0b00010000),
        X(0b00111000),
        X(0b00000000)
    }, { /* j */
        X(0b00010000),
        X(0b00000000),
        X(0b01110000),
        X(0b00010000),
        X(0b00010000),
        X(0b00010000),
        X(0b01100000),
        X(0b00000000)
    }, { /* k */
        X(0b01000000),
        X(0b01000000),
        X(0b01001000),
        X(0b01010000),
        X(0b01100000),
        X(0b01010000),
        X(0b01001000),
        X(0b00000000)
    }, { /* l */
        X(0b00110000),
        X(0b00010000),
        X(0b00010000),
        X(0b00010000),
        X(0b00010000),
        X(0b00010000),
        X(0b00001000),
        X(0b00000000)
    }, { /* m */
        X(0b00000000),
        X(0b00000000),
        X(0b10101100),
        X(0b11010010),
        X(0b10010010),
        X(0b10010010),
        X(0b10010010),
        X(0b00000000)
    }, { /* n */
        X(0b00000000),
        X(0b00000000),
        X(0b01011100),
        X(0b01100010),
        X(0b01000010),
        X(0b01000010),
        X(0b01000010),
        X(0b00000000)
    }, { /* o */
        X(0b00000000),
        X(0b00000000),
        X(0b00111100),
        X(0b01000010),
        X(0b01000010),
        X(0b01000010),
        X(0b00111100),
        X(0b00000000)
    }, { /* p */
        X(0b00000000),
        X(0b00000000),
        X(0b01111100),
        X(0b01000010),
        X(0b01111100),
        X(0b01000000),
        X(0b01000000),
        X(0b00000000)
    }, { /* q */
        X(0b00000000),
        X(0b00000000),
        X(0b00111110),
        X(0b01000010),
        X(0b00111110),
        X(0b00000010),
        X(0b00000010),
        X(0b00000000)
    }, { /* r */
        X(0b00000000),
        X(0b00000000),
        X(0b01011100),
        X(0b01100000),
        X(0b01000000),
        X(0b01000000),
        X(0b01000000),
        X(0b00000000)
    }, { /* s */
        X(0b00000000),
        X(0b00000000),
        X(0b00111100),
        X(0b01000000),
        X(0b00111000),
        X(0b00000100),
        X(0b01111000),
        X(0b00000000)
    }, { /* t */
        X(0b00000000),
        X(0b00010000),
        X(0b00111000),
        X(0b00010000),
        X(0b00010000),
        X(0b00010000),
        X(0b00001000),
        X(0b00000000)
    }, { /* u */
        X(0b00000000),
        X(0b00000000),
        X(0b01000010),
        X(0b01000010),
        X(0b01000010),
        X(0b01000010),
        X(0b00111110),
        X(0b00000000)
    }, { /* v */
        X(0b00000000),
        X(0b00000000),
        X(0b01000100),
        X(0b01000100),
        X(0b01000100),
        X(0b00101000),
        X(0b00010000),
        X(0b00000000)
    }, { /* w */
        X(0b00000000),
        X(0b00000000),
        X(0b10000010),
        X(0b10010010),
        X(0b10010010),
        X(0b01010100),
        X(0b00101000),
        X(0b00000000)
    }, { /* x */
        X(0b00000000),
        X(0b00000000),
        X(0b01000100),
        X(0b00101000),
        X(0b00010000),
        X(0b00101000),
        X(0b01000100),
        X(0b00000000)
    }, { /* y */
        X(0b00000000),
        X(0b00000000),
        X(0b01000100),
        X(0b01000100),
        X(0b00111100),
        X(0b00000100),
        X(0b00111000),
        X(0b00000000)
    }, { /* z */
        X(0b00000000),
        X(0b00000000),
        X(0b01111100),
        X(0b00001000),
        X(0b00010000),
        X(0b00100000),
        X(0b01111100),
        X(0b00000000)
    }, { /* { */
        X(0b00001100),
        X(0b00010000),
        X(0b00010000),
        X(0b00100000),
        X(0b00010000),
        X(0b00010000),
        X(0b00001100),
        X(0b00000000)
    }, { /* | */
        X(0b00010000),
        X(0b00010000),
        X(0b00010000),
        X(0b00010000),
        X(0b00010000),
        X(0b00010000),
        X(0b00010000),
        X(0b00000000)
    }, { /* } */
        X(0b01100000),
        X(0b00010000),
        X(0b00010000),
        X(0b00001000),
        X(0b00010000),
        X(0b00010000),
        X(0b01100000),
        X(0b00000000)
    }, { /* ~ */
        X(0b00000000),
        X(0b00000000),
        X(0b00000000),
        X(0b00110010),
        X(0b01001100),
        X(0b00000000),
        X(0b00000000),
        X(0b00000000)
    },
    INSERT_BLANK128, /* Control character, 127..254 */
    {
        X(0b11111111),
        X(0b11111111),
        X(0b11111111),
        X(0b11111111),
        X(0b11111111),
        X(0b11111111),
        X(0b11111111),
        X(0b11111111)
    }
};

#undef X

/******************************************************************************
 * Private Functions
 ******************************************************************************/

/******************************************************************************
 * Name: char_to_fontface
 *
 * Description:
 *   Get font-face bitmap for character.
 *
 * Input Parameters:
 *   c  - Character value
 *
 * Returned Value:
 *   Font-face bitmap for character
 *
 ******************************************************************************/

static inline const uint8_t *char_to_fontface(char c)
{
  return fonts_ascii[c & 0xff];
}

#if MEMLCD_FONT_BITWIDTH == 8

/******************************************************************************
 * Name: char_to_pixels
 *
 * Description:
 *   Get font pixels for character.
 *
 * Input Parameters:
 *   c    - Character value
 *   xpos - X coordinate for font bitmap
 *
 * Returned Value:
 *   Font-face pixels for character
 *
 ******************************************************************************/

static inline const uint8_t char_to_pixels(char c, unsigned int xpos)
{
  const uint8_t * face = char_to_fontface(c);

  return face[(xpos * MEMLCD_FONT_DATA_BITHEIGHT) / MEMLCD_FONT_BITHEIGHT];
}

/******************************************************************************
 * Name: char_line_to_pixels
 *
 * Description:
 *   Get font pixels for character line.
 *
 * Input Parameters:
 *   pxbuf - Target pixel buffer
 *   consline - Source character line buffer
 *   lineline - length of consline buffer
 *   xpos - X coordinate for font bitmap
 *
 * Returned Value:
 *   None.
 *
 ******************************************************************************/

static inline void char_line_to_pixels(uint8_t *pxbuf, const int8_t *consline,
                                       unsigned int linelen, unsigned int xpos)
{
  while (linelen--)
    {
      *pxbuf++ = char_to_pixels(*consline++, xpos);
    }
}

#elif MEMLCD_FONT_BITWIDTH == 16

/******************************************************************************
 * Name: char_to_pixels
 *
 * Description:
 *   Get font pixels for character.
 *
 * Input Parameters:
 *   c    - Character value
 *   xpos - X coordinate for font bitmap
 *
 * Returned Value:
 *   Font-face pixels for character
 *
 ******************************************************************************/

static inline const uint16_t char_to_pixels(char c, unsigned int xpos)
{
  const uint8_t * face = char_to_fontface(c);
  uint8_t eightbits =
      face[(xpos * MEMLCD_FONT_DATA_BITHEIGHT) / MEMLCD_FONT_BITHEIGHT];
  uint16_t sixteenbits = eightbits;

  sixteenbits = (sixteenbits | (sixteenbits << 4)) & 0x0f0fU;
  sixteenbits = (sixteenbits | (sixteenbits << 2)) & 0x3333U;
  sixteenbits = (sixteenbits | (sixteenbits << 1)) & 0x5555U;

  return sixteenbits | (sixteenbits << 1);
}

/******************************************************************************
 * Name: char_line_to_pixels
 *
 * Description:
 *   Get font pixels for character line.
 *
 * Input Parameters:
 *   pxbuf - Target pixel buffer
 *   consline - Source character line buffer
 *   lineline - length of consline buffer
 *   xpos - X coordinate for font bitmap
 *
 * Returned Value:
 *   None.
 *
 ******************************************************************************/

static inline void char_line_to_pixels(uint8_t *pxbuf, const int8_t *consline,
                                       unsigned int linelen, unsigned int xpos)
{
  uint16_t *pxbuf16 = (void *)pxbuf;

  while (linelen--)
    {
      *pxbuf16++ = char_to_pixels(*consline++, xpos);
    }
}

#endif

#endif

