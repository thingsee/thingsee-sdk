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
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>
#include "qr_simple_render.h"

#ifdef CONFIG_THINGSEE_DISPLAY_TRACES
#  define lcd_dbg(x, ...)    dbg(x, ##__VA_ARGS__)
#  define lcd_lldbg(x, ...)  lldbg(x, ##__VA_ARGS__)
#else
#  define lcd_dbg(x, ...)
#  define lcd_lldbg(x, ...)
#endif
#define QR_SIMPLE_SIDE_HEIGHT       25
#define QR_SIMPLE_SIDE_WIDTH        25
#define QR_SIMPLE_DISP_HEIGHT       63
#define QR_SIMPLE_DISP_WIDTH        127
#define QR_SIMPLE_BYTES_PER_LINE    7
#define QR_BCKGRND_COLOR            0xFF        /* Always white */

static void qr_simple_prepare_bkg_matrix(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH], NXWINDOW hwnd)
{
    int ret = OK;
    FAR struct nxgl_rect_s clear_rect;
    uint8_t color = 0;
    uint8_t row, col;
    color = QR_BCKGRND_COLOR;
    clear_rect.pt1.x = 0;
    clear_rect.pt1.y = 0;
    clear_rect.pt2.x = QR_SIMPLE_DISP_WIDTH;
    clear_rect.pt2.y = QR_SIMPLE_DISP_HEIGHT;
    ret = nx_fill(hwnd, &clear_rect, &color);
    color = 0x0;
    clear_rect.pt1.x = 2;
    clear_rect.pt1.y = 2;
    clear_rect.pt2.x = QR_SIMPLE_DISP_WIDTH-2;
    clear_rect.pt2.y = QR_SIMPLE_DISP_HEIGHT-2;
    ret = nx_fill(hwnd, &clear_rect, &color);
    color = QR_BCKGRND_COLOR;
    clear_rect.pt1.x = 4;
    clear_rect.pt1.y = 4;
    clear_rect.pt2.x = QR_SIMPLE_DISP_WIDTH-4;
    clear_rect.pt2.y = QR_SIMPLE_DISP_HEIGHT-4;
    ret = nx_fill(hwnd, &clear_rect, &color);
    if (ret < 0) {
        lcd_dbg("nx_bitmapwindow failed: %d\n", errno);
        return;
    }
    /* Let's invert all bits. Since one means white and zero means black */
    for (row = 0; row < QR_SIMPLE_SIDE_HEIGHT; row++) {
        for (col = 0; col < QR_SIMPLE_SIDE_WIDTH; col++) {
            *(*(matrix + row) + col) ^= 0x1;
        }
    }
}

static void qr_simple_compose_byte(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH], uint8_t row, uint8_t *bytes)
{
    int8_t i = 0;
    uint8_t col, byte_ind;
    for (col = 0, byte_ind = 0; byte_ind < QR_SIMPLE_BYTES_PER_LINE; byte_ind++) {
        if (col < (QR_SIMPLE_SIDE_WIDTH - 1)) {
            for (i = 7; i >= 0; i -= 2, col++) {
                *(bytes + byte_ind) |= *(*(matrix + row) + col) << i;
                *(bytes + byte_ind) |= *(*(matrix + row) + col) << (i - 1);
            }
        } else {
            *(bytes + byte_ind) = *(*(matrix + row) + col) << 7;
            *(bytes + byte_ind) |= *(*(matrix + row) + col) << 6;
        }
    }
}

static void qr_simple_draw(uint8_t (*matrix)[], NXWINDOW hwnd)
{
    int ret = OK;
    FAR struct nxgl_point_s pos;
    FAR struct nxgl_rect_s dest;
    FAR const void *src[CONFIG_NX_NPLANES];
    uint8_t row, render_lines;
    uint8_t bytes[QR_SIMPLE_BYTES_PER_LINE] = { 0 };
    pos.x = 39;
    pos.y = 7;
    for (row = 0, render_lines = 0; render_lines < 2 * QR_SIMPLE_SIDE_HEIGHT; row++, render_lines += 2) {
        pos.y = 7 + render_lines;
        dest.pt1.y = pos.y;
        dest.pt2.y = pos.y;
        qr_simple_compose_byte(matrix, row, bytes);
        dest.pt1.x = pos.x;
        dest.pt2.x = pos.x + 2 * QR_SIMPLE_SIDE_WIDTH - 1;
        src[0] = (FAR const void *)bytes;
        /* Render first line bytes */
        ret = nx_bitmap(hwnd, &dest, src, &pos, 2 * QR_SIMPLE_SIDE_WIDTH);
        if (ret < 0) {
            lcd_dbg("nx_bitmapwindow failed: %d\n", errno);
            return;
        }
        /* Render second line bytes */
        pos.y++;
        dest.pt1.y = pos.y;
        dest.pt2.y = pos.y;
        ret = nx_bitmap(hwnd, &dest, src, &pos, 2 * QR_SIMPLE_SIDE_WIDTH);
        if (ret < 0) {
            lcd_dbg("nx_bitmapwindow failed: %d\n", errno);
            return;
        }
        /* Clean memory */
        memset(bytes, 0, QR_SIMPLE_BYTES_PER_LINE);
    }
}

void qr_simple_render(uint8_t (*matrix)[], NXWINDOW hwnd)
{
    uint8_t bckgrnd_color = 0;
    /* To make things clear */
    bckgrnd_color = CONFIG_THINGSEE_DISPLAY_BKGND_COLOR;
    if (!bckgrnd_color) {
        qr_simple_prepare_bkg_matrix(matrix, hwnd);
    }
    lldbg("Starting drawing\n");
    /* Try to render code on screen */
    qr_simple_draw(matrix, hwnd);
    lldbg("Result see for yourself\n");
}
