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
#include <stdlib.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nxfonts.h>

#include "oled_inverter.h"




#ifdef CONFIG_THINGSEE_DISPLAY_TRACES
#  define lcd_dbg(x, ...)    dbg(x, ##__VA_ARGS__)
#  define lcd_lldbg(x, ...)  lldbg(x, ##__VA_ARGS__)
#else
#  define lcd_dbg(x, ...)
#  define lcd_lldbg(x, ...)
#endif


#define OLED_IMG_SCREEN_WIDTH            128
#define OLED_IMG_SCREEN_HEIGHT           64

typedef struct oled_inverter_s {
    NXWINDOW hwnd;
} oled_inverter_t;

static oled_inverter_t g_inv;




/****************************************************************************
 * Name: oled_invertor_do_invert
 *
 * Description:
 *  Inverts colors in user defined zone on the screen
 *
 * Input Parameters:
 *  pos_x  - x coordinate of the high left corner
 *  pos_y  - y coordinate of the high left corner
 *  width  - zone width
 *  height - zone height
 *
 * Returned Values:
 *     Returns 0 on SUCCESS. -1 if ERROR occurred
 *
 ****************************************************************************/

int oled_invertor_do_invert(uint8_t pos_x, uint8_t pos_y, uint8_t width, uint8_t height)
{
    int ret = OK;
    FAR uint8_t *dest;
    FAR struct nxgl_rect_s copy_area;
    FAR struct nxgl_point_s pos;
    uint32_t dstride;
    size_t region_size;
    uint16_t i, j;
    FAR void *src[CONFIG_NX_NPLANES];

    pos.x = pos_x;
    pos.y = pos_y;

    if (pos_x >= OLED_IMG_SCREEN_WIDTH || pos_y >= OLED_IMG_SCREEN_HEIGHT
        || (pos_x + width) > OLED_IMG_SCREEN_WIDTH
        || (pos_y + height) > OLED_IMG_SCREEN_HEIGHT) {
        lcd_dbg("Wrong coordinates to start draw image"
                "or image too big\n");
        return ERROR;
    }

    dstride = width / 8 + ((width % 8) ? 1 : 0);
    region_size = dstride * height;

    dest = malloc(region_size);

    if (!dest) {
        lcd_lldbg("Cannot allocate memory for destination rectangle: %d\n", errno);
        return ERROR;
    }

    copy_area.pt1.x = pos_x;
    copy_area.pt1.y = pos_y;
    copy_area.pt2.x = pos_x + width - 1;
    copy_area.pt2.y = pos_y + height - 1;

    nx_getrectangle(g_inv.hwnd, &copy_area, CONFIG_THINGSEE_DISPLAY_BKGND_COLOR, dest, dstride);

    /* Let's invert colors in copied area */
    for (i = 0; i < region_size; i++) {
        *(dest + i) ^= 0xFF;
    }

    /* Let's re-draw user selected region */
    for (i = 0, j = 0; i < height; i++, j += dstride) {
        copy_area.pt1.y = pos.y;
        copy_area.pt2.y = pos.y;

        src[0] = (FAR void *) (dest + j);

        ret = nx_bitmap(g_inv.hwnd, &copy_area, (FAR void *)src, &pos, width);
        if (ret < 0) {
            lcd_dbg("nx_bitmapwindow failed: %d\n", errno);
            free(dest);
            return ERROR;
        }

        pos.y++;
    }

    free(dest);

    return ret;
}

/****************************************************************************
 * Name: oled_inverter_init
 *
 * Description:
 *  Initializes image module
 *
 * Input Parameters:
 *  hwnd - pointer to background window
 *
 * Returned Values:
 *     Returns 0 on SUCCESS. -1 if ERROR occurred
 *
 ****************************************************************************/

void oled_inverter_init(NXWINDOW hwnd)
{
    g_inv.hwnd = hwnd;
    //TO DO: ADD more information into structure
}
