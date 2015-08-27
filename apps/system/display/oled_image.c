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

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <unistd.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nxfonts.h>

#include "oled_image.h"


#ifdef CONFIG_THINGSEE_DISPLAY_TRACES
#  define lcd_dbg(x, ...)    dbg(x, ##__VA_ARGS__)
#  define lcd_lldbg(x, ...)  lldbg(x, ##__VA_ARGS__)
#else
#  define lcd_dbg(x, ...)
#  define lcd_lldbg(x, ...)
#endif


#define OLED_IMG_SCREEN_WIDTH            128
#define OLED_IMG_SCREEN_HEIGHT           64


typedef struct oled_image_s {
    NXWINDOW hwnd;
} oled_image_t;

static oled_image_t g_img;


/****************************************************************************
 * Name: oled_image_crop_img
 *
 * Description:
 *  Crops pxs from image
 *
 * Input Parameters:
 *  pos_x    - x coordinate of the high left corner
 *  pos_y    - y coordinate of the high left corner
 *  img      - image canvas
 *  crop_pxs - pixels amount to be cropped from the image
 *
 * Returned Values:
 *  Returns 0 on SUCCESS. -1 if ERROR occurred
 *
 ****************************************************************************/

int oled_image_crop_img(uint8_t pos_x, uint8_t pos_y, oled_image_canvas_t *img, oled_image_crop_pxs_t *crop_pxs)
{
    const uint8_t byte_length = 8;
    int ret = OK;
    FAR struct nxgl_rect_s dest;
    FAR void *src[CONFIG_NX_NPLANES];
    FAR struct nxgl_point_s pos;
    uint32_t i, j, pxs;
    uint8_t stride;
    uint16_t left_pad,top_pad;
    uint8_t img_width_crop, img_height_crop;
    uint8_t pic_line[16] = { 0 };
    /* The only way to draw line from the image
     * is addressing to this line byte by byte.
     * Thus we need variables to save pixels pads
     * from the left side of the image */
    uint8_t left_pxs_offset;

    /* check if crop_pxs structure's data is valid */

    if (crop_pxs->bottom_pxs >= img->height || crop_pxs->top_pxs >= img->height
        || crop_pxs->left_pxs >= img->width || crop_pxs->right_pxs >= img->width
        || (crop_pxs->left_pxs + crop_pxs->right_pxs) >= img->width
        || (crop_pxs->top_pxs + crop_pxs->bottom_pxs) >= img->height) {
        lcd_lldbg("Cannot proceed. Amount of pixels to be cropped is greater or equal to the"
                  " image height or width\n");
        return ERROR;
    }

    /* check, will image fit on the screen after crop */

    if (pos_x >= OLED_IMG_SCREEN_WIDTH || pos_y >= OLED_IMG_SCREEN_HEIGHT
        || (pos_x + img->width - crop_pxs->left_pxs - crop_pxs->right_pxs) > OLED_IMG_SCREEN_WIDTH
        || (pos_y + img->height - crop_pxs->top_pxs - crop_pxs->bottom_pxs) > OLED_IMG_SCREEN_HEIGHT) {
        lcd_dbg("Wrong coordinates to start draw image"
                "or image too big\n");
        return ERROR;
    }

    left_pad = crop_pxs->left_pxs / byte_length;
    left_pxs_offset = crop_pxs->left_pxs % byte_length;
    top_pad = crop_pxs->top_pxs;
    /* New width and height calculation */
    img_width_crop = img->width - crop_pxs->left_pxs - crop_pxs->right_pxs;
    img_height_crop = img->height - crop_pxs->top_pxs - crop_pxs->bottom_pxs;

    pos.x = pos_x - left_pxs_offset;
    pos.y = pos_y;

    dest.pt1.x = pos.x;
    dest.pt2.x = pos.x + img_width_crop - 1;

    stride = img->width / 8 + (!(img->width % 8) ? 0 : 1);

    for (i = 0, j = left_pad + stride * top_pad; i < img_height_crop; i++, j += stride) {
        dest.pt1.y = pos.y;
        dest.pt2.y = pos.y;

        memcpy(pic_line, (const uint8_t *)(img->bitmap + j), stride - left_pad);
        for (pxs = 0; pxs < left_pxs_offset; pxs++) {
            DEBUGASSERT(pxs <= byte_length);
            if (!CONFIG_THINGSEE_DISPLAY_BKGND_COLOR) {
                *(pic_line) &= ~(1 << (byte_length - pxs));
            } else {
                *(pic_line) |= (1 << (byte_length - pxs));
            }
        }

        src[0] = (FAR void *) pic_line;

        ret = nx_bitmap(g_img.hwnd, &dest, (FAR void *)src, &pos, img_width_crop);
        if (ret < 0) {
            lcd_dbg("nx_bitmapwindow failed: %d\n", errno);
            return ERROR;
        }

        pos.y++;
    }

    return ret;
}

/****************************************************************************
 * Name: oled_draw_img
 *
 * Description:
 *  Initializes image module
 *
 * Input Parameters:
 *  pos_x  - x coordinate of the high left corner
 *  pos_y  - y coordinate of the high left corner
 *  img    - image canvas
 *
 * Returned Values:
 *  Returns 0 on SUCCESS. -1 if ERROR occurred
 *
 ****************************************************************************/

int oled_image_draw_img(uint8_t pos_x, uint8_t pos_y, const oled_image_canvas_t  * const img)
{
    int ret = OK;
    FAR struct nxgl_rect_s dest;
    FAR void *src[CONFIG_NX_NPLANES];
    FAR struct nxgl_point_s pos;
    uint32_t i, j;
    uint8_t stride;

    if (pos_x >= OLED_IMG_SCREEN_WIDTH || pos_y >= OLED_IMG_SCREEN_HEIGHT
        || (pos_x + img->width) > OLED_IMG_SCREEN_WIDTH
        || (pos_y + img->height) > OLED_IMG_SCREEN_HEIGHT) {
        lcd_dbg("Wrong coordinates to start draw image"
                "or image too big\n");
        return ERROR;
    }

    pos.x = pos_x;
    pos.y = pos_y;

    dest.pt1.x = pos.x;
    dest.pt2.x = pos.x + img->width - 1;

    stride = img->width / 8 + (!(img->width % 8) ? 0 : 1);

    for (i = 0, j = 0; i < img->height; i++, j += stride) {
        dest.pt1.y = pos.y;
        dest.pt2.y = pos.y;

        src[0] = (FAR void *) (img->bitmap + j);

        ret = nx_bitmap(g_img.hwnd, &dest, (FAR void *)src, &pos, img->width);
        if (ret < 0) {
            lcd_dbg("nx_bitmapwindow failed: %d\n", errno);
            return ERROR;
        }

        pos.y++;
    }

    return ret;
}

/****************************************************************************
 * Name: oled_image_fill_rectangle
 *
 * Description:
 *  Fills defined rectangle region with user defined color
 *
 * Input Parameters:
 *  x  - x coordinate of the high left corner
 *  y  - y coordinate of the high left corner
 *  width  - rectangle width
 *  height - rectangle height
 *  color  - color to fill rectangle with
 *
 * Returned Values:
 *  Returns 0 on SUCCESS. -1 if ERROR occurred
 *
 ****************************************************************************/

int oled_image_fill_rectangle(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t color)
{
    int ret = OK;
    FAR struct nxgl_rect_s rect;

    if (x >= OLED_IMG_SCREEN_WIDTH || y >= OLED_IMG_SCREEN_HEIGHT
        || (x + width) > OLED_IMG_SCREEN_WIDTH
        || (y + height) > OLED_IMG_SCREEN_HEIGHT) {
        lcd_dbg("Wrong coordinates to start draw image"
                "or image too big\n");
        return ERROR;
    }

    rect.pt1.x = x;
    rect.pt1.y = y;
    rect.pt2.x = x + width - 1;
    rect.pt2.y = y + height - 1;

    ret = nx_fill(g_img.hwnd, &rect, &color);
    if (ret < 0) {
        ret = ERROR;
        lcd_dbg("Cannot fill rectangle\n");
    }

    return ret;
}

/****************************************************************************
 * Name: oled_image_init
 *
 * Description:
 *  Initializes image module
 *
 * Input Parameters:
 *  hwnd - background window handler
 *
 * Returned Values:
 *
 ****************************************************************************/

void oled_image_init(NXWINDOW hwnd)
{
    g_img.hwnd = hwnd;
    //TO DO: Add more information into structure
}
