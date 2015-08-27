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

#ifndef OLED_IMAGE_H_
#define OLED_IMAGE_H_

/* Structure to keep image information in */

typedef struct oled_image_canvas_s {
    uint8_t width;                    /* Image width in pixels */
    uint8_t height;                   /* Image height in pixels */
    const uint8_t *bitmap;            /* Image data */
} oled_image_canvas_t;

/* This structure tells, how many pixels must be removed from the image */

typedef struct oled_image_crop_pxs_s {
    uint16_t left_pxs;
    uint16_t right_pxs;
    uint16_t top_pxs;
    uint16_t bottom_pxs;
} oled_image_crop_pxs_t;

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

int oled_image_crop_img(uint8_t pos_x, uint8_t pos_y, oled_image_canvas_t *img, oled_image_crop_pxs_t *crop_pxs);

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

int oled_image_draw_img(uint8_t pos_x, uint8_t pos_y, const oled_image_canvas_t * const img);

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

int oled_image_fill_rectangle(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t color);

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

void oled_image_init(NXWINDOW hwnd);

#endif /* OLED_IMAGE_H_ */
