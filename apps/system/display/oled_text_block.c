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
#include <string.h>
#include <stdlib.h>
#include <debug.h>


#include "oled_text_block.h"

/* TO DO: Add more functionality here */



/****************************************************************************
 * Name: oled_tblock_init
 *
 * Description:
 *  Allocates memory and initializes text block. By default
 *  the text is aligned to the left side of the text block
 *
 * Input Parameters:
 *  hndr - handler to text block
 *
 * Returned value:
 * 	pointer to new text block
 *
 ****************************************************************************/

oled_tblock_t *oled_tblock_init(uint8_t x, uint8_t y, uint8_t width, uint8_t height)
{
    oled_tblock_t *tblock = NULL;

    tblock = malloc(sizeof(oled_tblock_t));
    if (tblock) {
        tblock->x = x;
        tblock->y = y;
        tblock->width = width;
        tblock->height = height;
#if CONFIG_THINGSEE_DISPLAY_ALIGNMENT
        tblock->multiline = false;
        tblock->linespace = 0;
        tblock->alignment = 0;
        tblock->vert_alignment = 3;
#endif
    }

    return tblock;
}

/****************************************************************************
 * Name: oled_tblock_deinit
 *
 * Description:
 *  Frees memory used by text block
 *
 * Input Parameters:
 *  hndr - handler to text block
 *
 * Returned value:
 *
 ****************************************************************************/

void oled_tblock_deinit(oled_tblock_t *hndr)
{
    free(hndr);
}
