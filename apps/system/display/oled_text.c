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
#include <stdlib.h>
#include <stdarg.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nxfonts.h>

#include "oled_display.h"


/* The max. nuber of characters is 147 with 5x8 monospace font, assuming
 * there at least one pixel between each letter both horizontally and vertically */
#define OLED_DBG_SPC_BETWEEN_ROWS      1
#define OLED_DBG_MAX_ROW_CHS           24
#define OLED_DBG_BUF_LENGTH            169    /* Text + '\0' */
#define OLED_DBG_FONT                  FONTID_MONO5X8
#define OLED_DBG_SPC_PER_TAB           4
#define OLED_DBG_MAX_ROWS              7

#define OLED_WORK_AREA_X               127
#define OLED_WORK_AREA_Y               63



#define OLED_SPCPERTAB_NORMAL_TXT      4
#define OLED_NORMAL_TXT_BUF_SIZE       128    /* Well, anyway the max number of chars per row is 25. But we will keep buffer that big if someone will demand more  */

#ifdef CONFIG_THINGSEE_DISPLAY_TRACES
#  define lcd_dbg(x, ...)    dbg(x, ##__VA_ARGS__)
#  define lcd_lldbg(x, ...)  lldbg(x, ##__VA_ARGS__)
#else
#  define lcd_dbg(x, ...)
#  define lcd_lldbg(x, ...)
#endif

/***************************************************************************
 * Private types
 ***************************************************************************/

typedef struct oled_text_s {
    NXWINDOW hwnd;                         /* Used everywhere */
    FAR struct nxgl_point_s cursor;        /* Used only in debug prints with mono 5x8 */
    NXHANDLE dbg_hfont;                    /* Debug font handle */
    uint8_t *dbg_glyph;                    /* Used only in debug prints with mono 5x8 */
    size_t dbg_glyph_size;                 /* Debug glyph size */
    uint8_t chs_count;                     /* Characters in current row. Max is 21 characters. Used only with debug prints */
    uint8_t rows_counter;                 /* Row counter. We need to count only first 7 rows */
} oled_text_t;

/***************************************************************************
 * Private variables
 ***************************************************************************/

static uint8_t g_oled_dbg_buffer[OLED_DBG_BUF_LENGTH];
static uint8_t g_oled_normal_txt_buffer[OLED_NORMAL_TXT_BUF_SIZE];
static oled_text_t g_oled_dev;

/***************************************************************************
 * Private functions
 ***************************************************************************/
#if CONFIG_THINGSEE_DISPLAY_ALIGNMENT
static int oled_hor_align_render_text(oled_tblock_t *hndr, enum nx_fontid_e font, const char *str);
#endif

/****************************************************************************
 * Name: oled_fill_whitespace
 *
 * Description:
 *  Fills with empty rectangles whitespaces and
 *  garbage left from the previous rendering
 *
 * Input Parameters:
 *  cur_width         - current width used in block
 *  pos               - cursor position to reduce amount of calculation
 *  hndr              - pointer to text block
 *  white_space_width - whitespace width
 *
 * Returned Values:
 *  Characters number written to the screen. On ERROR returns -1
 *
 ****************************************************************************/

static void oled_fill_whitespace(uint8_t cur_width, oled_tblock_t *hndr, FAR const struct nxgl_point_s *pos, uint8_t white_space_width)
{
    int ret = OK;
    FAR struct nxgl_rect_s clear_rect;
    uint8_t color = 0;

    clear_rect.pt1.x = pos->x;
    clear_rect.pt1.y = hndr->y;
    if (!white_space_width) {
        clear_rect.pt2.x = hndr->x + hndr->width;
    } else {
        clear_rect.pt2.x = hndr->x + cur_width + white_space_width;
    }
    clear_rect.pt2.y = hndr->y + hndr->height;

    if (cur_width >= hndr->width) {
        return;
    }

    color = CONFIG_THINGSEE_DISPLAY_BKGND_COLOR;

    ret = nx_fill((NXWINDOW)g_oled_dev.hwnd, &clear_rect, &color);
    if (ret < 0) {
        lcd_dbg("nx_bitmapwindow failed: %d\n", errno);
        return;
    }
}

/****************************************************************************
 * Name: oled_normal_render
 *
 * Description:
 *  Normal rendering
 *
 * Input Parameters:
 *  font - font to be used. The font must be activated in the menuconfig before use
 *  str  - pointer to the string to be drawn
 *  hndr - pointer to text block
 *
 * Returned Values:
 *  Characters number written to the screen. On ERROR returns -1
 *
 ****************************************************************************/

static int oled_normal_render(enum nx_fontid_e font, const char *str, oled_tblock_t *hndr)
{
    int ret = OK;
    FAR const struct nx_fontbitmap_s *fbm;
    FAR struct nxgl_point_s pos;
    FAR struct nxgl_rect_s dest;
    FAR const struct nx_font_s *fontset;
    int count = 0;               /* Number of characters written */
    NXHANDLE font_handle;
    uint8_t *glyph;
    size_t glyph_size;

    /* Works with only NX-plane */

    FAR const void *src[CONFIG_NX_NPLANES];
    uint8_t fwidth = 0;
    uint8_t fheight = 0;
    uint8_t stride = 1;
    uint8_t width = 0;

    font_handle = nxf_getfonthandle(font);
    fontset = nxf_getfontset(font_handle);

    /* First of all, let's check, does character is small enough to be drawn on the screen */

    if (fontset->mxheight >= OLED_WORK_AREA_Y
        || fontset->mxwidth >= OLED_WORK_AREA_X) {
        lcd_lldbg("Font is too big\n");
        return ERROR;
    }

    if (hndr->height < fontset->mxheight) {
        return 0;
    }

    glyph_size = fontset->mxwidth * fontset->mxheight;
    glyph = (FAR uint8_t *)malloc(glyph_size);
    if (!glyph) {
        lcd_lldbg("Cannot allocate memory for glyph: %d\n", errno);
        return ERROR;
    }

    pos.x = hndr->x;
    /* New data always will be printed on next row */
    pos.y = hndr->y;

    for (; *str; str++) {
        lcd_dbg("Character: %c\n", *str);
        /* Get the bitmap font for this ASCII code */

        fbm = nxf_getbitmap(font_handle, *str);

        if (fbm) {
            fwidth = fbm->metric.width + fbm->metric.xoffset;
            fheight = fbm->metric.height + fbm->metric.yoffset;
            stride = (fwidth + 7) >> 3;
            memset(glyph, CONFIG_THINGSEE_DISPLAY_BKGND_COLOR, glyph_size);

            (void)nxf_convert_1bpp((FAR nxgl_mxpixel_t*)glyph, fheight, fwidth,
                                   stride, fbm, CONFIG_THINGSEE_DISPLAY_FOREGND_COLOR);

            /* Describe the destination of the font with a rectangle */

            dest.pt1.x = pos.x;
            dest.pt1.y = pos.y;

            dest.pt2.x = pos.x + fwidth - 1;
            dest.pt2.y = pos.y + hndr->height;

            src[0] = (FAR const void *)glyph;

            /* Increment width, to check if the next character will fit into the text block */

            width += fwidth;

            if (width >= hndr->width) {
                free(glyph);
                return count;
            }

            ret = nx_bitmap((NXWINDOW)g_oled_dev.hwnd, &dest, src, &pos, stride);
            if (ret < 0) {
                lcd_dbg("nx_bitmapwindow failed: %d\n", errno);
                free(glyph);
                return ERROR;
            }

            pos.x += fwidth;
        } else {
            if (width >= hndr->width) {
                free(glyph);
                return count;
            }

            if (*str == '\t') {
                /* In current version you can use tabs only on one row.
                 * May be later spaces will be moved to the next row */
                oled_fill_whitespace(width, hndr, &pos, OLED_SPCPERTAB_NORMAL_TXT * fontset->spwidth);
                pos.x += OLED_SPCPERTAB_NORMAL_TXT * fontset->spwidth;
                width +=  OLED_SPCPERTAB_NORMAL_TXT * fontset->spwidth;
            } else {
                oled_fill_whitespace(width, hndr, &pos, fontset->spwidth);
                pos.x += fontset->spwidth;
                width += fontset->spwidth;
            }
        }

        /* Characters counter that will be returned */
        count++;
    }

    oled_fill_whitespace(width, hndr, &pos, 0);

    free(glyph);

    return count;
}

/****************************************************************************
 * Name: oled_do_scroll_up
 *
 * Description:
 *  Scrolls text upwards
 *
 * Input Parameters:
 *  height - row's height: font's max height + space between rows
 *
 * Returned Values:
 *  On success returns OK. On ERROR returns -1
 *
 ****************************************************************************/

static int oled_do_scroll_up(uint8_t height)
{
    FAR struct nxgl_rect_s screen_area;
    FAR struct nxgl_rect_s clear_area;
    FAR struct nxgl_point_s offset;
    /* Color to fill rectangle with */
    nxgl_mxpixel_t color[1] = { CONFIG_THINGSEE_DISPLAY_BKGND_COLOR };
    int ret = OK;

    offset.x = 0;
    offset.y = (-1) * height;        /* -1 indicates that text will be moved upwards */
    screen_area.pt1.x = 0;
    screen_area.pt1.y = height;
    screen_area.pt2.x = OLED_WORK_AREA_X;
    screen_area.pt2.y = OLED_WORK_AREA_Y;

    clear_area.pt1.x = 0;
    clear_area.pt1.y = 0;
    clear_area.pt2.x = OLED_WORK_AREA_X;
    clear_area.pt2.y = height;

    ret = nx_fill((NXWINDOW)g_oled_dev.hwnd, &clear_area, color);

    ret = nx_move(g_oled_dev.hwnd, &screen_area, &offset);

    if (ret < 0) {
        lcd_lldbg("Cannot move screen area: %d\n", errno);
        return ERROR;
    }

    clear_area.pt1.x = 0;
    clear_area.pt1.y = OLED_WORK_AREA_Y - height;
    clear_area.pt2.x = OLED_WORK_AREA_X;
    clear_area.pt2.y = OLED_WORK_AREA_Y;

    ret = nx_fill((NXWINDOW)g_oled_dev.hwnd, &clear_area, color);

    if (ret < 0) {
        lcd_lldbg("Cannot fill clear area: %d\n", errno);
        return ERROR;
    }

    return OK;
}

/****************************************************************************
 * Name: oled_move_cursor_new_row
 *
 * Description:
 *  Moves cursor to next line
 *
 * Input Parameters:
 *  height - row's height: font's max height + space between rows
 *
 * Returned Values:
 *
 ****************************************************************************/

static void oled_move_cursor2new_row(uint8_t height)
{
    g_oled_dev.cursor.x = 0;
    g_oled_dev.chs_count = 0;

    if (g_oled_dev.rows_counter < OLED_DBG_MAX_ROWS) {
        g_oled_dev.rows_counter++;
        /* Here is the place that we need a bit different logic. Otherwise cursor position
         * and thus new string will not be visible */
        g_oled_dev.cursor.y += ((g_oled_dev.rows_counter <= (OLED_DBG_MAX_ROWS - 1)) ? height : 0);
    } else {
        oled_do_scroll_up(height);
    }
}

/****************************************************************************
 * Name: oled_dbg_render
 *
 * Description:
 *  Debug rendering
 *
 * Input Parameters:
 *  str - pointer to the string to be drawn
 *
 * Returned Values:
 *  Characters number written to the screen. On ERROR returns -1
 *
 ****************************************************************************/

static int oled_dbg_render(const char *str)
{
    int ret = OK;
    FAR const struct nx_fontbitmap_s *fbm;
    FAR struct nxgl_point_s pos;
    FAR struct nxgl_rect_s dest;
    FAR const struct nx_font_s *fontset;
    int count = 0;               /* Number of characters written */

    /* Works with only NX-plane */

    FAR const void *src[CONFIG_NX_NPLANES];
    uint8_t fwidth = 0;
    uint8_t fheight = 0;
    const uint8_t stride = 1;    /* We need only stride to draw properly text on the screen */

    fontset = nxf_getfontset(g_oled_dev.dbg_hfont);

    /* Check if need to scroll text upwards */
    if (g_oled_dev.rows_counter == OLED_DBG_MAX_ROWS) {
        oled_do_scroll_up(OLED_DBG_SPC_BETWEEN_ROWS + fontset->mxheight);
    }

    pos.x = g_oled_dev.cursor.x;
    /* New data always will be printed on next row */
    pos.y = g_oled_dev.cursor.y;
    /* Characters counter will be reseted always. */
    g_oled_dev.chs_count = 0;

    for (; *str; str++) {
        lcd_dbg("Character: %c\n", *str);
        /* Get the bitmap font for this ASCII code */

        fbm = nxf_getbitmap(g_oled_dev.dbg_hfont, *str);

        if (fbm) {
            fwidth = fbm->metric.width + fbm->metric.xoffset;
            fheight = fbm->metric.height + fbm->metric.yoffset;
            memset(g_oled_dev.dbg_glyph, 0, g_oled_dev.dbg_glyph_size);

            (void)nxf_convert_1bpp((FAR nxgl_mxpixel_t*)g_oled_dev.dbg_glyph, fheight, fwidth,
                                   stride, fbm, CONFIG_THINGSEE_DISPLAY_FOREGND_COLOR);

            /* Describe the destination of the font with a rectangle */

            if (g_oled_dev.chs_count > OLED_DBG_MAX_ROW_CHS) {
                /* Jump to the next row */
                oled_move_cursor2new_row(OLED_DBG_SPC_BETWEEN_ROWS + fontset->mxheight);
                pos.x = g_oled_dev.cursor.x;
                pos.y = g_oled_dev.cursor.y;
            }

            dest.pt1.x = pos.x;
            dest.pt1.y = pos.y;

            dest.pt2.x = pos.x + fwidth - 1;
            dest.pt2.y = pos.y + fheight - 1;

            src[0] = (FAR const void *)g_oled_dev.dbg_glyph;

            ret = nx_bitmap((NXWINDOW)g_oled_dev.hwnd, &dest, src, &pos, stride);
            if (ret < 0) {
                lcd_dbg("nx_bitmapwindow failed: %d\n", errno);
                return ERROR;
            }

            pos.x += fwidth;
        } else {
            if (*str == '\n') {
                /* Jump to the next row */
                oled_move_cursor2new_row(OLED_DBG_SPC_BETWEEN_ROWS + fontset->mxheight);
                pos.x = g_oled_dev.cursor.x;
                pos.y = g_oled_dev.cursor.y;
            } else if (*str == '\t') {
                /* In current version you can use tabs only on one row.
                 * May be later spaces will be moved to the next row */
                pos.x += OLED_DBG_SPC_PER_TAB * fontset->spwidth;
                g_oled_dev.chs_count += OLED_DBG_SPC_PER_TAB - 1;
            } else {
                pos.x += fontset->spwidth;
            }
        }

        g_oled_dev.chs_count++;
        /* Characters counter that will be returned */
        count++;
    }

    if (g_oled_dev.rows_counter < OLED_DBG_MAX_ROWS) {
        oled_move_cursor2new_row(OLED_DBG_SPC_BETWEEN_ROWS + fontset->mxheight);
    }

    return count;
}

#if CONFIG_THINGSEE_DISPLAY_ALIGNMENT

/****************************************************************************
 * Name: oled_align_check_text_length
 *
 * Description:
 *  Returns true if text length is smaller than the length of the text block
 *
 * Input Parameters:
 *  hndr - pointer to text block allocated earlier
 *  font - font to be used. See definition from menuconfig
 *  and nuttx/nx/nxfonts.h. Do not forget to activate needed font in
 *  menuconfig
 *  str  - pointer to the string to be written on display
 *
 * Output Parameters:
 *  text_length_px - pointer to the length of the string in pxs. This variable
 *  will return the real length of the string in pxs
 *
 * Returned Values:
 *
 *  True if text length is smaller than the length of the text block
 *
 ****************************************************************************/

static bool oled_align_check_text_length(oled_tblock_t *hndr, enum nx_fontid_e font, const char *str,
                                         int *text_length_px)
{
    *text_length_px = oled_get_text_length_px(font, str);

    return !!(*text_length_px < hndr->width);
}

/****************************************************************************
 * Name: oled_render_multiline_text
 *
 * Description:
 *  Aligns and renders multiline text
 *
 * Input Parameters:
 *  hndr - pointer to text block allocated earlier
 *  font - font to be used. See definition from menuconfig
 *  and nuttx/nx/nxfonts.h. Do not forget to activate needed
 *  font in menuconfig
 *  str  - pointer to the string to be written on display
 *
 * Returned Values:
 *  Characters rendered on the screen
 *
 ****************************************************************************/

static int oled_render_multiline_text(oled_tblock_t *hndr, enum nx_fontid_e font, const char *str)
{
    /* Font package variables */
    FAR const struct nx_fontbitmap_s *fbm;
    FAR const struct nx_font_s *fontset;
    NXHANDLE font_handle;

    uint8_t fwidth = 0;

    /* Normal variables */
    int str_length_pxs;
    int str_height_pxs;
    uint8_t must_render;
    /* I'll use a temporary storage to simplify things */
    char *offset_str;
    /* First offset before the possible line change. Must be initialized to prevent overflows */
    uint8_t offset = 0;
    /* At least one line of text can be rendered, if textblock's height is greater, than
     * the maximum height of the font specified. Thus can_render number is 1 */
    uint8_t can_render = 1;
    /* Variable to hold current height (virtual height) */
    uint8_t virt_height;
    uint16_t i = 0;
    /* Variable to keep special characters index */
    uint8_t special_char = 0;
    /* Text block to keep sub-line in */
    oled_tblock_t sub_line_tb;
    /* Counter to keep chars currently rendered to the screen */
    uint8_t ch_counter = 0;
    /* Sub-word */
    char *sub_word;
    /* Does word fit in sub-line */
    bool is_word_broken = false;
    /* Keeps the pointer to the sub-line of the line */
    char *tmp_sub_line;
    /* Keeps information about string. If string doesn't contain any alphanum characters
     * it's false. In this case function returns zero */
    bool is_alpha = false;


    /* First, get the length of the string in pxs */

    str_length_pxs = oled_get_text_length_px(font, str);

    /* Second, get string maximum height */

    str_height_pxs = oled_get_text_max_height_px(font);

    /* Third, check if text can be rendered (maximum height must be less than
     * text block height) */

    if (str_height_pxs >= hndr->height) {

        /* Nothing can be rendered here. Return zero */

        return 0;
    }

    /* If string consist only of whitespaces, return zero. There is nothing to print.
     * Otherwise very first whitespaces will be "trimmed" by incrementing the string pointer.
     * Sure, we can use here something like while(*str++), but in this case the very first
     * character will be trimmed */

    while(*(str + i)) {
        if (!isspace(*(str + i))) {
            is_alpha = true;
            break;
        }
        i++;
    }

    str += i;

    if (!is_alpha) {
        return 0;
    }

    /* Fourth, calculate lines number, those must be rendered */

    must_render = str_length_pxs / hndr->width + ((str_length_pxs % hndr->width) ? 1 : 0);
    while ((offset_str = strchr(str + offset, '\n')) != NULL) {
        offset = offset_str - str + 1;
        must_render++;
    }

    /* Fifth, calculate lines number, those can be render */

    virt_height = hndr->linespace;
    for (i = 0; i < must_render; i++) {
        virt_height += hndr->linespace;
        /* Check if we still in "the box" */
        if (virt_height > hndr->height) {
            virt_height -= hndr->linespace;
            break;
        }

        /* Yes, we are here, let's increment the "can do" lines' counter */
        can_render++;
    }

    /* Sixth, let's find out the vertical position in the text block.
     * Sub-line's text-block position coordinates and height will be set accordingly */

    sub_line_tb.height = str_height_pxs;
    sub_line_tb.alignment = hndr->alignment;
    sub_line_tb.width = hndr->width;
    sub_line_tb.x = hndr->x;

    switch (hndr->vert_alignment) {
    case OLED_TBLOCK_CENTER_ALIGNMENT:
        sub_line_tb.y = hndr->y + (hndr->height - virt_height) / 2;
        break;
    case OLED_TBLOCK_BOTTOM_ALIGNMENT:
        sub_line_tb.y = hndr->y + (hndr->height - virt_height);
        break;
    default:
        /* Any other case will just put text-block in its old place */
        sub_line_tb.y = hndr->y;
        break;
    }


    /* Seventh, we have the "can do" lines' counter. Now let's break our line to
     * sub-lines: the word will not be broken, but the nearest whitespace will be used
     * to break the line in sub-lines */

    /* Check if we still in "the box". There are 3 possible events:
     * - line is too small (in this case let's add a few characters to the end)
     * - line too big, remove characters till whitespace is found or new line sign
     * appears. In case if there are no any of mentioned characters the word will be divided
     * in two sub-lines
     * - line is just as long as we need */

    font_handle = nxf_getfonthandle(font);
    fontset = nxf_getfontset(font_handle);

    do {
        tmp_sub_line = (char *)str;

        /* To handle cases like one character in a line, i must be greater than 0 */

        for (i = 1; *str; str++, i++) {

            /* Get the bitmap font for this ASCII code */

            fbm = nxf_getbitmap(font_handle, *str);

            if (fbm) {
                fwidth += fbm->metric.width + fbm->metric.xoffset;
            } else {
                if (*str == '\t') {
                    fwidth +=  OLED_SPCPERTAB_NORMAL_TXT * fontset->spwidth;
                    special_char = i - 1;
                } else if (*str == '\n') {
                    break;
                } else {
                    fwidth += fontset->spwidth;
                    special_char = i - 1;
                }
            }

            /* Check if we still in "the box" */

            if (fwidth >= hndr->width) {
                /* Does word fit perfectly */
                if (isspace(*(str + 1))) {
                    is_word_broken = false;
                    str = tmp_sub_line + i - 1;
                } else {
                    is_word_broken = true;
                    str = tmp_sub_line + special_char - 1;
                }
                break;
            }
        }

        /* Decrement counter */
        can_render--;

        /* Allocate memory for the string and copy characters to it */
        if (special_char) {
            /* Was word broken? If was, the closest whitespace will be used */
            i = (is_word_broken ? special_char : i);
        } else {
            str = (is_word_broken ? (str + i - 1) : str);
        }

        /* To make things clear, sizeof(char) will be used here. Used length + 1. 1 is for the '\0' character */
        sub_word = calloc(i + 1, sizeof(char));
        if (!sub_word) {
            /* Return current number of characters rendered */
            return ch_counter;
        }

        memcpy(sub_word, tmp_sub_line, i);
        *(sub_word + i) = '\0';

        /* Trim whitespace from the line end */

        i--;
        while (isspace(*(sub_word + i))) {
            i--;
        }

        /* Move the '\0' character to the last occurrence place of the whitespace.
         * There is a NULL possibility memory overflow happens here */

        *(sub_word + i + 1) = '\0';

        /* Render sub-line */

        ch_counter += oled_hor_align_render_text(&sub_line_tb, font, sub_word);

        /* Free memory */

        free(sub_word);

        /* Change vertical position of the sub-line text-block */

        sub_line_tb.y += hndr->linespace;

        /* Reset special character counter, width and word flag */

        special_char = 0;
        fwidth = 0;
        is_word_broken = false;

        /* Is there any special character!?. if there is, skip it by incrementing
         * the pointer */

        /* There can be a lot of whitespace characters in front of the next sub-line.
         * In case, we have center horizontal alignment it can look very bad. Let's skip them all */

        str++;
        while (*str && isspace(*str)) {
            str++;
        }

    } while (can_render > 0);

    return ch_counter;
}

/****************************************************************************
 * Name: oled_hor_align_render_text
 *
 * Description:
 *  Aligns horizontally and renders string. Single line case.
 *  Multiple line case due to its complexity will be handled
 *  separately.
 *
 * Input Parameters:
 *  hndr - pointer to text block allocated earlier
 *  font - font to be used. See definition from menuconfig
 *  and nuttx/nx/nxfonts.h. Do not forget to activate needed
 *  font in menuconfig
 *  str  - pointer to the string to be written on display
 *
 * Returned Values:
 *  Characters rendered on the screen
 *
 ****************************************************************************/

static int oled_hor_align_render_text(oled_tblock_t *hndr, enum nx_fontid_e font, const char *str)
{
    int left_offset = 0;
    int str_length;
    int count;

    /* If the length of the string is greater than the length of the text block, then
     * text will be aligned to the left side of the text block and characters in the end will not be rendered.
     * Otherwise, offset will be calculated and some old garbage will be cleaned up */

    bool will_fit;

    /* To reduce flickering the amount of white-spaces must be minimal.
     * Thus, only needed offset will be cleared from old garbage */

    oled_tblock_t clear_block;
    struct nxgl_point_s pos;

    /* Initialize vertical position and height of the clear_block */

    clear_block.y = hndr->y;
    clear_block.height = hndr->height;

    /* initialize position */

    pos.x = hndr->x;
    pos.y = hndr->y;

    /* TO DO: in current version the same code is written twice.
     * Think, how we can get reed of the code repeating and still
     * skip unnecessary string length calculating */

    switch (hndr->alignment) {
    case OLED_TBLOCK_RIGHT_ALIGNMENT:
        will_fit = oled_align_check_text_length(hndr, font, str, &str_length);
        if (will_fit) {
            left_offset = hndr->width - str_length;
            clear_block.x = hndr->x;
            clear_block.width = left_offset;
            oled_fill_whitespace(0, &clear_block, &pos, 0);
        }
        break;
    case OLED_TBLOCK_CENTER_ALIGNMENT:
        will_fit = oled_align_check_text_length(hndr, font, str, &str_length);
        if (will_fit) {
            left_offset = (hndr->width - str_length) / 2;
            clear_block.x = hndr->x;
            clear_block.width = left_offset;

            /* The right side of the text block will be cleaned in the end of the rendering process */

            oled_fill_whitespace(0, &clear_block, &pos, 0);
        }
        break;
    default:
        /* Left alignment or something else that doesn't have any effect on text alignment
         * Thus, do nothing here */
        break;
    }

    /* Rendering starts from the offset calculated before */

    hndr->x += left_offset;
    hndr->width -= left_offset;
    count = oled_normal_render(font, str, hndr);
    hndr->x -= left_offset;
    hndr->width += left_offset;

    return count;
}

#endif

/***************************************************************************
 * Public functions
 ***************************************************************************/

/****************************************************************************
 * Name: oled_clear_dbg_screen
 *
 * Description:
 *  Clears debug screen
 *
 * Input Parameters:
 *
 * Returned Values:
 *
 ****************************************************************************/

void oled_clear_dbg_screen(void)
{
    g_oled_dev.rows_counter = 0;
    g_oled_dev.cursor.y = 0;
    oled_fill_screen_with_color(CONFIG_THINGSEE_DISPLAY_BKGND_COLOR);
}

/****************************************************************************
 * Name: oled_get_text_max_height_px
 *
 * Description:
 *  Returns the height of the character in string
 *  in accordance with the font in use
 *
 * Input Parameters:
 *  font - font to be used. See definition from menuconfig
 *
 * Returned Values:
 *  Max height of the character in string in accordance with the font in use
 *
 ****************************************************************************/

int oled_get_text_max_height_px(enum nx_fontid_e font)
{
    FAR const struct nx_font_s *fontset;
    NXHANDLE font_handle;


    font_handle = nxf_getfonthandle(font);
    fontset = nxf_getfontset(font_handle);

    return fontset->mxheight;
}

/****************************************************************************
 * Name: oled_get_text_length_px
 *
 * Description:
 *  Returns the length of the string in pixels
 *
 * Input Parameters:
 *  font - font to be used. See definition from menuconfig
 *  str  - string to be parsed
 *
 * Returned Values:
 *  The width of the string in pixels
 *
 ****************************************************************************/

int oled_get_text_length_px(enum nx_fontid_e font, const char *str)
{
    FAR const struct nx_fontbitmap_s *fbm;
    FAR const struct nx_font_s *fontset;
    NXHANDLE font_handle;

    uint8_t fwidth = 0;
    uint16_t width = 0;

    font_handle = nxf_getfonthandle(font);
    fontset = nxf_getfontset(font_handle);

    for (; *str; str++) {
        lcd_dbg("Character: %c\n", *str);
        /* Get the bitmap font for this ASCII code */

        fbm = nxf_getbitmap(font_handle, *str);

        if (fbm) {
            fwidth = fbm->metric.width + fbm->metric.xoffset;

            /* Increment width, to check if the next character will fit into the text block */

            width += fwidth;

        } else {

            if (*str == '\t') {
                /* In current version you can use tabs only on one row.
                 * May be later spaces will be moved to the next row */
                width +=  OLED_SPCPERTAB_NORMAL_TXT * fontset->spwidth;
            } else {
                width += fontset->spwidth;
            }
        }
    }

    /* One pixel added, to guarantee the text will fit into the text block
     * and there is enough long pat on the right side to prevent text overlapping
     * in case if two text blocks are positioned one after another
     *  without any space between them */

    return width + 1;
}

/****************************************************************************
 * Name: oled_putc
 *
 * Description:
 *  Prints one character on the screen. Advanced interface needed for the
 *  GUI
 *
 * Input Parameters:
 *  hndr - pointer to text block allocated earlier
 *  font - font to be used. See definition from menuconfig
 *  and nuttx/nx/nxfonts.h. Do not forget to activate needed
 *  font in menuconfig
 *  ch   - character to be printed on display
 *
 * Returned Values:
 *  Character's code written. On ERROR returns -1
 *
 ****************************************************************************/

int oled_putc(oled_tblock_t *hndr, enum nx_fontid_e font, const char ch)
{
    char tmp [2] = { 0 };

    tmp[0] = ch;
    tmp[1] = '\0';

#if CONFIG_THINGSEE_DISPLAY_ALIGNMENT
    if (hndr->multiline) {
        return oled_render_multiline_text(hndr, font, (const char *)tmp);
    } else {
        return oled_hor_align_render_text(hndr, font, (const char *)tmp);
    }
#else
    return oled_normal_render(font, tmp, hndr);
#endif
}

/****************************************************************************
 * Name: oled_puts
 *
 * Description:
 *  Prints one characters on the screen. Advanced interface needed for the
 *  GUI
 *
 * Input Parameters:
 *  hndr - pointer to text block allocated earlier
 *  font - font to be used. See definition from menuconfig
 *  and nuttx/nx/nxfonts.h. Do not forget to activate needed
 *  font in menuconfig
 *  ch   - pointer to the string to be written on display
 *
 * Returned Values:
 *  Number of characters written. On ERROR returns -1
 *
 ****************************************************************************/

int oled_puts(oled_tblock_t *hndr, enum nx_fontid_e font, const char *const ch)
{
#if CONFIG_THINGSEE_DISPLAY_ALIGNMENT
    if (hndr->multiline) {
        return oled_render_multiline_text(hndr, font, (const char *)ch);
    } else {
        return oled_hor_align_render_text(hndr, font, (const char *)ch);
    }
#else
    return oled_normal_render(font, ch, hndr);
#endif
}

/****************************************************************************
 * Name: oled_printf
 *
 * Description:
 *  Prints one characters on the screen. Advanced interface needed for the
 *  GUI
 *
 * Input Parameters:
 *  hndr     - pointer to text block allocated earlier
 *  font     - font to be used. See definition from menuconfig
 *  and nuttx/nx/nxfonts.h. Do not forget to activate needed
 *  font in menuconfig
 *  format   - format that will be used with variables past in variable
 *  length list
 *
 * Returned Values:
 *  Number of characters written. On ERROR returns -1
 *
 ****************************************************************************/

int oled_printf(oled_tblock_t *hndr, enum nx_fontid_e font, const char *format, ...)
{
    va_list ap;
    unsigned int char_nbr = 0;        /* Our buffer is not that big, but let's reserve some space for future endeavors */

    /* Clear buffer */
    memset(g_oled_normal_txt_buffer, 0, OLED_NORMAL_TXT_BUF_SIZE);

    va_start(ap, format);
    /* Fill buffer with those characters */
    char_nbr = vsnprintf((char *)g_oled_normal_txt_buffer, sizeof(g_oled_normal_txt_buffer), format, ap);
    if (char_nbr < 0) {
        va_end(ap);
        return ERROR;
    }
    va_end(ap);

    /* Draw them on the screen */
#if CONFIG_THINGSEE_DISPLAY_ALIGNMENT
    if (hndr->multiline) {
        return oled_render_multiline_text(hndr, font, (const char *)g_oled_normal_txt_buffer);
    } else {
        return oled_hor_align_render_text(hndr, font, (const char *)g_oled_normal_txt_buffer);
    }
#else
    return oled_normal_render(font, (const char *)g_oled_normal_txt_buffer, hndr);
#endif
}

/****************************************************************************
 * Name: oled_dbg_putc
 *
 * Description:
 *  Prints one character on the screen. Simple interface needed for
 *  printing debug information on the screen
 *
 * Input Parameters:
 *  ch - character to be printed on display
 *
 * Returned Values:
 *  Character's code written. On ERROR returns -1
 *
 ****************************************************************************/

int oled_dbg_putc(const char ch)
{
    char tmp [2] = { 0 };
    tmp[0] = ch;
    tmp[1] = '\0';
    return oled_dbg_render(tmp);
}

/****************************************************************************
 * Name: oled_dbg_puts
 *
 * Description:
 *  Prints one characters on the screen. Simple interface needed for
 *  printing debug information on the screen
 *
 * Input Parameters:
 *  ch - pointer to the string to be written on display
 *
 * Returned Values:
 *  Number of characters written. On ERROR returns -1
 *
 ****************************************************************************/

int oled_dbg_puts(const char *const ch)
{
    return oled_dbg_render(ch);
}

/****************************************************************************
 * Name: oled_dbg_printf
 *
 * Description:
 *  Prints one characters on the screen. Simple interface needed for
 *  printing debug information on the screen
 *
 * Input Parameters:
 *  format - format that will be used with variables past in variable
 *  length list
 *
 * Returned Values:
 *  Number of characters written. On ERROR returns -1
 *
 ****************************************************************************/

int oled_dbg_printf(const char *format, ...)
{
    va_list ap;
    unsigned int char_nbr = 0;        /* Our buffer is not that big, but let's reserve some space for future endeavors */

    /* Clear buffer */
    memset(g_oled_dbg_buffer, 0, OLED_DBG_BUF_LENGTH);

    va_start(ap, format);
    /* Fill buffer with those characters */
    char_nbr = vsnprintf((char *)g_oled_dbg_buffer, sizeof(g_oled_dbg_buffer), format, ap);
    if (char_nbr < 0) {
        va_end(ap);
        return ERROR;
    }
    va_end(ap);

    /* Draw them on the screen */

    return oled_dbg_render((char *)g_oled_dbg_buffer);
}

/****************************************************************************
 * Name: oled_init_text
 *
 * Description:
 *  Initializes oled_text_t object for further use
 *
 * Input Parameters:
 *  hwnd - background's window handle
 *
 * Returned Values:
 *  Number of characters written. On ERROR returns -1
 *
 ****************************************************************************/

int oled_init_text(NXWINDOW hwnd)
{
    FAR const struct nx_font_s *fontset;
    int ret = OK;


    g_oled_dev.hwnd = hwnd;
    g_oled_dev.cursor.x = 0;        /* Drawing begins from the top left corner */
    g_oled_dev.cursor.y = 0;        /* Drawing begins from the top left corner */
    g_oled_dev.chs_count = 0;       /* Reset characters counter */
    g_oled_dev.rows_counter = 0;    /* Row counter */
    memset(g_oled_dbg_buffer, 0, OLED_DBG_BUF_LENGTH);

    g_oled_dev.dbg_hfont = nxf_getfonthandle(OLED_DBG_FONT);
    if (!g_oled_dev.dbg_hfont) {
        lcd_lldbg("Cannot get font handle for debug prints: %d\n", errno);
        return ERROR;
    }

    fontset = nxf_getfontset(g_oled_dev.dbg_hfont);
    /* Enough memory to keep the largest font */
    g_oled_dev.dbg_glyph_size = fontset->mxwidth * fontset->mxheight;
    g_oled_dev.dbg_glyph = (FAR uint8_t *)malloc(g_oled_dev.dbg_glyph_size);

    if (!g_oled_dev.dbg_glyph) {
        lcd_lldbg("Cannot allocate memory for dbg_glyph: %d\n", errno);
        return ERROR;
    }

    memset(g_oled_dev.dbg_glyph, 0, g_oled_dev.dbg_glyph_size);

    return ret;
}

/****************************************************************************
 * Name: oled_deinit_text
 *
 * Description:
 *  Frees memory
 *
 * Input Parameters:
 *
 * Returned Values:
 *
 ****************************************************************************/

void oled_deinit_text(void)
{
    /* Debug glyph always freed last */

    free(g_oled_dev.dbg_glyph);
}
