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


/*
 * High level interfaces to draw text, windows, pictures etc.
 * */

#ifndef OLED_DISPLAY_H_
#define OLED_DISPLAY_H_

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nxfonts.h>

#include "oled_text_block.h"

/* Text interfaces */
//TO DO: Think, do we need move text interfaces into separate header...

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

int oled_putc(oled_tblock_t *hndr, enum nx_fontid_e font, const char ch);

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

int oled_puts(oled_tblock_t *hndr, enum nx_fontid_e font, const char *const ch);

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

int oled_printf(oled_tblock_t *hndr, enum nx_fontid_e font, const char *format, ...);

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

int oled_dbg_putc(const char ch);

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

int oled_dbg_puts(const char *const ch);

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

int oled_dbg_printf(const char *format, ...);

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

int oled_init_text(NXWINDOW hwnd);

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

void oled_deinit_text(void);

/* Image helper interface */

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

void oled_clear_dbg_screen(void);

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

int oled_get_text_max_height_px(enum nx_fontid_e font);

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

int oled_get_text_length_px(enum nx_fontid_e font, const char *str);

/****************************************************************************
 * Name: oled_fill_screen_with_color
 *
 * Description:
 *  Fills display with specified color
 *
 * Input Parameters:
 * color - color to fill display with (black or white)
 *
 * Returned Values:
 *  OK on success. On ERROR returns -1
 *
 ****************************************************************************/

int oled_fill_screen_with_color(uint8_t color);

/* Modules interface */

/****************************************************************************
 * Name: oled_start_module
 *
 * Description:
 *  Initializes oled module.
 *
 * Input Parameters:
 *
 * Returned Values:
 *  OK on success. On ERROR returns -1
 *
 ****************************************************************************/

int oled_start_module(void);

/****************************************************************************
 * Name: oled_stop_module
 *
 * Description:
 *  Stops oled module.
 *
 * Input Parameters:
 *
 * Returned Values:
 *  OK on success. On ERROR returns -1
 *
 ****************************************************************************/

void oled_stop_module(void);

#endif /* OLED_DISPLAY_H_ */
