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

#include <sys/types.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/lcd/lcd.h>

#include <nuttx/arch.h>
#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nxfonts.h>

#include "oled_display.h"
#include "oled_inverter.h"
#include "oled_image.h"
#include "qr/qr_main.h"


#ifdef CONFIG_THINGSEE_DISPLAY_TRACES
#  define lcd_dbg(x, ...)    dbg(x, ##__VA_ARGS__)
#  define lcd_lldbg(x, ...)  lldbg(x, ##__VA_ARGS__)
#else
#  define lcd_dbg(x, ...)
#  define lcd_lldbg(x, ...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/
typedef struct nx_data_s {
	/* The NX handles */
	NXHANDLE hnx;
	NXHANDLE hbkgd;		/* Background used for the all objects drawing */
	/* The screen resolution */
	nxgl_coord_t xres;
	nxgl_coord_t yres;
	sem_t sem;
	bool is_pos;
} nx_data_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* To make compiler happy */
extern void ssd1306_fill(FAR struct lcd_dev_s *dev, uint8_t color);

static void oled_redraw(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                        bool morem, FAR void *arg);
static void oled_position(NXWINDOW hwnd, FAR const struct nxgl_size_s *size,
                          FAR const struct nxgl_point_s *pos,
                          FAR const struct nxgl_rect_s *bounds,
                          FAR void *arg);
#ifdef CONFIG_NX_XYINPUT
static void oled_mousein(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos,
                         uint8_t buttons, FAR void *arg);
#endif

#ifdef CONFIG_NX_KBD
static void oled_kbdin(NXWINDOW hwnd, uint8_t nch, FAR const uint8_t *ch,
                       FAR void *arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Background window call table */

static const struct nx_callback_s g_nxoledcb = {
    oled_redraw,   /* redraw */
    oled_position  /* position */
#ifdef CONFIG_NX_XYINPUT
    , oled_mousein /* mousein */
#endif
#ifdef CONFIG_NX_KBD
    , oled_kbdin   /* my kbdin */
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

static struct nx_data_s g_nx_oled = {
    NULL,          /* hnx */
    NULL,          /* hbkgd */
    0,             /* xres */
    0,             /* yres */
    { 0 }          /* sem */
 };

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void oled_redraw(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                        bool more, FAR void *arg)
{
    lcd_dbg("hwnd=%p rect={(%d,%d),(%d,%d)} more=%s\n",
               hwnd, rect->pt1.x, rect->pt1.y, rect->pt2.x, rect->pt2.y,
               more ? "true" : "false");
}

static void oled_position(NXWINDOW hwnd, FAR const struct nxgl_size_s *size,
                          FAR const struct nxgl_point_s *pos,
                          FAR const struct nxgl_rect_s *bounds,
                          FAR void *arg)
{
    /* Report the position */

    lcd_dbg("hwnd=%p size=(%d,%d) pos=(%d,%d) bounds={(%d,%d),(%d,%d)}\n",
            hwnd, size->w, size->h, pos->x, pos->y,
            bounds->pt1.x, bounds->pt1.y, bounds->pt2.x, bounds->pt2.y);

    /* Save the background window handle */
if (!g_nx_oled.is_pos){
    g_nx_oled.hbkgd = hwnd;

    /* Save the window limits */

    g_nx_oled.xres = bounds->pt2.x + 1;
    g_nx_oled.yres = bounds->pt2.y + 1;

    sem_post(&g_nx_oled.sem); g_nx_oled.is_pos = true;}
    lcd_dbg("Have xres=%d yres=%d\n", g_nx_oled.xres, g_nx_oled.yres);
}

#ifdef CONFIG_NX_XYINPUT
static void oled_mousein(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos,
                         uint8_t buttons, FAR void *arg)
{
    lcd_dbg("oled_mousein: hwnd=%p pos=(%d,%d) button=%02x\n",
                 hwnd,  pos->x, pos->y, buttons);
}
#endif

#ifdef CONFIG_NX_KBD
static void oled_kbdin(NXWINDOW hwnd, uint8_t nch, FAR const uint8_t *ch,
                       FAR void *arg)
{
    lcd_dbg("hwnd=%p nch=%d\n", hwnd, nch);

    /* In this example, there is no keyboard so a keyboard event is not
    * expected.
    */

    lcd_dbg("oled_kbdin: Unexpected keyboard callback\n");
}
#endif

/****************************************************************************
 * Name: oled_initialize
 *
 * Description:
 *  Initializes oled driver
 *
 * Input Parameters:
 *
 * Returned Values:
 *  OK on success. On ERROR returns -1
 *
 ****************************************************************************/

static int oled_initialize(void)
{
    FAR NX_DRIVERTYPE *dev;
    int ret;

    /* Initialize the LCD driver */

    lcd_dbg("Initializing LCD-driver\n");
    ret = board_lcd_initialize();
    if (ret < 0) {
        perror("Cannot initialize LCD-driver");
        return ERROR;
    }

    /* Get the device instance */

    dev = board_lcd_getdev(0);
    if (!dev) {
        perror("Cannot get device instance");
        return ERROR;
    }

    /* Then open NX */

    lcd_dbg("Open NX\n");
    g_nx_oled.hnx = nx_open(dev);
    if (!g_nx_oled.hnx) {
        perror("Cannot create NX handle");
        return ERROR;
    }

    /* Initialize semaphore */

    sem_init(&(g_nx_oled.sem), 0, 0);

    return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

int oled_fill_screen_with_color(uint8_t color)
{
    int ret = OK;
    struct lcd_dev_s *lcd;

    lcd = board_lcd_getdev(0);
    if (!lcd)
        return ERROR;

    ssd1306_fill(lcd, color);

    return ret;
}

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

int oled_start_module(void)
{
    nxgl_mxpixel_t color;
    int ret = OK;

    g_nx_oled.is_pos = false;

    /* Initialize NX */

    ret = oled_initialize();
    lcd_dbg("NX handle=%p\n", g_nx_oled.hnx);
    if (!g_nx_oled.hnx || ret < 0) {
        perror("Cannot initialize oled");
        goto errout;
    }

    /* Set the background to the configured background color */

    color = CONFIG_THINGSEE_DISPLAY_BKGND_COLOR;
    ret = nx_setbgcolor(g_nx_oled.hnx, &color);
    if (ret < 0) {
        perror("Cannot set background color");
        goto errout_with_nx;
    }

    /* Get the background window. For now the only background window is quite enough
     * Read specifications for the answers on possible "why's" and "what's" */

    ret = nx_requestbkgd(g_nx_oled.hnx, &g_nxoledcb, NULL);
    if (ret < 0) {
        perror("Cannot request background");
        goto errout_with_nx;
    }

    /* Wait until we have the screen resolution.  We'll have this immediately
    * unless we are dealing with the NX server.
    */
    (void)sem_wait(&g_nx_oled.sem);

    lcd_dbg("Screen resolution (%d,%d)\n", g_nx_oled.xres, g_nx_oled.yres);

    // TO DO: add submodules initialization here
    /* Init text submodule */

    ret = oled_init_text(g_nx_oled.hbkgd);
    if (ret < 0) {
        perror("Cannot initialize text submodule");
        goto errout_with_bckgrnd;
    }

#if CONFIG_THINGSEE_DISPLAY_QR
    /* Run QR module */
    qr_main_init(g_nx_oled.hbkgd);
#endif

#if CONFIG_THINGSEE_DISPLAY_INVERT
    /* Initialize inverter submodule */
    oled_inverter_init(g_nx_oled.hbkgd);
#endif

    /* Initialize image */
    oled_image_init(g_nx_oled.hbkgd);

    return ret;

errout_with_bckgrnd:
    /* Release background */
    (void)nx_releasebkgd(g_nx_oled.hbkgd);

errout_with_nx:
    lcd_dbg("Close NX\n");
    nx_close(g_nx_oled.hnx);
errout:
    return ret;
}

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

void oled_stop_module(void)
{
    /* Release background */

    (void)nx_releasebkgd(g_nx_oled.hbkgd);
    lcd_dbg("Background released\n");
    nx_close(g_nx_oled.hnx);
    lcd_dbg("NX closed\n");

    //TO DO: add submodules release here
    /* Deinit text submodule */

    oled_deinit_text();

    /* Turn off display */

    board_lcd_uninitialize();

    /* Deinit semaphore */

    (void)sem_destroy(&(g_nx_oled.sem));
}
