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
 * Author: Petri Salonen <petri.salonen@haltian.com>
 *
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/version.h>
#include <stdio.h>
#include <stdlib.h>

#include <apps/thingsee/modules/ts_emmc.h>
#include <arch/board/board.h>
#include <arch/board/board-reset.h>
#include <arch/board/board-device.h>
#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nxfonts.h>
#include "../../../apps/system/display/oled_display.h"
#include "../../../apps/system/display/oled_text_block.h"
#include "../../../apps/system/display/oled_inverter.h"
#include "../../../apps/system/display/oled_image.h"

#include <apps/ts_engine/ts_engine.h>

#include <thingsee_ui.h>
#include <ui_bitmaps.h>
#include <ui_connectivity.h>
#include <ui_calibration.h>
#include <ui_menu.h>

#define ts_watchdog_kick() ((void)0)

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
#define SIDE_MARGIN                       20   /* Pixels */
#define MAX_MENU_LINES                    3    /* Lines */
#define MODE_SWITCH_SLEEP_TIME            2    /* Seconds */
#define CALIB_INFO_SLEEP_TIME             3    /* Seconds */
#define CALIB_COUNT_TIME                  1    /* Seconds */
#define CALIB_COUNT_TOTAL_TIME            5    /* Seconds */
#define MAX_CALIBRATION_TEXT_LINE_LENGTH  100  /* Characters */
#define USB_INFO_SLEEP_TIME               3    /* Seconds */

/****************************************************************************
* Private Function Prototypes
*****************************************************************************/
static void show_menu(uint8_t first_item, uint8_t highlight);
static uint8_t correct_menuitem_index(uint8_t menuitem);
static void display_about_product(void);
#ifdef CONFIG_THINGSEE_UI_DFU_MODE
static void dfu_mode_switch(void);
#endif
static void reset_sw(void);
static void show_text_and_sleep(const char *str, unsigned int sleeptime);
#ifdef CONFIG_THINGSEE_NINEAXELS_MODULE
static void device_calibration(int sensor_id);
#endif
static bool check_usb_attached_state(bool wanted_state, const char *str);

/****************************************************************************
* Private Data
****************************************************************************/
static uint8_t menu_top_item = 0;
static uint8_t menu_highlight_line = 0;
static bool menu_displayed = false;
static bool deeper_in_menu = false;

typedef enum {
    MENUITEM_EMPTY = -1,
    MENUITEM_CHECK,
    MENUITEM_BACKEND,

#ifdef CONFIG_THINGSEE_UI_DFU_MODE
    MENUITEM_DFU,
#endif

#ifdef CONFIG_THINGSEE_NINEAXELS_MODULE

    SUBMENU_CALIBRATION,
      MENUITEM_GYRO_CALIB,
      MENUITEM_MAG_CALIB,

#endif

    MENUITEM_ABOUT_PRODUCT,
    MENUITEM_RESET,
    MENUITEM_BACK
} UI_menuitem_t;

typedef struct {
    UI_menuitem_t item_number;
    const char *item_name;
} UI_menuitems_t;

static const UI_menuitems_t main_menu[] = {
    {MENUITEM_CHECK, "Connection check"},
    {MENUITEM_BACKEND, "Backend update"},
#ifdef CONFIG_THINGSEE_UI_DFU_MODE
    {MENUITEM_DFU, "Dfu mode"},
#endif
#ifdef CONFIG_THINGSEE_NINEAXELS_MODULE
    {SUBMENU_CALIBRATION, "Calibration"},
#endif
    {MENUITEM_ABOUT_PRODUCT, "About product"},
    {MENUITEM_RESET, "Restart device"},
    {MENUITEM_BACK, "Back"}
};

#ifdef CONFIG_THINGSEE_NINEAXELS_MODULE
static const UI_menuitems_t calibration_submenu[] = {
    {MENUITEM_GYRO_CALIB, "Gyroscope"},
    {MENUITEM_MAG_CALIB, "Magnetometer"},
    {MENUITEM_BACK, "Back"}
};
#endif

static UI_menuitem_t g_selected_menu_item = MENUITEM_EMPTY;
static int g_about_product_scr_counter = 0;

static const UI_menuitems_t *current_menu;

/****************************************************************************
* UI strings
****************************************************************************/
static const char g_settings_str[] = "Settings";
static const char g_version_str[] = "SW version";
static const char g_contains_word[] = "Contains:";
static const char g_model_str_1[] = "Model: SARAG350";
static const char g_model_str_2[] = "Model: CC3000EM";
static const char g_fcc_ids_sara[] = "FCC ID: XPYSARAG350";
static const char g_fcc_ids_cc3000[] = "FCC ID: Z64-CC3000EM";
static const char g_ic_1[] = "& IC: 8595A-SARAG350";
static const char g_ic_2[] = "& IC: 451I-CC3000EM";
#ifdef CONFIG_THINGSEE_UI_DFU_MODE
static const char g_indfumode_str[] = "Going to DFU mode";
#endif
#ifdef CONFIG_THINGSEE_NINEAXELS_MODULE
static const char g_calib_gyro_info_1_str[] = "Starting \n gyroscope \n calibration";
static const char g_calib_gyro_info_2_str[] = "Place the device \n on stable \n surface";
static const char g_calib_mag_info_1_str[] = "Starting \n magnetometer \n calibration";
static const char g_calib_mag_info_2_str[] = "Wave device in figure eight during calibration";
static const char g_calib_starting_str[] = "Calibration is \n starting in \n %d second%c";
static const char g_calib_calibrating_str[] = "Calibrating";
static const char g_calib_done_str[] = "Calibration done";
static const char g_calib_fail_str[] = "Calibration failed!";
#endif
static const char g_attach_usb_str[] = "Please attach \n USB cable \n first!";
static const char g_remove_usb_str[] = "Please remove \n USB cable \n first!";

/****************************************************************************
* Public Functions
****************************************************************************/
void thingsee_UI_show_menu(void)
{
    struct oled_tblock_s loc = {.x = SIDE_MARGIN, .y = 21, .width = OLED_SCREEN_WIDTH - SIDE_MARGIN * 2, .height = 22, CENTER_ALIGN};
    const oled_image_canvas_t *text_select_img;

    oled_fill_screen_with_color(CONFIG_THINGSEE_DISPLAY_BKGND_COLOR);
    text_select_img=UI_get_text_img(TEXT_SELECT);
    oled_image_draw_img(OLED_SCREEN_WIDTH - text_select_img->width, (OLED_SCREEN_HEIGHT - text_select_img->height) / 2, text_select_img);

    oled_printf(&loc, FONTID_SANS17X22, g_settings_str);
    menu_top_item = 0;
    menu_highlight_line = 0;
    menu_displayed = false;
    deeper_in_menu = false;
    current_menu = NULL;
    g_selected_menu_item = MENUITEM_EMPTY;
    g_about_product_scr_counter = 0;
}

void thingsee_UI_return_to_menu(void)
{
    oled_fill_screen_with_color(CONFIG_THINGSEE_DISPLAY_BKGND_COLOR);
    show_menu(menu_top_item, menu_highlight_line);
    deeper_in_menu = false;
    g_selected_menu_item = MENUITEM_EMPTY;
    g_about_product_scr_counter = 0;
}

void thingsee_UI_menu_received_power_key(void)
{
    int item, index;
#ifdef CONFIG_THINGSEE_NINEAXELS_MODULE
    static uint8_t old_menu_top_item;
    static uint8_t old_menu_highlight_line;
#endif

    if (menu_displayed) {
        index = correct_menuitem_index(menu_top_item + menu_highlight_line);
        item = current_menu[index].item_number;
        deeper_in_menu = true;

        if (current_menu == main_menu) {
            switch (item) {
                case MENUITEM_CHECK:
                    connectivity_UI_received_power_key(true);
                    break;

                case MENUITEM_BACKEND:
                    if (!check_usb_attached_state(false, g_remove_usb_str))
                        connectivity_UI_received_power_key(false);
                    else
                        thingsee_UI_return_to_menu();
                    break;

#ifdef CONFIG_THINGSEE_UI_DFU_MODE
                case MENUITEM_DFU:
                    if (check_usb_attached_state(true, g_attach_usb_str))
                        dfu_mode_switch();
                    else
                        thingsee_UI_return_to_menu();
                    break;
#endif
#ifdef CONFIG_THINGSEE_NINEAXELS_MODULE
                case SUBMENU_CALIBRATION:
                    current_menu = calibration_submenu;

                    /* Store old location as return position */

                    old_menu_top_item = menu_top_item;
                    old_menu_highlight_line = menu_highlight_line;
                    menu_top_item = 0;
                    menu_highlight_line = 0;
                    deeper_in_menu = false;
                    show_menu(menu_top_item, menu_highlight_line);
                    break;
#endif
                case MENUITEM_ABOUT_PRODUCT:
                    if (g_selected_menu_item != MENUITEM_ABOUT_PRODUCT) {
                        g_selected_menu_item = MENUITEM_ABOUT_PRODUCT;
                        deeper_in_menu = false;
                        display_about_product();
                    } else {
                        thingsee_UI_return_to_menu();
                    }
                    break;

                case MENUITEM_RESET:
                    reset_sw();
                    break;

                case MENUITEM_BACK:
                    thingsee_UI_show_menu();
                    break;

                default:
                    break;
            }
        }
#ifdef CONFIG_THINGSEE_NINEAXELS_MODULE
        else if (current_menu == calibration_submenu) {
            switch (item) {
                case MENUITEM_GYRO_CALIB:
                    device_calibration(UI_CALIB_SENS_GYROSCOPE);
                    break;

                case MENUITEM_MAG_CALIB:
                    device_calibration(UI_CALIB_SENS_MAGN);
                    break;

                case MENUITEM_BACK:
                    /* Restore main menu and position */
                    current_menu = main_menu;
                    menu_top_item = old_menu_top_item;
                    menu_highlight_line = old_menu_highlight_line;
                    deeper_in_menu = false;
                    show_menu(menu_top_item, menu_highlight_line);
                    break;

                default:
                    break;
            }
        }
#endif
    } else {
        menu_displayed = true;
        current_menu = main_menu;
        show_menu(menu_top_item, menu_highlight_line);
    }
}

void thingsee_UI_menu_received_capsense_key(void)
{
    if (menu_displayed && !deeper_in_menu) {
        if (g_selected_menu_item != MENUITEM_ABOUT_PRODUCT) {

            if (menu_highlight_line == 0)
                menu_highlight_line = 1;
            else
                menu_top_item = correct_menuitem_index(menu_top_item + 1);
            show_menu(menu_top_item, menu_highlight_line);
        } else {
            if (g_about_product_scr_counter == 2) {
                g_about_product_scr_counter = 0;
            } else {
                g_about_product_scr_counter++;
            }

            /* Refresh the screen to display a correct sub-screen */
            display_about_product();
        }
    }
}

bool is_screen_change_allowed(void)
{
    /* Changing screens or turning it off is not allowed unless we are on main screens */
    if (!menu_displayed)
        return true;

    return false;
}

/****************************************************************************
* Private Functions
****************************************************************************/
static void show_menu(uint8_t first_item, uint8_t highlight)
{
    struct oled_tblock_s loc[3] = {
            {.x = 0, .y = 0, .width = OLED_SCREEN_WIDTH - SIDE_MARGIN, .height = 22},
            {.x = 0, .y = 22, .width = OLED_SCREEN_WIDTH - SIDE_MARGIN, .height = 22},
            {.x = 0, .y = 44, .width = OLED_SCREEN_WIDTH - SIDE_MARGIN, .height = 22}
    };
    const oled_image_canvas_t *text_select_img;
    int i, item;

    text_select_img=UI_get_text_img(TEXT_SELECT);
    oled_image_draw_img(OLED_SCREEN_WIDTH - text_select_img->width, (OLED_SCREEN_HEIGHT - text_select_img->height) / 2, text_select_img);

    for (i = 0; i < MAX_MENU_LINES ; i++) {
        item = correct_menuitem_index(first_item + i);
        oled_printf(&loc[i], FONTID_SANS17X22, current_menu[item].item_name);
    }
    oled_invertor_do_invert(loc[highlight].x, loc[highlight].y, loc[highlight].width, loc[highlight].height);
}

static uint8_t correct_menuitem_index(uint8_t menuitem) {

    if (current_menu == main_menu) {
        if (menuitem >= sizeof(main_menu) / sizeof(UI_menuitems_t))

            return menuitem - sizeof(main_menu) / sizeof(UI_menuitems_t);

    }
#ifdef CONFIG_THINGSEE_NINEAXELS_MODULE
    else if (current_menu == calibration_submenu) {

        if (menuitem >= sizeof(calibration_submenu) / sizeof(UI_menuitems_t))
            return menuitem - sizeof(calibration_submenu) / sizeof(UI_menuitems_t);

    }
#endif

    return menuitem;
}

static void display_sw_version(void)
{
    struct oled_tblock_s loc_1 = {.x = 0, .y = 0, .width = OLED_SCREEN_WIDTH - SIDE_MARGIN, .height = 22, CENTER_ALIGN};
    struct oled_tblock_s loc_2 = {.x = 0, .y = 22, .width = OLED_SCREEN_WIDTH - SIDE_MARGIN, .height = 20,
        .multiline = true, .linespace = 10, .alignment = OLED_TBLOCK_LEFT_ALIGNMENT, .vert_alignment = OLED_TBLOCK_CENTER_ALIGNMENT};
    const oled_image_canvas_t *text_back_img;

    oled_fill_screen_with_color(CONFIG_THINGSEE_DISPLAY_BKGND_COLOR);
    text_back_img = UI_get_text_img(TEXT_BACK);
    oled_image_draw_img(OLED_SCREEN_WIDTH - text_back_img->width, (OLED_SCREEN_HEIGHT - text_back_img->height) / 2, text_back_img);

    oled_printf(&loc_1, FONTID_SANS17X22, g_version_str);
    oled_printf(&loc_2, FONTID_MONO5X8, CONFIG_VERSION_BUILD);
}

static void display_fcc_ids(int index)
{
    /* Text: "Contains:" */
    struct oled_tblock_s loc_1 = { .x = 0, .y = 15, .width = OLED_SCREEN_WIDTH - SIDE_MARGIN, .height = 10, LEFT_ALIGN };
    /* Text: "Model: SARAG350" or "Model: CC3000EM" */
    struct oled_tblock_s loc_2 = { .x = 0, .y = 0, .width = OLED_SCREEN_WIDTH - SIDE_MARGIN, .height = 10, LEFT_ALIGN };
    /* Text: "FCC ID: XPYSARAG350" or "FCC ID: Z64-CC3000EM" */
    struct oled_tblock_s loc_3 = { .x = 0, .y = 29, .width = OLED_SCREEN_WIDTH - SIDE_MARGIN, .height = 10, LEFT_ALIGN };
    /* Text: "IC: 8595A-SARAG350" or "IC: 451I-CC3000EM" */
    struct oled_tblock_s loc_4 = { .x = 0, .y = 43, .width = OLED_SCREEN_WIDTH - SIDE_MARGIN, .height = 10, LEFT_ALIGN };
    const oled_image_canvas_t *text_back_img;

    oled_fill_screen_with_color(CONFIG_THINGSEE_DISPLAY_BKGND_COLOR);
    text_back_img = UI_get_text_img(TEXT_BACK);
    oled_image_draw_img(OLED_SCREEN_WIDTH - text_back_img->width, (OLED_SCREEN_HEIGHT - text_back_img->height) / 2, text_back_img);

    oled_printf(&loc_1, FONTID_MONO5X8, g_contains_word);

    if (index) {
        oled_printf(&loc_2, FONTID_MONO5X8, g_model_str_2);
        oled_printf(&loc_3, FONTID_MONO5X8, g_fcc_ids_cc3000);
        oled_printf(&loc_4, FONTID_MONO5X8, g_ic_2);
    } else {
        oled_printf(&loc_2, FONTID_MONO5X8, g_model_str_1);
        oled_printf(&loc_3, FONTID_MONO5X8, g_fcc_ids_sara);
        oled_printf(&loc_4, FONTID_MONO5X8, g_ic_1);
    }
}

static void display_about_product(void)
{
    switch (g_about_product_scr_counter) {
    case 0:
    case 1:
        display_fcc_ids(g_about_product_scr_counter);
        break;
    case 2:
        display_sw_version();
        break;
    default:
        break;
    }
}

#ifdef CONFIG_THINGSEE_UI_DFU_MODE
static void dfu_mode_switch(void)
{
    /* Display DFU mode text for few seconds before turning the screen off and the mode on */
    show_text_and_sleep(g_indfumode_str, MODE_SWITCH_SLEEP_TIME);
    board_lcdoff();
    thingsee_UI_reset_device("dfu");
}
#endif

static void show_text_and_sleep(const char *str, unsigned int sleeptime)
{
    struct oled_tblock_s loc = {.x = 0, .y = 0, .width = OLED_SCREEN_WIDTH, .height = OLED_SCREEN_HEIGHT,
        .multiline = true, .linespace = 20, .alignment = OLED_TBLOCK_CENTER_ALIGNMENT, .vert_alignment = OLED_TBLOCK_CENTER_ALIGNMENT};

    /* Display the text for a short while before returning */
    oled_fill_screen_with_color(CONFIG_THINGSEE_DISPLAY_BKGND_COLOR);
    oled_printf(&loc, FONTID_SANS17X22, str);
    sleep(sleeptime);
}

static void reset_sw(void)
{
    board_lcdoff();
    thingsee_UI_reset_device("reset");
}

#ifdef CONFIG_THINGSEE_NINEAXELS_MODULE

static void device_calibration(int sensor_id)
{

    int count;
    int result;
    char *text_line;

    /* Timeconsuming operation, kick the watchdog to avoid resets */

    if (board_get_hw_ver() < BOARD_HWVER_B2_0)
        ts_watchdog_kick();

    /* Pause purpose during the calibration */
    ts_engine_stop(thingsee_UI_get_app_instance(), false, false);

    /* Show instructions */
    oled_fill_screen_with_color(CONFIG_THINGSEE_DISPLAY_BKGND_COLOR);
    if (sensor_id == UI_CALIB_SENS_GYROSCOPE) {
        show_text_and_sleep(g_calib_gyro_info_1_str, CALIB_INFO_SLEEP_TIME);
        oled_fill_screen_with_color(CONFIG_THINGSEE_DISPLAY_BKGND_COLOR);
        show_text_and_sleep(g_calib_gyro_info_2_str, CALIB_INFO_SLEEP_TIME);
    } else {
        show_text_and_sleep(g_calib_mag_info_1_str, CALIB_INFO_SLEEP_TIME);
        oled_fill_screen_with_color(CONFIG_THINGSEE_DISPLAY_BKGND_COLOR);
        show_text_and_sleep(g_calib_mag_info_2_str, CALIB_INFO_SLEEP_TIME);
    }

    oled_fill_screen_with_color(CONFIG_THINGSEE_DISPLAY_BKGND_COLOR);
    count = CALIB_COUNT_TOTAL_TIME;

    /* Since "pojut" do not want to use memset,
     * every malloc will be replaced with calloc */

    text_line = calloc(1, MAX_CALIBRATION_TEXT_LINE_LENGTH);

    while (text_line != NULL && count > 0) {
        snprintf(text_line, MAX_CALIBRATION_TEXT_LINE_LENGTH, g_calib_starting_str, count, count != 1 ? 's' : ' ');
        show_text_and_sleep(text_line, CALIB_COUNT_TIME);
        count--;
    }

    free(text_line);

    oled_fill_screen_with_color(CONFIG_THINGSEE_DISPLAY_BKGND_COLOR);
    show_text_and_sleep(g_calib_calibrating_str, CALIB_INFO_SLEEP_TIME);

    /* Run the calibration */

    result = ui_calib_lsm9ds1_calibrate(sensor_id);

    /* Refresh watchdog just in case */
    if (board_get_hw_ver() < BOARD_HWVER_B2_0)
        ts_watchdog_kick();

    /* Report calibration result */
    oled_fill_screen_with_color(CONFIG_THINGSEE_DISPLAY_BKGND_COLOR);
    if (result == OK)
        show_text_and_sleep(g_calib_done_str, CALIB_INFO_SLEEP_TIME);
    else
        show_text_and_sleep(g_calib_fail_str, CALIB_INFO_SLEEP_TIME);

    /* Resume purpose when finished */
    ts_engine_continue(thingsee_UI_get_app_instance());

    thingsee_UI_return_to_menu();
}

#endif

static bool check_usb_attached_state(bool wanted_state, const char *str)
{
    bool pcusb_state;

    pcusb_state = thingsee_UI_get_PC_USB_connected();

    /* Show note if the usb is not in wanted state */
#ifndef CONFIG_BOARD_RND_USE_USB_CONSOLE
    if (pcusb_state != wanted_state)
        show_text_and_sleep(str, USB_INFO_SLEEP_TIME);

    /* Return the usb state */
    return pcusb_state;
#else
    /* Because usb connection type == "nonstd" */
    dbg("check_usb_attached_state: pcusb_state: %d",pcusb_state);
    return wanted_state;
#endif
}
