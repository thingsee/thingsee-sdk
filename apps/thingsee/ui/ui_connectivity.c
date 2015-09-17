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
#include <stdio.h>
#include <stdlib.h>

#include <apps/thingsee/ts_core.h>
#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nxfonts.h>
#include "../../../apps/system/display/oled_display.h"
#include "../../../apps/system/display/oled_text_block.h"
#include "../../../apps/system/display/oled_image.h"
#include <apps/thingsee/modules/ts_gps.h>
#include <apps/system/conman.h>
#include <apps/ts_engine/ts_engine.h>
#include <apps/thingsee/modules/ts_bluetooth.h>
#include <thingsee_ui.h>
#include <ui_bitmaps.h>
#include <ui_menu.h>
#include <arch/board/board-reset.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
#define SIDE_MARGIN             20   /* Pixels */
#define CATEGORY_IMAGE_SIZE     16   /* Pixels */
#define CATEGORY_Y_OFFSET       16   /* Pixels */
#define TEXTLINE_Y_OFFSET       4    /* Pixels */
#define GPS_FIX_TIMEOUT         300  /* Seconds (5 minutes) */
#define CONNECTION_TIMEOUT      120  /* Seconds (2 minutes) */
#define MAX_STATUS_LINE_LENGTH  32   /* Characters */
#define CONNECT_STATUS_TIMEOUT  1000 /* Milliseconds (1 second) */
#define BT_CONNECT_TIMEOUT      5000 /* Milliseconds (5 seconds) */
#define ALL_TESTS_RUNNING       0x0F /* Binary 00001111 */

/****************************************************************************
* Private Function Prototypes
*****************************************************************************/
static void backend_connecting_view(void);
static void connectivity_checking_view(void);
static void backend_connecting_view_updated(enum backend_update_status status);
static void backend_update_cb(enum backend_update_status status);
static void show_connectivity_symbols(void);
static int start_gps_connectivity(void);
static void UI_gps_callback(void const * const e, void * const priv);
static int stop_gps_connectivity(void);
static void stop_running_purpose(void);
static void resume_running_purpose(void);
static int connection_status_timeout(const int timer_id, void * const priv);
static void stop_connection_activities(void);
static void update_test_result(uint8_t test_id, const char* result);
static void start_next_test(void);
static void show_test_result(void);
static void change_button_if_done(void);
static int run_bluetooth_test(void);
#ifdef CONFIG_THINGSEE_BLUETOOTH_MODULE
static void ts_app_bt_cb(void const * const e, void * const priv);
static int bt_connection_status_timeout(const int timer_id, void * const priv);
static void finish_bt_test(const char* result);
#endif
static int connection_timeout(const int timer_id, void * const priv);

/****************************************************************************
* Private Data
****************************************************************************/
typedef enum {
    BACKEND_CONNECTIVITY_SUBSCREEN,
    BACKEND_CONNECTING_SUBSCREEN,
    BACKEND_UPDATED_SUBSCREEN
} backend_connectivity_subscreens_t;
static backend_connectivity_subscreens_t backend_connectivity_subscreen = BACKEND_CONNECTIVITY_SUBSCREEN;

typedef enum {
    CONNECTIVITY_CHECK_SUBSCREEN,
    CONNECTIVITY_CHECKING_SUBSCREEN
} connectivity_check_subscreens_t;
static connectivity_check_subscreens_t connectivity_check_subscreen = CONNECTIVITY_CHECK_SUBSCREEN;

typedef enum {
    NO_TEST,
    WIFI_TEST,
    CELLULAR_TEST,
    BLUETOOTH_TEST,
    GPS_TEST
} connection_tests_t;

typedef union {
    struct {
        uint8_t wifi_test : 1;
        uint8_t cellular_test : 1;
        uint8_t bluetooth_test : 1;
        uint8_t gps_test : 1;
    };
    uint8_t tests : 4;
} running_tests_t;

static struct conman_client_s g_conman_client;
static struct conman_status_s g_conman_status;
static int g_connection_status_timeout_timer = -1;
static int g_connection_timeout_timer = -1;
static uint32_t g_connection_test_connid = CONMAN_CONNID_CLEAR;
static connection_tests_t g_running_test;
static running_tests_t running_tests;

#ifdef CONFIG_THINGSEE_BLUETOOTH_MODULE
static int g_bt_connection_timeout_timer;
#endif

static uint8_t connect_test(connection_tests_t test);

/****************************************************************************
* UI strings
****************************************************************************/
static const char g_test_started_str[] = "Checking...";
static const char g_test_nottested_str[] = "Not tested";
static const char g_test_passed_str[] = "Test passed";
static const char g_test_failed_str[] = "FAILED";
static const char g_test_stopped_str[] = "Test stopped";
static const char g_connecting_str[] = "Connecting...";
static const char g_backend_updated_str[] = "Purpose updated from backend";
static const char g_backend_ok_str[] = "Backend connection successful";
static const char g_backend_failed_str[] = "Backend connection failed";
static const char g_searching_fix_str[] = "Searching GPS...";
static const char g_fix_acquired_str[] = "GPS fix acquired";

/****************************************************************************
* Public Functions
****************************************************************************/
void connectivity_UI_received_power_key(bool bConnectivity) {
    if (bConnectivity) {
        if (connectivity_check_subscreen == CONNECTIVITY_CHECK_SUBSCREEN) {
            connectivity_check_subscreen++;
            stop_running_purpose();
            stop_gps_connectivity();
            connectivity_checking_view();
        }
        else if (connectivity_check_subscreen == CONNECTIVITY_CHECKING_SUBSCREEN) {
            connectivity_check_subscreen = CONNECTIVITY_CHECK_SUBSCREEN;
            stop_gps_connectivity();
            stop_connection_activities();
            thingsee_UI_return_to_menu();
            resume_running_purpose();
        }
    }
    else {
        if (backend_connectivity_subscreen == BACKEND_CONNECTIVITY_SUBSCREEN) {
            backend_connectivity_subscreen++;
            backend_connecting_view();
        }
        else if (backend_connectivity_subscreen == BACKEND_CONNECTING_SUBSCREEN) {
            thingsee_UI_return_to_menu();
            backend_connectivity_subscreen = BACKEND_CONNECTIVITY_SUBSCREEN;
        }
        else if (backend_connectivity_subscreen == BACKEND_UPDATED_SUBSCREEN) {
            thingsee_UI_return_to_menu();
            backend_connectivity_subscreen = BACKEND_CONNECTIVITY_SUBSCREEN;
        }
    }
}

void reset_connectivity_activities(void) {
    connectivity_check_subscreen = CONNECTIVITY_CHECK_SUBSCREEN;
    backend_connectivity_subscreen = BACKEND_CONNECTIVITY_SUBSCREEN;
}

/****************************************************************************
* Private Functions
****************************************************************************/
static void backend_connecting_view(void) {
    struct oled_tblock_s loc = {.x = SIDE_MARGIN, .y = 21, .width = OLED_SCREEN_WIDTH - SIDE_MARGIN * 2, .height = 22, CENTER_ALIGN};
    const oled_image_canvas_t *text_cancel_img;

    oled_fill_screen_with_color(CONFIG_THINGSEE_DISPLAY_BKGND_COLOR);
    text_cancel_img=UI_get_text_img(TEXT_CANCEL);
    oled_image_draw_img(OLED_SCREEN_WIDTH - text_cancel_img->width, (OLED_SCREEN_HEIGHT - text_cancel_img->height) / 2, text_cancel_img);

    oled_printf(&loc, FONTID_SANS17X22, g_connecting_str);

    if (ts_engine_backend_update(thingsee_UI_get_app_instance(), backend_update_cb) != OK)
        backend_connecting_view_updated(BACKEND_UPDATE_PING_FAILED);
}

static void backend_update_cb(enum backend_update_status status) {
    if (backend_connectivity_subscreen == BACKEND_CONNECTING_SUBSCREEN)
        backend_connecting_view_updated(status);
}

static void backend_connecting_view_updated(enum backend_update_status status) {
    struct oled_tblock_s loc = {.x = SIDE_MARGIN, .y = 0, .width = OLED_SCREEN_WIDTH - SIDE_MARGIN * 2, .height = OLED_SCREEN_HEIGHT,
        .multiline = true, .linespace = 20, .alignment = OLED_TBLOCK_CENTER_ALIGNMENT, .vert_alignment = OLED_TBLOCK_CENTER_ALIGNMENT};
    const oled_image_canvas_t *text_back_img;

    oled_fill_screen_with_color(CONFIG_THINGSEE_DISPLAY_BKGND_COLOR);
    backend_connectivity_subscreen = BACKEND_UPDATED_SUBSCREEN;

    if (status == BACKEND_UPDATE_OK)
        oled_printf(&loc, FONTID_SANS17X22, g_backend_ok_str);
    else if (status == BACKEND_UPDATE_PROFILE_UPDATED)
        oled_printf(&loc, FONTID_SANS17X22, g_backend_updated_str);
    else if (status == BACKEND_UPDATE_PING_FAILED) {
        oled_printf(&loc, FONTID_SANS17X22, g_backend_failed_str);
    }

    text_back_img=UI_get_text_img(TEXT_BACK);
    oled_image_draw_img(OLED_SCREEN_WIDTH - text_back_img->width, (OLED_SCREEN_HEIGHT - text_back_img->height) / 2, text_back_img);
}

static void connectivity_checking_view(void) {
    const oled_image_canvas_t *text_cancel_img;

    running_tests.tests = ALL_TESTS_RUNNING;
    oled_fill_screen_with_color(CONFIG_THINGSEE_DISPLAY_BKGND_COLOR);
    show_connectivity_symbols();
    text_cancel_img=UI_get_text_img(TEXT_CANCEL);
    oled_image_draw_img(OLED_SCREEN_WIDTH - text_cancel_img->width, (OLED_SCREEN_HEIGHT - text_cancel_img->height) / 2, text_cancel_img);

    update_test_result((uint8_t)WIFI_TEST, g_test_started_str);
    update_test_result((uint8_t)CELLULAR_TEST, g_test_started_str);
    update_test_result((uint8_t)BLUETOOTH_TEST, g_test_started_str);
    update_test_result((uint8_t)GPS_TEST, g_test_started_str);

    /* Test GPS */
    if (start_gps_connectivity() < 0) {
        update_test_result((uint8_t)GPS_TEST, g_test_failed_str);
        running_tests.gps_test = 0;
    }

    /* Run connectivity tests */
    g_running_test = NO_TEST;
    start_next_test();
}

static void update_test_result(uint8_t test_id, const char* result) {
    struct oled_tblock_s loc = {.x = SIDE_MARGIN, .y = TEXTLINE_Y_OFFSET, .width = OLED_SCREEN_WIDTH - SIDE_MARGIN * 2, .height = 8};

    if (connectivity_check_subscreen == CONNECTIVITY_CHECKING_SUBSCREEN || backend_connectivity_subscreen == BACKEND_CONNECTING_SUBSCREEN) {
        loc.y += (test_id - (uint8_t)WIFI_TEST) * CATEGORY_Y_OFFSET;
        oled_printf(&loc, FONTID_MONO5X8, result);
    }
}

static void show_connectivity_symbols(void) {
    const oled_image_canvas_t *cat_img;
    uint8_t offx, offy, category_offset;

    cat_img = UI_get_wlan_img();
    offx = (CATEGORY_IMAGE_SIZE-cat_img->width) / 2;
    offy = (CATEGORY_IMAGE_SIZE-cat_img->width) / 2;
    oled_image_draw_img(offx, offy, cat_img);

    category_offset = CATEGORY_Y_OFFSET;
    cat_img=UI_get_cellular_img();
    offx = (CATEGORY_IMAGE_SIZE-cat_img->width) / 2;
    offy = (CATEGORY_IMAGE_SIZE-cat_img->width) / 2;
    oled_image_draw_img(offx, category_offset + offy, cat_img);

    category_offset += CATEGORY_Y_OFFSET;
    cat_img=UI_get_bluetooth_img();
    offx = (CATEGORY_IMAGE_SIZE-cat_img->width) / 2;
    offy = (CATEGORY_IMAGE_SIZE-cat_img->width) / 2;
    oled_image_draw_img(offx, category_offset + offy, cat_img);

    category_offset += CATEGORY_Y_OFFSET;
    cat_img=UI_get_gps_img();
    offx = (CATEGORY_IMAGE_SIZE-cat_img->width) / 2;
    offy = (CATEGORY_IMAGE_SIZE-cat_img->width) / 2;
    oled_image_draw_img(offx, category_offset + offy, cat_img);
}

static int start_gps_connectivity(void) {
    int ret;

    /* Initialize GPS */
    ret = ts_gps_initialize();
    if (ret < 0)
      return ret;

    /* Register GPS callback */
    ret = ts_gps_callback_register(GPS_EVENT_TARGET_STATE_NOT_REACHED |
                                   GPS_EVENT_TARGET_STATE_REACHED |
                                   GPS_EVENT_TARGET_STATE_TIMEOUT |
                                   GPS_EVENT_STATE_CHANGE |
                                   GPS_EVENT_TIME,
                                   UI_gps_callback,
                                   NULL);
    if (ret < 0)
      return ret;

    /* Request fix */
    ret = ts_gps_request_state(GPS_STATE_FIX_ACQUIRED, GPS_FIX_TIMEOUT);

    return ret;
}

static int stop_gps_connectivity(void) {
    running_tests.gps_test = 0;
    change_button_if_done();

    (void)ts_gps_callback_unregister(UI_gps_callback);

    return ts_gps_request_state(GPS_STATE_POWER_OFF, GPS_FIX_TIMEOUT);
}

static int
gps_set_time(uint16_t const year, uint8_t const month, uint8_t const day,
             uint8_t const hour, uint8_t const min, uint8_t const sec) {
    struct timespec ts;
    struct tm t;

    /* Prepare time structure */

    t.tm_year = year - 1900;
    t.tm_mon = month - 1;
    t.tm_mday = day;
    t.tm_hour = hour;
    t.tm_min = min;
    t.tm_sec = sec;

    ts.tv_sec = mktime (&t);
    ts.tv_nsec = 0;

    /* Set clock */

    clock_settime (CLOCK_REALTIME, &ts);

    dbg ("System time set: %04d.%02d.%02d %02d:%02d:%02d.\n", year, month, day,
        hour, min, sec);

    return OK;
}

static void UI_gps_callback(void const * const e, void * const priv) {
    struct gps_event_s const * const event = e;
    struct gps_event_target_state_s const * const tevent = e;
    struct gps_event_state_change_s const * const cevent = e;
    struct gps_event_time_s const * const gps = e;

    switch (event->id) {
        /* Show progress */
        case GPS_EVENT_STATE_CHANGE:
            if (cevent->state == GPS_STATE_SEARCHING_FIX)
                if (connectivity_check_subscreen == CONNECTIVITY_CHECKING_SUBSCREEN)
                    update_test_result((uint8_t)GPS_TEST, g_searching_fix_str);
            break;

        /* Show test OK if GPS fix acquired */
        case GPS_EVENT_TARGET_STATE_REACHED:
            if (tevent->current_state == GPS_STATE_FIX_ACQUIRED) {
                if (connectivity_check_subscreen == CONNECTIVITY_CHECKING_SUBSCREEN)
                    update_test_result((uint8_t)GPS_TEST, g_fix_acquired_str);
                stop_gps_connectivity();
            }
            break;

        /* Show test failed if GPS fix not received in time */
        case GPS_EVENT_TARGET_STATE_NOT_REACHED:
        case GPS_EVENT_TARGET_STATE_TIMEOUT:
            if (tevent->target_state == GPS_STATE_FIX_ACQUIRED) {
                if (connectivity_check_subscreen == CONNECTIVITY_CHECKING_SUBSCREEN)
                    update_test_result((uint8_t)GPS_TEST, g_test_failed_str);
                stop_gps_connectivity();
            }
            break;

        case GPS_EVENT_TIME:

            /* Check if time & date is available */
            if (!gps->time->validity.time || !gps->time->validity.date)
              break;

            if (board_rtc_time_is_set(NULL)) {
                /* System time is already set */
                break;
            }

            /* Set system time */
            gps_set_time (gps->time->year, gps->time->month, gps->time->day,
              gps->time->hour, gps->time->min, gps->time->sec);

            break;

        default:
            break;
    }
}

static void stop_running_purpose(void) {
    ts_engine_stop(thingsee_UI_get_app_instance(), false, false);
}

static void resume_running_purpose(void) {
    ts_engine_continue(thingsee_UI_get_app_instance());
}

static uint8_t connect_test(connection_tests_t test) {
    int ret;

    g_running_test = test;
    ret = conman_client_init(&g_conman_client);
    if (ret == OK) {
        ret = ERROR;
        if (g_connection_test_connid != CONMAN_CONNID_CLEAR) {
            dbg("g_connection_test_connid has not been cleared!\n");
        }
        if (test == WIFI_TEST)
            ret = conman_client_request_connection(&g_conman_client, CONMAN_WIFI, &g_connection_test_connid);
        else if (test == CELLULAR_TEST)
            ret = conman_client_request_connection(&g_conman_client, CONMAN_2G, &g_connection_test_connid);

        if (ret == OK) {
            g_connection_status_timeout_timer=ts_core_timer_setup(TS_TIMER_TYPE_INTERVAL, CONNECT_STATUS_TIMEOUT, connection_status_timeout, NULL);
            g_connection_timeout_timer=ts_core_timer_setup(TS_TIMER_TYPE_TIMEOUT, CONNECTION_TIMEOUT  * 1000, connection_timeout, NULL);
        }
        else
            conman_client_uninit(&g_conman_client);
    }
    return ret;
}

static int connection_status_timeout(const int timer_id, void * const priv) {
    int ret;

    ret = conman_client_get_connection_status(&g_conman_client, &g_conman_status);
    if (ret == OK && (g_conman_status.status == CONMAN_STATUS_ESTABLISHED ||
        g_conman_status.status == CONMAN_STATUS_FAILED || g_conman_status.status == CONMAN_STATUS_DESTROYING)) {
        if (g_conman_status.status == CONMAN_STATUS_ESTABLISHED) {
            if (g_running_test == WIFI_TEST || g_running_test == CELLULAR_TEST)
                show_test_result();
            else {
                update_test_result((uint8_t)g_running_test, g_test_passed_str);
            }
            if (conman_client_destroy_connection(&g_conman_client, g_connection_test_connid) != OK)
                dbg("conman_client_destroy_connection failed\n");
            g_connection_test_connid = CONMAN_CONNID_CLEAR;
        }
        else {
            update_test_result((uint8_t)g_running_test, g_test_failed_str);
        }

        /* Stop timers */
        if (g_connection_status_timeout_timer >= 0)
            ts_core_timer_stop(g_connection_status_timeout_timer);
        g_connection_status_timeout_timer = -1;
        if (g_connection_timeout_timer >= 0)
            ts_core_timer_stop(g_connection_timeout_timer);
        g_connection_timeout_timer = -1;

        /* Mark test not running */
        if (g_running_test == WIFI_TEST)
            running_tests.wifi_test = 0;
        else
            running_tests.cellular_test = 0;

        conman_client_uninit(&g_conman_client);
        start_next_test();
    }

    return OK;
}

static int connection_timeout(const int timer_id, void * const priv) {
    /* Mark current test failed */
    update_test_result((uint8_t)g_running_test, g_test_failed_str);
    if (g_running_test == WIFI_TEST)
        running_tests.wifi_test = 0;
    else
        running_tests.cellular_test = 0;

    /* And kill the connection */
    if (conman_client_destroy_connection(&g_conman_client, g_connection_test_connid) != OK)
        dbg("conman_client_destroy_connection failed\n");
    g_connection_test_connid = CONMAN_CONNID_CLEAR;

    conman_client_uninit(&g_conman_client);

    /* And start next test */
    start_next_test();

    return OK;
}

static void show_test_result(void) {
    char *result_line;

    /* Allocate buffer for result line and create it */
    result_line = calloc(1, MAX_STATUS_LINE_LENGTH);
    if (result_line != NULL) {
        if (g_running_test == WIFI_TEST)
            snprintf(result_line, MAX_STATUS_LINE_LENGTH, "%s", g_conman_status.info.wifi.ssid_name);
        else
            snprintf(result_line, MAX_STATUS_LINE_LENGTH, "%s", g_conman_status.info.cellu.oper_name);
        if (result_line[0] != 0)
            update_test_result((uint8_t)g_running_test, result_line);
        else
            update_test_result((uint8_t)g_running_test, g_test_passed_str);
        free(result_line);
    }
}

static void start_next_test(void) {
    switch ((uint8_t)g_running_test) {
        case NO_TEST:
            g_running_test = WIFI_TEST;
            if (connect_test(WIFI_TEST) != OK) {
                update_test_result((uint8_t)g_running_test, g_test_failed_str);
                running_tests.wifi_test = 0;
                start_next_test();
            }
            break;

        case WIFI_TEST:
            g_running_test = CELLULAR_TEST;
            if (connect_test(CELLULAR_TEST) != OK) {
                update_test_result((uint8_t)g_running_test, g_test_failed_str);
                running_tests.cellular_test = 0;
                start_next_test();
            }
            break;

        case CELLULAR_TEST:
            g_running_test = BLUETOOTH_TEST;
            if (run_bluetooth_test() != OK) {
                update_test_result((uint8_t)g_running_test, g_test_failed_str);
                running_tests.bluetooth_test = 0;
                start_next_test();
            }
            break;

        case BLUETOOTH_TEST:
            g_running_test = NO_TEST;
            break;
    }

    change_button_if_done();
}

static void stop_connection_activities(void) {
    if (g_connection_status_timeout_timer >= 0) {
        ts_core_timer_stop(g_connection_status_timeout_timer);
        g_connection_status_timeout_timer = -1;
    }
    if (g_connection_timeout_timer >= 0) {
        ts_core_timer_stop(g_connection_timeout_timer);
        g_connection_timeout_timer = -1;
    }
    if (g_running_test == WIFI_TEST || g_running_test == CELLULAR_TEST) {
        if (g_connection_test_connid == CONMAN_CONNID_CLEAR)
            dbg("g_connection_test_connid not set\n");
        if (conman_client_destroy_connection(&g_conman_client, g_connection_test_connid) != OK)
            dbg("conman_client_destroy_connection failed\n");
        g_connection_test_connid = CONMAN_CONNID_CLEAR;
        conman_client_uninit(&g_conman_client);
    }
#ifdef CONFIG_THINGSEE_BLUETOOTH_MODULE
    else if (g_running_test == BLUETOOTH_TEST) {
        finish_bt_test(g_test_stopped_str);
    }
#endif
}

static void change_button_if_done(void) {
    const oled_image_canvas_t *text_back_img;

    if (running_tests.tests == 0) {
        text_back_img=UI_get_text_img(TEXT_BACK);
        oled_image_fill_rectangle(OLED_SCREEN_WIDTH - text_back_img->width, 0, text_back_img->width, OLED_SCREEN_HEIGHT, CONFIG_THINGSEE_DISPLAY_BKGND_COLOR);
        oled_image_draw_img(OLED_SCREEN_WIDTH - text_back_img->width, (OLED_SCREEN_HEIGHT - text_back_img->height) / 2, text_back_img);
    }
}

static int run_bluetooth_test(void) {
#ifdef CONFIG_THINGSEE_BLUETOOTH_MODULE
    int ret;

    /* Initialize the module and register for BT callback */
    ret = ts_bluetooth_initialize();
    DEBUGASSERT(ret == OK);
    ret = ts_bluetooth_callback_register(BT_EVENT_CONNECTION_STATE, ts_app_bt_cb, NULL);
    DEBUGASSERT(ret == OK);

    /* Start timer to detect BT connection timeout */
    g_bt_connection_timeout_timer=ts_core_timer_setup(TS_TIMER_TYPE_TIMEOUT, BT_CONNECT_TIMEOUT, bt_connection_status_timeout, NULL);
#else
    update_test_result((uint8_t)g_running_test, g_test_nottested_str);
    running_tests.bluetooth_test = 0;
    start_next_test();
#endif /* CONFIG_THINGSEE_BLUETOOTH_MODULE */

    return OK;
}

#ifdef CONFIG_THINGSEE_BLUETOOTH_MODULE
static void ts_app_bt_cb(void const * const e, void * const priv) {
    struct bluetooth_event_conn_state_s const * const conn = e;

    DEBUGASSERT(conn);

    switch (conn->event.id) {
        case BT_EVENT_CONNECTION_STATE:
            if (conn->state == BTLE_STATE_ADVERTISING) {
                /* Show test passed if we get to this state */
                finish_bt_test(g_test_passed_str);
                start_next_test();
            }
            break;

        default:
            break;
    }
}

static int bt_connection_status_timeout(const int timer_id, void * const priv) {
    finish_bt_test(g_test_failed_str);
    start_next_test();

    return OK;
}

static void finish_bt_test(const char* result) {
    /* Stop timer and free BT module */
    if (g_bt_connection_timeout_timer != 0)
        ts_core_timer_stop(g_bt_connection_timeout_timer);
    g_bt_connection_timeout_timer = 0;
    ts_bluetooth_callback_unregister(ts_app_bt_cb);
    ts_bluetooth_uninitialize();

    /* Show results and continue to next test */
    update_test_result((uint8_t)g_running_test, result);
    running_tests.bluetooth_test = 0;
}
#endif /* CONFIG_THINGSEE_BLUETOOTH_MODULE */
