/****************************************************************************
 * Copyright (C) 2014-2016 Haltian Ltd. All rights reserved.
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
#include <nuttx/time.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>

#include <arch/board/board.h>
#include <arch/board/board-reset.h>
#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nxfonts.h>
#include <arch/board/board-adc.h>
#include <arch/board/board.h>
#include <apps/thingsee/ts_core.h>
#include "../../../apps/system/display/oled_display.h"
#include "../../../apps/system/display/oled_text_block.h"
#include "../../../apps/system/display/oled_image.h"
#include <nuttx/input/cypress_mbr3108.h>
#include "../charger/bq24251_module.h"

#include <thingsee_ui.h>
#include <ui_bitmaps.h>
#include <ui_battery.h>
#include <ui_connectivity.h>
#include <ui_menu.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#define SCREEN_OFF_TIMEOUT          20     /* Seconds */
#ifdef UI_SHOW_STARTUP_ANIMATION
#define STARTUP_TIMEOUT             5      /* Seconds */
#else
#define STARTUP_TIMEOUT             2      /* Seconds */
#endif
#define SHUTDOWN_TIMEOUT            3      /* Seconds */
#define UI_ELEMENT_INTERVAL         5      /* Pixels */
#define MAX_SCREEN_LINE_LENGTH      40     /* Characters */
#define CHARGING_ANIMATION_STEPS    25     /* Percentage */
#define CHARGING_ANIMATION_EMPTY    0      /* Percentage */
#define CHARGING_ANIMATION_FULL     100    /* Percentage */
#define CHARGING_ANIMATION_TIMEOUT  1000   /* Milliseconds */
#define CHARGING_SYMBOL_LEVEL       48     /* Pixels */

/****************************************************************************
 * Private Function Prototypes
 *****************************************************************************/

static void UI_button_initialize(void);
static void UI_button_uninitialize(void);
static void UI_capsense_initialize(void);
static void UI_capsense_uninitialize(void);
static void stop_screen_off_timer(void);
static void start_screen_off_timer(void);
static void stop_shutdown_timer(void);
static void start_shutdown_timer(void);
static void show_UI(void);
static void hide_UI(void);
static bool get_power_key(void);
static void shutdown_device(void);
static void show_purpose_and_state(void);
static uint8_t get_center_x(int length);
static void show_shutdown_boxes(uint8_t boxes, bool bShutdown, uint8_t timeout_length);
static void show_battery_and_charging(void);
static void move_lines_up(char *new_line);
static void print_sense_lines(void);
static bool is_sense_empty(void);
static void UI_draw_init_screen(void);

/* Callbacks */
static bool UI_deepsleep_hook(void * const priv);
static int UI_button_handler(const struct pollfd * const pfd, void * const arg);
static int UI_capsense_callback(const struct pollfd * const pfd, void * const priv);
static int UI_screen_off_timeout(const int timer_id, const struct timespec *date, void * const priv);
static int shutdown_timeout(const int timer_id, const struct timespec *date, void * const priv);
static int battery_animation_timeout(const int timer_id, void * const priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

typedef enum
{
  NONE,
  RIGHT,
  LEFT
} capsense_buttons_t;

static struct thingsee_UI_data_t thingsee_UI_data =
{
  .screen_off_timer_id = -1,
  .shutdown_timer_id = -1,
  .shutdown_counter = 0,
  .buttonfd = -1,
  .capsensefd = -1,
  .bStarting = true,
  .purpose = NULL,
  .state = NULL,
  .bLcd_on = false,
  .bCharging = false,
  .system_do_shutdown_fn = NULL,
  .displayed_screen = HOME_SCREEN,
  .bInHeatMode = false,
  .app = NULL,
  .battery_animation_timer_id = -1,
  .bPCUSBConnected = false,
};

static uint32_t g_reset_reason = 0;
static bool g_is_initialization = false;
static int init_screen_timer = -1;

/****************************************************************************
 * UI strings
 ****************************************************************************/

#ifdef UI_SHOW_STARTUP_ANIMATION
static const char g_starting_str[] = "Starting";
#endif
static const char g_heatmode_str[] = "In Heat mode";
static const char g_empty_str[] = "";
static const char g_shutdown_str[] = "Shutting down";
static const char g_no_sense_str[] = "- No sense data -";

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void start_thingsee_UI(void (*system_do_shutdown_fn)(const char *reset_type), void *app)
{
  int timeout = 0, i;
  bool button;

  uidbg("\n");

#ifdef UI_SHOW_STARTUP_ANIMATION
  struct oled_tblock_s loc =
  {
    .x = 0,
    .y = 10,
    .width = OLED_SCREEN_WIDTH,
    .height = 22,
    CENTER_ALIGN
  };
#endif

  thingsee_UI_data.system_do_shutdown_fn = system_do_shutdown_fn;
  thingsee_UI_data.app = (struct ts_engine_app *) app;

  /* Set power button handler */

  UI_button_initialize();

  /* If device was put to sleep purposefully, only allow wakeup by power key */

  g_reset_reason = board_reset_get_reason(true);
  if (g_reset_reason & BOARD_RESET_REASON_STANDBY_WAKEUP)
    {
      while (++timeout <= STARTUP_TIMEOUT)
        {
          button = get_power_key();
          if (!button)
            {
              /* If button has been released */

              shutdown_device();
            }
          else
            {
#ifdef UI_SHOW_STARTUP_ANIMATION
              oled_printf(&loc, FONTID_SANS17X22, g_starting_str);
              show_shutdown_boxes(timeout, false, STARTUP_TIMEOUT);
#endif
              sleep(1);
            }
        }
    }

  for (i = 0; i < MAX_LINES_PER_SCREEN; i++)
    {
      thingsee_UI_data.lines_on_screen[i] = calloc(1, MAX_SCREEN_LINE_LENGTH);
    }
  thingsee_UI_data.bStarting = false;
  g_is_initialization = true;

  UI_draw_init_screen();
}

void thingsee_UI_set_purpose_and_state(const char * const purpose,
                                       const char * const state)
{
  uidbg("purpose: %s, state: %s\n", purpose, state);

  if (thingsee_UI_data.purpose != NULL)
    free(thingsee_UI_data.purpose);
  thingsee_UI_data.purpose = strdup(purpose);

  thingsee_UI_set_state(state);
}

void thingsee_UI_set_state(const char * const state)
{
  uidbg("state: %s\n", state);

  /* Copy stuff anyway. But do not show it if no one wants to see it */

  if (thingsee_UI_data.state != NULL)
    free(thingsee_UI_data.state);
  thingsee_UI_data.state = strdup(state);

  if (!thingsee_UI_data.bInHeatMode && !g_is_initialization)
    {
      show_purpose_and_state();
    }
}

void thingsee_UI_set_charger_state(bool bCharging)
{
  uidbg("charging: %d\n", bCharging);

  thingsee_UI_data.bCharging = bCharging;

  /* Check if we still showing initialization screen */

  if (!g_is_initialization)
    {
      if (bCharging && thingsee_UI_data.battery_animation_timer_id == -1)
        thingsee_UI_data.battery_animation_timer_id = ts_core_timer_setup(
            TS_TIMER_TYPE_INTERVAL, CHARGING_ANIMATION_TIMEOUT,
            battery_animation_timeout, NULL);
      else if (!bCharging && thingsee_UI_data.battery_animation_timer_id != -1)
        {
          ts_core_timer_stop(thingsee_UI_data.battery_animation_timer_id);
          thingsee_UI_data.battery_animation_timer_id = -1;
        }

      /* Whenever charger connection changes, and if not in connectivity subscreens,
       * go to main screen and show UI */

      if (is_screen_change_allowed())
        {
          thingsee_UI_data.displayed_screen = HOME_SCREEN;
          show_UI();
        }
    }
}

void thingsee_UI_sense_event(const char *format, ...)
{
  va_list arglist;
  char *new_line;

  uidbg("\n");

  va_start(arglist, format);

  /* Allocate buffer for new line and store it */

  new_line = calloc(1, MAX_SCREEN_LINE_LENGTH);
  if (new_line != NULL)
    {
      vsnprintf(new_line, MAX_SCREEN_LINE_LENGTH, format, arglist);
      move_lines_up(new_line);

      /* Print out the sense data if displaying senses screen */

      if (thingsee_UI_data.displayed_screen == SENSES_SCREEN && thingsee_UI_data.bLcd_on
          && thingsee_UI_data.shutdown_counter == 0)
        {
          oled_dbg_printf(new_line);
        }
    }
  va_end(arglist);
}

void thingsee_UI_set_heat_mode(void)
{
  uidbg("\n");

  thingsee_UI_set_purpose_and_state(g_heatmode_str, g_empty_str);
  thingsee_UI_data.bInHeatMode = true;
}

struct ts_engine_app * thingsee_UI_get_app_instance(void)
{
  return thingsee_UI_data.app;
}

void thingsee_UI_reset_device(const char *reset_type)
{
  uidbg("type: %s\n", reset_type);

  /* Disable Capsense input (prevent UI from messing shutdown process) */

  UI_capsense_uninitialize();

  /* Also disable button input. */

  UI_button_uninitialize();

  /* Prevent timer messing shutdown. */

  stop_screen_off_timer();

  /* Start shutdown. */

  ASSERT(thingsee_UI_data.system_do_shutdown_fn);
  thingsee_UI_data.system_do_shutdown_fn(reset_type);
}

void thingsee_UI_set_PC_USB_connected(bool state)
{
  uidbg("state: %d\n", state);
  thingsee_UI_data.bPCUSBConnected = state;
}

bool thingsee_UI_get_PC_USB_connected(void)
{
  return thingsee_UI_data.bPCUSBConnected;
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void UI_load_ui(void)
{
  uidbg("\n");

  /* Because for some reasons, at this point neither semaphores nor timers
   * work, we have to split initialization in 2 parts and do not care about
   * consecvenses and nonsenses of adding a new feature in the end of the
   * project */

#ifdef UI_SHOW_STARTUP_ANIMATION
  struct oled_tblock_s loc =
  {
    .x = 0,
    .y = 10,
    .width = OLED_SCREEN_WIDTH,
    .height = 22,
    CENTER_ALIGN
  };
#endif

  g_is_initialization = false;

  UI_capsense_initialize();

  /* Clear text and show UI */
#ifdef UI_SHOW_STARTUP_ANIMATION
  oled_printf(&loc, FONTID_SANS17X22, g_empty_str);
#endif
  show_UI();
}

static void UI_init_screen_shutdown_timer(void)
{
  uidbg("\n");

  if (init_screen_timer >= 0)
    {
      ts_core_timer_stop(init_screen_timer);
      init_screen_timer = -1;
    }
}

static void UI_init_screen_remove(void)
{
  uidbg("\n");

  UI_init_screen_shutdown_timer();
  oled_fill_screen_with_color(CONFIG_THINGSEE_DISPLAY_BKGND_COLOR);
  UI_load_ui();
}

static int UI_init_screen_timeout(const int timer_id,
                                  const struct timespec *date,
                                  void * const priv)
{
  uidbg("\n");

  UI_init_screen_remove();
  return OK;
}

static void UI_draw_init_screen(void)
{
  struct timespec ts = { };

  uidbg("\n");

  oled_image_draw_img(0, 0, UI_get_init_screen_img());

  clock_gettime(CLOCK_MONOTONIC, &ts);

  /* Wait for 5 seconds */

  ts.tv_sec += 5;
  init_screen_timer = ts_core_timer_setup_date(&ts, UI_init_screen_timeout, NULL);
}

static void shutdown_device(void)
{
  uidbg("\n");

  thingsee_UI_reset_device("standby");
}

static bool UI_deepsleep_hook(void * const priv)
{
  return false ;
}

static int UI_screen_off_timeout(const int timer_id, const struct timespec *date, void * const priv)
{
  uidbg("\n");

  if (is_screen_change_allowed())
    hide_UI();

  return OK;
}

static int shutdown_timeout(const int timer_id, const struct timespec *date, void * const priv)
{
  struct oled_tblock_s loc =
  {
    .x = 0,
    .y = 10,
    .width = OLED_SCREEN_WIDTH,
    .height = 22,
    CENTER_ALIGN
  };
  bool key = get_power_key();

  uidbg("key: %d\n", key);

  /* If key has been released but event has disappeared stop the shutdown */

  if (!key)
    {
      return OK;
    }

  thingsee_UI_data.shutdown_counter++;
  board_lcdon();
  thingsee_UI_data.bLcd_on = true;

  /* Display shutdown information */

  if (thingsee_UI_data.shutdown_counter == 1)
    {
      /* If starting shutdown set deepsleep hook to enable proper timer running */

      ts_core_deepsleep_hook_remove(UI_deepsleep_hook);
      ts_core_deepsleep_hook_add(UI_deepsleep_hook, NULL);

      /* And clear the display */

      oled_fill_screen_with_color(CONFIG_THINGSEE_DISPLAY_BKGND_COLOR);
      oled_printf(&loc, FONTID_SANS17X22, g_shutdown_str);
    }

  show_shutdown_boxes(thingsee_UI_data.shutdown_counter, true, SHUTDOWN_TIMEOUT);

  /* Shutdown if 5 seconds */

  if (thingsee_UI_data.shutdown_counter == SHUTDOWN_TIMEOUT)
    {
      shutdown_device();
    }
  else
    {
      /* Restart timer */

      thingsee_UI_data.shutdown_timer_id = -1;
      start_shutdown_timer();
    }

  return OK;
}

static void show_shutdown_boxes(uint8_t boxes, bool bShutdown, uint8_t timeout_length)
{
  const oled_image_canvas_t *empty_box_img, *full_box_img;
  int i, start, width, pos;

  uidbg("\n");

  empty_box_img = UI_get_box_img(false);
  full_box_img = UI_get_box_img(true);

  width = empty_box_img->width * timeout_length
      + UI_ELEMENT_INTERVAL * (timeout_length - 1);
  start = get_center_x(width);
  for (i = 0; i < timeout_length; i++)
    {
      pos = start + (empty_box_img->width + UI_ELEMENT_INTERVAL) * i;
      if (bShutdown ^ (i < boxes))
        oled_image_draw_img(pos, 32, full_box_img);
      else
        oled_image_draw_img(pos, 32, empty_box_img);
    }
}

static int battery_animation_timeout(const int timer_id, void * const priv)
{
  static uint8_t percentage = 0;
  uint8_t pos;
  const oled_image_canvas_t *battery_img, *charging_img;

  uidbg("\n");

  if (thingsee_UI_data.displayed_screen == HOME_SCREEN)
    {
      percentage += CHARGING_ANIMATION_STEPS;
      if (percentage > CHARGING_ANIMATION_FULL)
        percentage = CHARGING_ANIMATION_EMPTY;

      battery_img = UI_get_battery_img(percentage);
      pos = (OLED_SCREEN_WIDTH - battery_img->width) / 2;
      if (thingsee_UI_data.bCharging)
        {
          charging_img = UI_get_charging_img();
          pos -= (charging_img->width + UI_ELEMENT_INTERVAL) / 2;
          oled_image_draw_img(pos + battery_img->width + UI_ELEMENT_INTERVAL,
                              CHARGING_SYMBOL_LEVEL,
                              charging_img);
        }
      oled_image_draw_img(pos, CHARGING_SYMBOL_LEVEL, battery_img);

      /* Check for the charger IC status */

      if (!bq24251_is_charging())
        {
          thingsee_UI_set_charger_state(false);
        }
    }

  return OK;
}

static void UI_button_initialize(void)
{
  uidbg("\n");

  thingsee_UI_data.buttonfd = open("/dev/buttons0", O_RDONLY);
  if (thingsee_UI_data.buttonfd >= 0)
    {
      ts_core_fd_register(thingsee_UI_data.buttonfd, POLLIN, &UI_button_handler,
                          NULL);
    }
  else
    {
      dbg("Power button initialization for UI failed.\n");
    }
}

static void UI_button_uninitialize(void)
{
  uidbg("buttonfd: %d\n", thingsee_UI_data.buttonfd);

  if (thingsee_UI_data.buttonfd >= 0)
    {
      ts_core_fd_unregister(thingsee_UI_data.buttonfd);
      close(thingsee_UI_data.buttonfd);
      thingsee_UI_data.buttonfd = -1;
    }
}

static int UI_button_handler(const struct pollfd * const pfd, void * const arg)
{
  bool power_key_pressed;

  uidbg("starting: %d\n", thingsee_UI_data.bStarting);

  /* Exit this handling immediately if device is starting */

  if (thingsee_UI_data.bStarting)
    return OK;

  power_key_pressed = get_power_key();

  if (power_key_pressed)
    {
      /* React on press */

      thingsee_UI_data.shutdown_counter = 0;
      if (g_is_initialization)
        {
          UI_init_screen_remove();
          return OK;
        }

      if (!thingsee_UI_data.bLcd_on)
        {
          show_UI();
        }
      else
        {
          if (thingsee_UI_data.displayed_screen == HOME_SCREEN ||
              thingsee_UI_data.displayed_screen == SENSES_SCREEN)
            {
              hide_UI();
            }
          else
            {
              /* Don't hide but restart screen off timer */

              start_screen_off_timer();

              /* Send event to menu if visible */

              if (thingsee_UI_data.displayed_screen == MENU_SCREEN)
                thingsee_UI_menu_received_power_key();
            }
        }
      start_shutdown_timer();
    }
  else
    { /* Stop shutdown on release */

      stop_shutdown_timer();

      /* If partially progressed shutdown, return to active UI state */

      if (thingsee_UI_data.shutdown_counter != 0)
        {
          thingsee_UI_data.shutdown_counter = 0;
          show_UI();
        }
    }

  return OK;
}

static bool get_power_key(void)
{
  ssize_t ret;
  uint8_t button_flags;
  bool retval = false;

  ret = read(thingsee_UI_data.buttonfd, &button_flags, sizeof(button_flags));
  if (ret == sizeof(button_flags))
    {
      retval = !!(button_flags & BOARD_BUTTON_POWERKEY_BIT);
    }

  uidbg("retval: %d\n", retval);

  return retval;
}

static void UI_capsense_initialize(void)
{
  uidbg("\n");

  if (thingsee_UI_data.capsensefd >= 0)
    return; /* Already initialized. */

  thingsee_UI_data.capsensefd = open("/dev/capsense0", O_RDWR);
  if (thingsee_UI_data.capsensefd >= 0)
    ts_core_fd_register(thingsee_UI_data.capsensefd, POLLIN,
                        UI_capsense_callback, NULL);
  else
    dbg("Capsense initialization for UI failed.\n");
}

static void UI_capsense_uninitialize(void)
{
  uidbg("capsensefd: %d\n", thingsee_UI_data.capsensefd);

  if (thingsee_UI_data.capsensefd >= 0)
    {
      ts_core_fd_unregister(thingsee_UI_data.capsensefd);
      close(thingsee_UI_data.capsensefd);
      thingsee_UI_data.capsensefd = -1;
    }
}

static int UI_capsense_callback(const struct pollfd * const pfd,
                                void * const priv)
{
  struct mbr3108_sensor_status_s data;
  static capsense_buttons_t prev_button = NONE;

  uidbg("\n");

  if (read(pfd->fd, &data, sizeof(data)) < 0)
    {
      dbg("Capsense read for UI failed.\n");
      return ERROR;
    }

  /* Return if the screen is not on */
  if (!thingsee_UI_data.bLcd_on)
    return OK;

  if ((data.button & LEFT) == NONE)
    {
      /* React on button release */

      if (prev_button == LEFT && is_screen_change_allowed())
        {
          if (++thingsee_UI_data.displayed_screen == LAST_SCREEN)
            thingsee_UI_data.displayed_screen = HOME_SCREEN;
          show_UI();
        }
      else if (prev_button == LEFT && thingsee_UI_data.displayed_screen == MENU_SCREEN)
        {
          /* Send event to menu if visible */

          thingsee_UI_menu_received_capsense_key();
        }
    }
  prev_button = (data.button & LEFT);

  /* If there has been any button event restart the screen of timer */

  if (data.button)
    start_screen_off_timer();

  return OK;
}

static void stop_screen_off_timer(void)
{
  uidbg("\n");

  if (thingsee_UI_data.screen_off_timer_id >= 0)
    {
      ts_core_timer_stop(thingsee_UI_data.screen_off_timer_id);
      thingsee_UI_data.screen_off_timer_id = -1;
    }
}

static void start_screen_off_timer(void)
{
  struct timespec ts = { };

  uidbg("\n");

  stop_screen_off_timer();

  clock_gettime(CLOCK_MONOTONIC, &ts);
  ts.tv_sec += SCREEN_OFF_TIMEOUT;
  thingsee_UI_data.screen_off_timer_id = ts_core_timer_setup_date(
      &ts, UI_screen_off_timeout, NULL);
}

static void stop_shutdown_timer(void)
{
  uidbg("id: %d\n", thingsee_UI_data.shutdown_timer_id);

  if (thingsee_UI_data.shutdown_timer_id >= 0)
    {
      ts_core_timer_stop(thingsee_UI_data.shutdown_timer_id);
      thingsee_UI_data.shutdown_timer_id = -1;
    }
}

static void start_shutdown_timer(void)
{
  struct timespec ts = { };

  uidbg("\n");

  stop_shutdown_timer();

  clock_gettime(CLOCK_MONOTONIC, &ts);
  ts.tv_sec += 1;
  thingsee_UI_data.shutdown_timer_id = ts_core_timer_setup_date(&ts, shutdown_timeout, NULL);
}

static void show_UI(void)
{
  uidbg("\n");

  /* Disable deep sleep */

  ts_core_deepsleep_hook_remove(UI_deepsleep_hook);
  ts_core_deepsleep_hook_add(UI_deepsleep_hook, NULL);

  board_lcdon(); /* Switch screen on */
  thingsee_UI_data.bLcd_on = true;
  UI_capsense_initialize(); /* Enable Capsense for input */

  oled_fill_screen_with_color(CONFIG_THINGSEE_DISPLAY_BKGND_COLOR);

  /* Start charging animation if not already running */

  if (thingsee_UI_data.bCharging && thingsee_UI_data.battery_animation_timer_id == -1)
    thingsee_UI_data.battery_animation_timer_id = ts_core_timer_setup(
        TS_TIMER_TYPE_INTERVAL, CHARGING_ANIMATION_TIMEOUT,
        battery_animation_timeout, NULL);

  switch (thingsee_UI_data.displayed_screen)
    {
    case HOME_SCREEN:
      /* Show data */

      show_purpose_and_state();
      show_battery_and_charging();
      reset_connectivity_activities();
      break;

    case SENSES_SCREEN:
      /* Show stored sense data lines */

      print_sense_lines();
      break;

    case MENU_SCREEN:
      reset_connectivity_activities();
      thingsee_UI_show_menu();
      break;

    default:
      break;
    }

  /* Setup timer */

  start_screen_off_timer();
}

static void show_battery_and_charging(void)
{
  int percentage, pos;
  const oled_image_canvas_t *battery_img, *charging_img;

  uidbg("\n");

  percentage = UI_get_battery_percentage();
  battery_img = UI_get_battery_img(percentage);
  pos = (OLED_SCREEN_WIDTH - battery_img->width) / 2;
  if (thingsee_UI_data.bCharging)
    {
      charging_img = UI_get_charging_img();
      pos -= (charging_img->width + UI_ELEMENT_INTERVAL) / 2;
      oled_image_draw_img(pos + battery_img->width + UI_ELEMENT_INTERVAL,
                          CHARGING_SYMBOL_LEVEL,
                          charging_img);
    }
  oled_image_draw_img(pos, CHARGING_SYMBOL_LEVEL, battery_img);
}

static void show_purpose_and_state(void)
{
  struct oled_tblock_s loc_1 =
  {
    .x = 0,
    .y = 0,
    .width = OLED_SCREEN_WIDTH,
    .height = 22,
    CENTER_ALIGN
  };
  struct oled_tblock_s loc_2 =
  {
    .x = 0,
    .y = 22,
    .width = OLED_SCREEN_WIDTH,
    .height = 22,
    CENTER_ALIGN
  };

  uidbg("\n");

  /* Don't show if shutting down or not on home screen */

  if (thingsee_UI_data.shutdown_counter == 0 && thingsee_UI_data.displayed_screen == HOME_SCREEN)
    {
      if (thingsee_UI_data.purpose != NULL)
        {
          oled_printf(&loc_1, FONTID_SANS17X22, thingsee_UI_data.purpose);
        }

      if (thingsee_UI_data.state != NULL)
        {
          oled_printf(&loc_2, FONTID_SANS17X22, thingsee_UI_data.state);
        }
    }
}

static void hide_UI(void)
{
  uidbg("\n");

  UI_capsense_uninitialize(); /* Disable Capsense to save power */
  board_lcdoff(); /* Turn screen off */
  thingsee_UI_data.bLcd_on = false;

  /* Stop battery animation if it is ongoing */

  if (thingsee_UI_data.battery_animation_timer_id != -1)
    {
      ts_core_timer_stop(thingsee_UI_data.battery_animation_timer_id);
      thingsee_UI_data.battery_animation_timer_id = -1;
    }

  stop_screen_off_timer();

  /* Remove deep sleep hook */

  ts_core_deepsleep_hook_remove(UI_deepsleep_hook);
}

static uint8_t get_center_x(int length)
{
  return (length > OLED_SCREEN_WIDTH) ? 0 : (OLED_SCREEN_WIDTH - length) / 2;
}

static void move_lines_up(char *new_line)
{
  int i;

  /* Free topmost line*/

  free(thingsee_UI_data.lines_on_screen[0]);

  /* Move lines up by one */

  for (i = 0; i < MAX_LINES_PER_SCREEN - 1; i++)
    thingsee_UI_data.lines_on_screen[i] = thingsee_UI_data.lines_on_screen[i + 1];

  /* Add new line to bottom */

  thingsee_UI_data.lines_on_screen[i] = new_line;
}

static void print_sense_lines(void)
{
  int i;
  struct oled_tblock_s loc =
  {
    .x = 0,
    .y = 0,
    .width = OLED_SCREEN_WIDTH,
    .height = 8,
    CENTER_ALIGN
  };

  oled_clear_dbg_screen();

  for (i = 0; i < MAX_LINES_PER_SCREEN; i++)
    {
      if (thingsee_UI_data.lines_on_screen[i] != NULL &&
          thingsee_UI_data.lines_on_screen[i][0] != 0)
        {
          oled_dbg_printf(thingsee_UI_data.lines_on_screen[i]);
        }
    }

  if (is_sense_empty())
    oled_printf(&loc, FONTID_MONO5X8, g_no_sense_str);
}

static bool is_sense_empty(void)
{
  int i;

  for (i = 0; i < MAX_LINES_PER_SCREEN; i++)
    {
      if (thingsee_UI_data.lines_on_screen[i] != NULL &&
          thingsee_UI_data.lines_on_screen[i][0] != 0)
        {
          return false;
        }
    }

  return true;
}
