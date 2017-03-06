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

#ifndef THINGSEE_UI_H_
#define THINGSEE_UI_H_

#include <nuttx/config.h>
#include <debug.h>

#ifdef CONFIG_THINGSEE_UI_DEBUG
# define uidbg dbg
#else
# define uidbg(...) ((void)0)
#endif

void start_thingsee_UI(void (*system_do_shutdown_fn)(const char *reset_type),
                       bool force_power_on, void *app);
void thingsee_UI_set_purpose_and_state(const char * const purpose,
                                       const char * const state);
void thingsee_UI_set_state(const char * const state);
void thingsee_UI_set_charger_state(bool bCharging);
void thingsee_UI_sense_event(const char *format, ...);
void thingsee_UI_set_heat_mode(void);
void thingsee_UI_reset_device(const char *reset_type);
void thingsee_UI_set_PC_USB_connected(bool state);
bool thingsee_UI_get_PC_USB_connected(void);
struct ts_engine_app * thingsee_UI_get_app_instance(void);

#define OLED_SCREEN_WIDTH       128
#define OLED_SCREEN_HEIGHT      64
#define MAX_LINES_PER_SCREEN    7

#define CENTER_ALIGN            .alignment=OLED_TBLOCK_CENTER_ALIGNMENT
#define LEFT_ALIGN              .alignment=OLED_TBLOCK_LEFT_ALIGNMENT

typedef enum
{
  HOME_SCREEN,
  SENSES_SCREEN,
  MENU_SCREEN,
  LAST_SCREEN
} UI_screens_t;

struct thingsee_UI_data_t
{
  int screen_off_timer_id;
  int shutdown_timer_id;
  uint8_t shutdown_counter;
  int buttonfd;
  int capsensefd;
  bool bStarting;
  char *purpose;
  char *state;
  bool bLcd_on;
  bool bCharging;
  UI_screens_t displayed_screen;
  void (*system_do_shutdown_fn)(const char *reset_type);
  char *lines_on_screen[MAX_LINES_PER_SCREEN];
  bool bInHeatMode;
  struct ts_engine_app *app;
  int battery_animation_timer_id;
  bool bPCUSBConnected;
};

#endif /* THINGSEE_UI_H_ */
