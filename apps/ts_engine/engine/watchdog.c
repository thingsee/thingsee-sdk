/****************************************************************************
 * apps/ts_engine/engine/watchdog.c
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
 *   Author: Jussi Kivilinna <jussi.kivilinna@haltian.com>
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <stdio.h>

#include <nuttx/board.h>
#include <nuttx/timers/watchdog.h>
#include <arch/board/board.h>

#include <apps/thingsee/ts_core.h>

#if defined(CONFIG_WATCHDOG_LCD) && defined(CONFIG_LCD_SSD1306)
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ssd1306.h>
#endif

#include <apps/ts_engine/watchdog.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define WATCHDOG_DEVNAME "/dev/watchdog0"

/* We use maximum possible timeout here: 28.437 seconds */
#define WATCHDOG_TIMEOUT_MSEC (1000 * 0xfff / (STM32_LSI_FREQUENCY / 256))

/* 3/4 of watchdog timeout value, rounded to seconds: 21 seconds */
#define WATCHDOG_KEEPALIVE_MSEC (((WATCHDOG_TIMEOUT_MSEC / 4) / 1000) * 3 * 1000)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void wdog_setup_timer(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct {
  int fd;
  int timer_id;
#if defined(CONFIG_WATCHDOG_LCD) && defined(CONFIG_LCD_SSD1306)
  int lcd_on_cnt;
#endif
} watchdog = {
  .fd = -1,
  .timer_id = -1
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Start-up light, indication for start-up/reset/boot. */

static void light_on_startup(void)
{
#if defined(CONFIG_WATCHDOG_LCD) && defined(CONFIG_LCD_SSD1306)
  struct lcd_dev_s *lcd;

  /* Fill beginning of first line on LCD with pattern. LCD will be cleared
   * by watchdog handler. */

  lcd = board_lcd_getdev(0);
  if (lcd)
    {
      uint8_t line[8];
      struct lcd_planeinfo_s pinfo = {};

      lcd->getplaneinfo(lcd, 0, &pinfo);
      DEBUGASSERT(pinfo.putrun);
      DEBUGASSERT(pinfo.bpp >= 1);

      if (lcd->getpower(lcd) <= 0)
        {
          /* Power on display. */

          lcd->setpower(lcd, 1);
        }

      /* Clear display (black). */

      ssd1306_fill(lcd, SSD1306_Y1_BLACK);

      /* Fill beginning of first line with pattern. */

      memset(line, 0xAA, sizeof(line));
      pinfo.putrun(0, 0, line, 8 * sizeof(line) / pinfo.bpp);
    }
#endif

#ifdef CONFIG_ARCH_LEDS
  /* Turn on LED at start-up, will be turned off by watchdog handler. */

  board_setled(BOARD_LED1, true);
#endif
}

static void light_on(void)
{
#if defined(CONFIG_WATCHDOG_LCD) && defined(CONFIG_LCD_SSD1306)
  struct lcd_dev_s *lcd;

  if (++watchdog.lcd_on_cnt >= 3)
    {
      watchdog.lcd_on_cnt = 0;

      lcd = board_lcd_getdev(0);
      if (lcd)
        {
          if (lcd->getpower(lcd) <= 0)
            {
              lcd->setpower(lcd, 1);

              ssd1306_fill(lcd, SSD1306_Y1_WHITE);

              /* TODO: register short timer for turning off disable instead
               * of sleeping here. Note, remember to add deep-sleep hook
               * in addition to short timer. */
              usleep(80 * 1000);
            }
        }
    }
#endif

#ifdef CONFIG_ARCH_LEDS
  board_setled(BOARD_LED1, true);
#endif
}

static void light_off(void)
{
#if defined(CONFIG_WATCHDOG_LCD) && defined(CONFIG_LCD_SSD1306)
  board_lcdoff();
#endif

#ifdef CONFIG_ARCH_LEDS
  board_setled(BOARD_LED1, false);
#endif
}

static int wdog_timer_cb(const int timer_id, const struct timespec *date,
                         void * const priv)
{
  light_on();

  lldbg("kick the dog.\n");
  ts_watchdog_kick();
  wdog_setup_timer();

  light_off();

  return OK;
}

static void wdog_setup_timer(void)
{
  struct timespec ts = {};

  /* Setup new timer. */

  clock_gettime(CLOCK_MONOTONIC, &ts);

  ts.tv_nsec += (WATCHDOG_KEEPALIVE_MSEC % 1000) * (1000 * 1000);
  ts.tv_sec += (WATCHDOG_KEEPALIVE_MSEC / 1000);
  while (ts.tv_nsec >= 1000 * 1000 * 1000)
    {
      ts.tv_nsec -= 1000 * 1000 * 1000;
      ts.tv_sec++;
    }

  watchdog.timer_id = ts_core_timer_setup_date(&ts, wdog_timer_cb, NULL);
  DEBUGASSERT(watchdog.timer_id >= 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int ts_watchdog_kick(void)
{
  int ret = OK;

  if (watchdog.fd >= 0)
    {
      ret = ioctl(watchdog.fd, WDIOC_KEEPALIVE, 0);
      if (ret < 0)
        {
          lldbg("ioctl(WDIOC_KEEPALIVE) failed: %d\n", errno);
        }
    }
  return ret;
}

void ts_watchdog_initialize(void)
{
  int ret;

#ifdef CONFIG_ARCH_LEDS
  board_led_initialize();
#endif

  light_off();

  ret = up_wdginitialize();
  if (ret < 0)
    {
      dbg("Could not initialize watchdog HW.\n");
      return;
    }

  watchdog.fd = open(WATCHDOG_DEVNAME, O_RDONLY);
  if (watchdog.fd < 0)
    {
      dbg("Could not open %s\n", WATCHDOG_DEVNAME);
      return;
    }

  /* Setup watchdog. */

  dbg("Setting up HW watchdog with timeout of %u.%03u seconds...\n",
      WATCHDOG_TIMEOUT_MSEC / 1000,
      WATCHDOG_TIMEOUT_MSEC % 1000);
  ret = ioctl(watchdog.fd, WDIOC_SETTIMEOUT, (unsigned long)WATCHDOG_TIMEOUT_MSEC);
  if (ret < 0)
    {
      dbg("wdog_main: ioctl(WDIOC_SETTIMEOUT) failed: %d\n", errno);
      close(watchdog.fd);
      watchdog.fd = -1;
      return;
    }

  ret = ioctl(watchdog.fd, WDIOC_START, 0);
  if (ret < 0)
    {
      dbg("ioctl(WDIOC_START) failed: %d\n", errno);
      close(watchdog.fd);
      watchdog.fd = -1;
      return;
    }

  wdog_setup_timer();

  /*
   * Watchdog handler has been setup successfully, now light-up LED and/or LCD
   * for start-up/reset/boot indication.
   */

  light_on_startup();
}
