/************************************************************************************
 * configs/haltian-tsone/src/up_oled.c
 * arch/arm/src/board/up_oled.c
 *
 *   Copyright (C) 2014-2015 Haltian Ltd. All rights reserved.
 *
 * Authors:
 *   Dmitry Nikolaev <dmitry.nikolaev@haltian.com>
 *   Jussi Kivilinna <jussi.kivilinna@haltian.com>
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
 ************************************************************************************/

#include "nuttx/config.h"

#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/lcd/ssd1306.h>

#include <arch/board/board.h>
#include <arch/board/board-pwrctl.h>
#include "haltian-tsone.h"
#include "stm32.h"

#ifdef CONFIG_LCD_SSD1306

/**************************************************************************************
 * Pre-processor Definitions
 **************************************************************************************/

#ifndef CONFIG_STM32_SPI2
#  error "Expecting LCD on SPI bus 2."
#endif

#define CONFIG_SSD1309_SPIBUS 2 /* TODO: move SPI bus configuration for SPI devices
                                         to haltian-tsone-bXX.h. */

#ifdef CONFIG_DEBUG_LCD
#  define lcddbg(format, ...)     dbg(format, ##__VA_ARGS__)
#  define lcdvdbg(format, ...)    vdbg(format, ##__VA_ARGS__)
#else
#  define lcddbg(x...)
#  define lcdvdbg(x...)
#endif

/**************************************************************************************
 * Private Type Definition
 **************************************************************************************/

struct oled_dev_s
{
  FAR struct lcd_dev_s *lcd;

  bool initialized:1;
  bool powered_on:1;
};

/**************************************************************************************
 * Private Function Prototypes
 **************************************************************************************/

static bool oled_set_regulator(bool on);

/**************************************************************************************
 * Private Data
 **************************************************************************************/

static struct oled_dev_s priv_oled =
{
  .lcd        = NULL,
  .powered_on = false,
  .initialized = false,
};

static const struct ssd1306_priv_s ssd1306_board_priv =
{
  .set_vcc = oled_set_regulator,
};

/**************************************************************************************
 * Private Functions
 **************************************************************************************/

static int oled_config_pins(void)
{
  return stm32_configgpio(GPIO_LCD_SSD1309_RESET);
}

static void oled_make_reset(void)
{
  /* Resets display with RESET pin. Low ---> wait for 10 us ---> High. */

  stm32_gpiowrite(GPIO_LCD_SSD1309_RESET, false);
  usleep(10);
  stm32_gpiowrite(GPIO_LCD_SSD1309_RESET, true);
}

static void oled_set_power(bool on)
{
  if (on == priv_oled.powered_on)
    return;

  if (on)
   {
      oled_make_reset();

      board_pwrctl_get(PWRCTL_REGULATOR_DISPLAY);

      /* Just in case, wait a bit for Vcc becomes stable.
       * Later probably will be removed, if HW is fast enough. */

      usleep(30);

      lcdvdbg("SSD1309 powered ON.\n");
    }
  else
    {
      board_pwrctl_put(PWRCTL_REGULATOR_DISPLAY);

      /* Wait for 100 ms as ordered in manual */

      usleep(100 * 1000);

      lcdvdbg("SSD1309 powered OFF.\n");
    }

  priv_oled.powered_on = on;
}

static bool oled_set_regulator(bool on)
{
  oled_set_power(on);
  return true;
}

/**************************************************************************************
 * Public Functions
 **************************************************************************************/

/**************************************************************************************
 * Name: board_lcd_initialize, board_lcd_getdev, board_lcd_uninitialize
 *
 * Description:
 *   If an architecture supports a parallel or serial LCD, then it must
 *   provide APIs to access the LCD as follows:
 *
 *   board_lcd_initialize   - Initialize the LCD video hardware.  The initial
 *                        state of the LCD is fully initialized, display
 *                        memory cleared, and the LCD ready to use, but with
 *                        the power setting at 0 (full off).
 *   board_lcd_getdev       - Return a a reference to the LCD object for
 *                        the specified LCD.  This allows support for
 *                        multiple LCD devices.
 *   board_lcd_uninitialize - Unitialize the LCD support
 *
 **************************************************************************************/

int board_lcd_initialize(void)
{
  FAR struct spi_dev_s *spi_oled = NULL;
  int ret;

  if (priv_oled.initialized)
    return OK;

  /* Config reset pin and others if necessary  */

  ret = oled_config_pins();
  if (ret < 0)
      return ret;

  spi_oled = up_spiinitialize(CONFIG_SSD1309_SPIBUS);
  if (!spi_oled)
      return -ENODEV;

  priv_oled.lcd = ssd1306_initialize(spi_oled, &ssd1306_board_priv, 0);
  if (!priv_oled.lcd)
    {
      lcdvdbg("SSD1309 initialization failed!\n");
      return -ENOENT;
    }

  priv_oled.initialized = true;

  return OK;
}

FAR struct lcd_dev_s *board_lcd_getdev(int lcddev)
{
  lcdvdbg("Requested oled-device.\n");

  return priv_oled.lcd;
}

void board_lcd_uninitialize(void)
{
  if (!priv_oled.initialized)
    return;

  priv_oled.lcd->setpower(priv_oled.lcd, false);

  priv_oled.initialized = false;
}

/* Board specific additions to NuttX LCD API */

void board_lcdoff(void)
{
  struct lcd_dev_s *lcd;

  lcd = priv_oled.lcd;
  if (lcd)
    {
      /* Power-off only if currently powered on. */

      if (lcd->getpower(lcd) > 0)
        lcd->setpower(lcd, 0);
    }
}

void board_lcdon(void)
{
  struct lcd_dev_s *lcd;

  lcd = priv_oled.lcd;
  if (lcd)
    {
      /* Power-on only if currently powered off. */

      if (lcd->getpower(lcd) <= 0)
        lcd->setpower(lcd, 1);
    }
}

#endif
