/****************************************************************************
 * configs/haltian-tsone/src/up_memlcd.c
 *
 *   Copyright (C) 2014-2015 Haltian Ltd. All rights reserved.
 *   Authors: Jussi Kivilinna <jussi.kivilinna@haltian.com>
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <semaphore.h>
#include <poll.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/ascii.h>
#include <nuttx/streams.h>
#include <nuttx/fs/fs.h>
#include <nuttx/lcd/slcd_ioctl.h>
#include <nuttx/lcd/slcd_codec.h>
#include <nuttx/pwm.h>
#include <arch/board/board.h>
#include <arch/board/board-pwrctl.h>
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/memlcd.h>

#include "chip.h"
#include "up_arch.h"
#include "stm32_pwm.h"
#include "stm32_gpio.h"
#include "stm32_rcc.h"

#include "haltian-tsone.h"

#ifdef CONFIG_LCD_SHARP_MEMLCD

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_GRAPHICS
#  undef CONFIG_DEBUG_LCD
#endif

#ifndef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_LCD
#endif

#ifndef CONFIG_STM32_TIM2_PWM
#  error "TIM2 in PWM mode required for EXTVCOM signal"
#endif

#ifndef CONFIG_MEMLCD_EXTCOMIN_MODE_HW
#  error "Only EXTCOMIN mode supported"
#endif

#ifndef CONFIG_LCD_SHARP_MEMLCD_SPI_BUS
#  define CONFIG_LCD_SHARP_MEMLCD_SPI_BUS 3
#endif

#define MAX_VCOM_FREQ 10
#define MIN_VCOM_FREQ 1 /* This should be 0.5 Hz, but 1 Hz is smallest
                           supported by NuttX. */

/* The ever-present MIN/MAX macros ******************************************/

#ifndef MIN
#  define MIN(a,b) (a < b ? a : b)
#endif

#ifndef MAX
#  define MAX(a,b) (a > b ? a : b)
#endif

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_LCD
#  define lcddbg              dbg
#  define lcdvdbg             vdbg
#else
#  define lcddbg(x...)
#  define lcdvdbg(x...)
#endif

/****************************************************************************
 * Private Type Definition
 ****************************************************************************/

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct
{
  FAR struct lcd_dev_s *dev;
  FAR struct pwm_lowerhalf_s *pwm;

  uint8_t extvcomfreq;
  bool enabled;
  bool initialized;
} memlcd;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void memlcd_control_extvcom(bool on)
{
  int ret;

  if (on)
    {
      struct pwm_info_s pwm_info;

      /* Start-up TIM peripheral as PWM. */

      ret = memlcd.pwm->ops->setup(memlcd.pwm);
      DEBUGASSERT(ret == OK);

      /*
       * Setup PWM for 1-10 hz frequency and with 1/0xffff duty cycle.
       *
       * EXTVCOM signal width must more more than 1 µsec.
       *
       * With 10 Hz, we have:
       *  1000000 µsec / 10 / 0xffff * 1 ≃ 1.5 µsec
       * With 1 Hz:
       *  1000000 µsec / 1 / 0xffff * 1 ≃ 15 µsec
       *
       */

      pwm_info.frequency = memlcd.extvcomfreq;
      pwm_info.duty = 1;

      ret = memlcd.pwm->ops->start(memlcd.pwm, &pwm_info);
      DEBUGASSERT(ret == OK);
    }
  else
    {
      ret = memlcd.pwm->ops->stop(memlcd.pwm);
      DEBUGASSERT(ret == OK);

      /* Stop peripheral completely for power-save. */

      ret = memlcd.pwm->ops->shutdown(memlcd.pwm);
      DEBUGASSERT(ret == OK);

      /* Place GPIO in to the default pull-down state. */

      (void)stm32_configgpio(GPIO_TIM2_CH4OUT);
    }
}

static bool memlcd_setup_extvcom(void)
{
  /* Set default refresh frequency to 1 hz. */

  memlcd.extvcomfreq = MIN_VCOM_FREQ;

  /* Initialize PWM hardware. */

  memlcd.pwm = stm32_pwminitialize(PWM_MEMLCD_EXTVCOM);
  if (!memlcd.pwm)
    {
      idbg("Failed to initialize PWM for EXTVCOM\n");
      return false;
    }

  return true;
}

static void up_lcddispcontrol(bool on)
{
  lcddbg("set: %s\n", on ? "on" : "off");

  if (on && !memlcd.enabled)
    {
      memlcd.enabled = true;

      /* Turn on display regulator. */

      board_pwrctl_get(PWRCTL_REGULATOR_DISPLAY);

      /* Set DISP pin high. */

      stm32_gpiowrite(GPIO_MEMLCD_DISP, true);

      /* Enable PWM. */

      memlcd_control_extvcom(true);
    }
  else if (!on && memlcd.enabled)
    {
      memlcd.enabled = false;
      /* Set DISP pin low. */

      stm32_gpiowrite(GPIO_MEMLCD_DISP, false);

      /* Disable PWM. */

      memlcd_control_extvcom(false);

      /* Turn off display regulator. */

      board_pwrctl_put(PWRCTL_REGULATOR_DISPLAY);
    }
}

static void up_lcdsetvcomfreq(unsigned int freq)
{
  lcddbg("freq: %d\n", freq);

  DEBUGASSERT(freq >= MIN_VCOM_FREQ && freq <= MAX_VCOM_FREQ);

  /* Exit early if no change is required. */

  if (freq == memlcd.extvcomfreq)
    {
      return;
    }

  /* Set PWM frequency. */

  memlcd.extvcomfreq = freq;

  if (memlcd.enabled)
    {
      memlcd_control_extvcom(false);
      memlcd_control_extvcom(true);
    }
}

static FAR struct memlcd_priv_s memlcd_priv =
{
  .dispcontrol = up_lcddispcontrol,
  .setvcomfreq = up_lcdsetvcomfreq,
};

#if 0
#define TEST_MEMLCD 1
static void test_memlcd(struct lcd_dev_s *lcd, const char *devname)
{
  static const uint8_t pixel_lines[] =
  {
     0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00,
     0xff, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0xff,
     0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
     0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,

     0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00,
     0xff, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0xff,
     0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
     0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,

     0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00,
     0xff, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0xff,
     0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
     0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,

     0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00,
     0xff, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0xff,
     0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
     0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,

     0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00,
     0xff, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0xff,
     0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
     0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,

     0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00,
     0xff, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0xff,
     0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
     0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,

     0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00,
     0xff, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0xff,
     0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
     0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,

     0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00,
     0xff, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0xff,
     0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
     0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,
  };
  struct memlcd_pixellines_s draw_pixels;
  struct memlcd_dimensions_s disp_dim;
  FILE *fp;
  int i;

  fp = fopen(devname, "wb");
  DEBUGASSERT(fp != NULL);

  /* Turn on power, clear screen. */

  ioctl(fp->fs_fd, MEMLCD_IOC_SETPOWER, true);

  /* Write lines to screen. */

  for (i = 1; i <= 20; i++)
    {
      fprintf(fp, "count: %d\n", i);
    }

  /* Write all characters to screen. */

  for (i = 0; i < 256; i++)
    {
      uint8_t c = i;
      fwrite(&c, 1, 1, fp);
    }

  /* Get device dimensions. */

  ioctl(fp->fs_fd, MEMLCD_IOC_GETDIMENSIONS, (intptr_t)&disp_dim);

  /* Write random characters to random positions on screen. */

#if 0
  for (i = 0; i < 1024; i++)
#else
  for (i = 0; i < 8; i++)
#endif
    {
      struct memlcd_cursor_s cursor;
      char rnd_char = (rand() % (126 - 32)) + 32;
      int rnd_row = rand() % disp_dim.cons_rows;
      int rnd_col = rand() % disp_dim.cons_cols;

      cursor.colpos = rnd_col;
      cursor.rowpos = rnd_row;

      ioctl(fp->fs_fd, MEMLCD_IOC_SETCURSOR, (intptr_t)&cursor);
      write(fp->fs_fd, &rnd_char, 1);
    }

  /* Draw raw pixels. */

  draw_pixels.first_line = 40;
  draw_pixels.num_lines = (sizeof(pixel_lines) * 8) / disp_dim.width_pixels;
  draw_pixels.pixelbuflen = draw_pixels.num_lines * disp_dim.width_pixels / 8;
  draw_pixels.pixels = &pixel_lines;
  ioctl(fp->fs_fd, MEMLCD_IOC_DRAWPIXELLINES, (intptr_t)&draw_pixels);

  draw_pixels.first_line = 123;
  ioctl(fp->fs_fd, MEMLCD_IOC_DRAWPIXELLINES, (intptr_t)&draw_pixels);

  /* Close */

  fclose(fp);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_memlcd_initialize
 *
 * Description:
 *   Initialize the Sharp Memory LCD device.
 *
 ****************************************************************************/

int up_memlcd_initialize(void)
{
  FAR struct spi_dev_s *spi;

  /* Only initialize the driver once. */

  if (memlcd.initialized)
    return OK;

  /* Initialize PWM for EXTVCOM pulse. */

  if (!memlcd_setup_extvcom())
    {
      return -ENODEV;
    }

  /* Get an instance of the SPI interface */

  spi = up_spiinitialize(CONFIG_LCD_SHARP_MEMLCD_SPI_BUS);
  if (!spi)
    {
      idbg("Failed to initialize SPI bus %d\n", CONFIG_LCD_SHARP_MEMLCD_SPI_BUS);
      return -ENODEV;
    }

  memlcd.dev = memlcd_initialize(spi, &memlcd_priv, 0);
  if (!memlcd.dev)
    {
      idbg("Failed to initialize Sharp Memory LCD\n");
      return -ENODEV;
    }

  /* Register memlcd. */
  memlcd_console_register(memlcd.dev, 0);

#if TEST_MEMLCD
  /* Test. */
  test_memlcd(memlcd.dev, "/dev/memlcdcon0");
#endif

  memlcd.initialized = true;
  return OK;
}

#endif /* CONFIG_STM32_LCD */
