/************************************************************************************
 * configs/haltian-tsone/src/up_deepsleep.c
 * arch/arm/src/board/up_deepsleep.c
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <debug.h>
#include <sched.h>
#include <nuttx/arch.h>
#include <nuttx/mmcsd.h>
#include <nuttx/random.h>

#include <arch/board/board-deepsleep.h>
#include <arch/board/board-pwrctl.h>
#include <arch/board/board-gpio.h>
#include <arch/board/board.h>
#include "chip.h"

#include "stm32.h"
#include "stm32_pm.h"
#include "stm32_uart.h"
#include "stm32_rtc.h"
#include "haltian-tsone.h"
#include "up_modem.h"
#include "up_gpio.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

//#define DEEPSLEEP_WAKETIME_DEBUG
//#define DEEPSLEEP_SLEEPTIME_DEBUG

#define DEEP_SLEEP_MIN_MSECS 100

#ifndef BOARD_DEEPSLEEP_SKIP_DEBUG
#  define dss_lldbg(...) ((void)0)
#else
#  define dss_lldbg(...) lldbg(__VA_ARGS__)
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
* Name: board_deepsleep_with_stopmode_msecs
*
* Description:
*   Drive MCU to STOP mode for 'msecs'. Wake-up is performed either by RTC
*   or by external interrupt on EXTI line.
*
************************************************************************************/

#ifndef CONFIG_BOARD_DISABLE_DEEPSLEEP

void board_deepsleep_with_stopmode_msecs(uint32_t msecs)
{
#if defined(CONFIG_RTC) && defined(CONFIG_RTC_DATETIME)
  struct timespec diff_ts;
#endif
  uint32_t millisecs;
#ifdef CONFIG_BOARD_DEEPSLEEP_RECONFIGURE_GPIOS
  uint32_t gpio_off_mask;
  bool sdcard_enabled;
#endif
  uint32_t regval;
  int ret;
#ifdef DEEPSLEEP_WAKETIME_DEBUG
  static struct timespec wake_ts;
#endif
#ifdef DEEPSLEEP_SLEEPTIME_DEBUG
  bool slept = false;
#endif

  /* Do not allow other tasks to takeover (for example, by wake-up with semaphore
   * from EXTI interrupt) before we have restored MCU functionalities. */

  sched_lock();

  /* Check if modem is in such state that in can allow deep-sleep. 'msecs'
   * might be adjusted for quicker wake-up from deep-sleep in some activity
   * modes. */

  if (!up_modem_deep_sleep_readiness(&msecs))
    {
      dss_lldbg("INFO: Modem active, skipping deep-sleep!\n");

      goto skip_deepsleep;
    }

  millisecs = msecs;

#ifdef DEEPSLEEP_SLEEPTIME_DEBUG
  lldbg("sleep for %d msec\n", millisecs);
#endif

  if (millisecs != 0 && millisecs < DEEP_SLEEP_MIN_MSECS)
    {
      dss_lldbg("INFO: too short deep-sleep: %d msec!\n", millisecs);

      goto skip_deepsleep;
    }

#ifdef CONFIG_BOARD_DEEPSLEEP_RECONFIGURE_GPIOS

  /* Special support for SDcard. */

  if (mmcsd_slot_pm_allowed(CONFIG_BOARD_MMCSDSLOTNO) == false)
    {
      dss_lldbg("INFO: SDcard active while trying deep-sleep!\n");

      goto skip_deepsleep;
    }

  if (stm32_gpioread(GPIO_CHIP_SELECT_SDCARD) == false)
    {
      lldbg("ERROR! SDcard communication active while trying deep-sleep!\n");

      goto skip_deepsleep;
    }

  /* Following devices do not have proper GPIO reconfigure recovery yet (and
   * some cannot work with deep-sleep at all). Abort deep-sleep instead of
   * introducing hard-to-diagnose bugs and general instability.
   */

  if (stm32_gpioread(GPIO_REGULATOR_WLAN))
    {
      lldbg("ERROR! Trying to deep-sleep when WLAN active!\n");

      goto skip_deepsleep;
    }

  if (stm32_gpioread(GPIO_REGULATOR_GPS))
    {
      lldbg("ERROR! Trying to deep-sleep when GPS active!\n");

      goto skip_deepsleep;
    }

#ifdef CONFIG_BLUETOOTH
  if (stm32_gpioread(GPIO_REGULATOR_BLUETOOTH))
    {
      lldbg("ERROR! Trying to deep-sleep when Bluetooth active!\n");

      goto skip_deepsleep;
    }
#endif

#ifdef CONFIG_LCD_SSD1306
  if (stm32_gpioread(GPIO_REGULATOR_DISPLAY))
    {
      lldbg("ERROR! Trying to deep-sleep when Display active!\n");

      goto skip_deepsleep;
    }
#endif

#endif /* CONFIG_BOARD_DEEPSLEEP_RECONFIGURE_GPIOS */

#ifdef DEEPSLEEP_WAKETIME_DEBUG
  if (wake_ts.tv_nsec || wake_ts.tv_sec)
    {
      struct timespec curr_ts;
      int64_t msecs;

      clock_gettime(CLOCK_REALTIME, &curr_ts);

      msecs = ((int64_t)curr_ts.tv_sec - wake_ts.tv_sec) * 1000;
      msecs += ((int64_t)curr_ts.tv_nsec - wake_ts.tv_nsec) / (1000 * 1000);

      lldbg("sleep to sleep time: %d msecs\n", (int)msecs);
    }
  else
    {
      lldbg("first sleep (for %u secs)\n", secs);
    }
#endif

#ifdef CONFIG_RTC_PERIODIC_AUTORELOAD_WAKEUP
  if (millisecs > 0)
    {
      /* Prepare RTC for wake-up. */

      ret = up_rtc_setperiodicwakeup(millisecs, NULL);
      DEBUGASSERT(ret == OK);
    }
#else
  DEBUGASSERT(secs == 0);
#endif

  /* Prevent USART interrupt activity, make sure that last transmission has
   * completed. */

  stm32_serial_set_suspend(true);

  /* Setup modem GPIOs for deep-sleep. */

  up_modem_gpio_suspend(true);

#ifdef CONFIG_BOARD_DEEPSLEEP_RECONFIGURE_GPIOS

  /* Read control state of SDCard regulator (TODO: add proper pwrctl api) */

  sdcard_enabled = stm32_gpioread(GPIO_REGULATOR_SDCARD);
  if (sdcard_enabled)
    {
      /* Mark SDcard as disconnected for block driver. */

      mmcsd_slot_pm_suspend(CONFIG_BOARD_MMCSDSLOTNO);
    }

  /* Force SDcard off. */

  stm32_gpiowrite(GPIO_REGULATOR_SDCARD, false);

  /* SDcard is off, and does not react on GPIOs. Force GPIOs low to avoid leak
   * current.
   */

  gpio_off_mask = ~(GPIO_PUPD_MASK | GPIO_MODE_MASK | GPIO_OUTPUT_SET);
  stm32_configgpio((GPIO_SPI3_MOSI & gpio_off_mask) | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT);
  stm32_configgpio((GPIO_SPI3_MISO & gpio_off_mask) | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT);
  stm32_configgpio((GPIO_SPI3_SCK & gpio_off_mask) | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT);
  stm32_configgpio((GPIO_CHIP_SELECT_SDCARD & gpio_off_mask) | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT);

  /* Reconfigure GPIO's for stop mode (most pins are setup as analog input). */

  up_reconfigure_gpios_for_pmstop();

#endif

#if defined(CONFIG_USBDEV) && defined (CONFIG_STM32_USB)
  /* Uninitialize USB stack, free up resources */

  up_usbuninitialize();

  /* Reset USB peripheral */

  regval = getreg32(STM32_RCC_APB1RSTR);
  putreg32(regval | RCC_APB1RSTR_USBRST, STM32_RCC_APB1RSTR);
  putreg32(regval, STM32_RCC_APB1RSTR);
#endif

#if defined(CONFIG_STM32_PWR)
  /* Disable PVD during sleep because I_DD (PVD/BOR) consumes 2.6 uA,
     and because there is nothing better to do than die while sleeping
     if lose power in stop-mode. */

  stm32_pwr_disablepvd();
#endif

#ifndef CONFIG_BOARD_DEEPSLEEP_SKIP_PMSTOP
  /* Enter stop-mode with MCU internal regulator in low-power mode. */

  ret = stm32_pmstop(true);
#endif

#ifdef DEEPSLEEP_SLEEPTIME_DEBUG
  slept = true;
#endif

#if defined(CONFIG_STM32_PWR)
  /* Re-enable PVD and check if voltage has dropped too much
     while we slept. We might have enough power for MCU, but not
     for SD card or other peripherals. */

  stm32_pwr_enablepvd();
  board_pwr_checkpvd();
#endif

#ifdef CONFIG_RTC_PERIODIC_AUTORELOAD_WAKEUP
  if (millisecs > 0)
    {
      /* Prevent more wake-ups. */

      (void)up_rtc_cancelperiodicwakeup();
    }
#endif

#if defined(CONFIG_RTC) && defined(CONFIG_RTC_DATETIME)
  /* Synchronize systime clock with RTC after STOP.
   *
   * During STOP, RTC advanced normally and system clock was paused.
   */

  clock_resynchronize(&diff_ts);

  /* Add nanoseconds to entropy pool. */

  add_time_randomness(diff_ts.tv_nsec);

#if defined(CONFIG_CLOCK_MONOTONIC) && defined(CONFIG_CLOCK_MONOTONIC_OFFSET_TIME)
  /* Adjust CLOCK_MONOTONIC offset by amount of time adjusted by RTC. */

  clock_increase_monotonic_offset(&diff_ts);
#endif

#ifdef DEEPSLEEP_WAKETIME_DEBUG
  (void)clock_gettime(CLOCK_REALTIME, &wake_ts);
#endif

#endif

  /* Stop mode disables HSE/HSI/PLL and wake happens with default system
   * clock (MSI, 2Mhz). So reconfigure clocks early on. RTC is however
   * handled before this for accurate time keeping.
   */

  stm32_clockconfig();

#ifdef CONFIG_BOARD_DEEPSLEEP_RECONFIGURE_GPIOS
  /* Restore pin configuration */

  up_restore_gpios_after_pmstop();
#endif

#if defined(CONFIG_USBDEV) && defined (CONFIG_STM32_USB)
  /* Initialize USB */
  /* TODO initialize USB properly, up_usbinitialize() call alone is not sufficient */

  up_usbinitialize();

#endif

  /* Setup modem GPIOs for active mode. */

  up_modem_gpio_suspend(false);

  /* Resume serial ports after setting up system clock, as USART baud rate is
   * configured relative to system clock. */

  stm32_serial_set_suspend(false);

  /* Check that we have enough battery left, just in case
     PVD detection missed it. */

  board_pwr_checkvbat(false);

#ifdef CONFIG_BOARD_DEEPSLEEP_RECONFIGURE_GPIOS
  /* Special support for SD card. */

  /* Restore SDcard GPIOs */

  stm32_configgpio(GPIO_SPI3_MOSI);
  stm32_configgpio(GPIO_SPI3_MISO | GPIO_PULLUP);
  stm32_configgpio(GPIO_SPI3_SCK);
  stm32_configgpio(GPIO_CHIP_SELECT_SDCARD);

  if (sdcard_enabled)
    {
      /* Re-enable power now that GPIOs are in correct state. */

      stm32_gpiowrite(GPIO_REGULATOR_SDCARD, true);

      /* Mark SDcard as reconnected for block driver, will reinitialize/reconfigure
       * SDcard. Apparently, block driver works fine without additional delay
       * for regulator. Might be because plug-in nature of SDcards require
       * block driver to keep retrying initialization until card stabilizes.
       */

      mmcsd_slot_pm_resume(CONFIG_BOARD_MMCSDSLOTNO);
    }
#endif

skip_deepsleep:

  /* Allow scheduler to switch tasks. */

  sched_unlock();

#ifdef DEEPSLEEP_SLEEPTIME_DEBUG
  if (slept)
    {
      /* Get amount of time adjusted by RTC. */

      lldbg("slept for %u.%03u\n", diff_ts.tv_sec, diff_ts.tv_nsec / (1000 * 1000));
    }
#endif
}

#else /* !CONFIG_BOARD_DISABLE_DEEPSLEEP */

void board_deepsleep_with_stopmode_msecs(uint32_t msecs)
{
  /* Do nothing, as if woken up by interrupt before sleep. */

  (void)msecs;
}

#endif /* CONFIG_BOARD_DISABLE_DEEPSLEEP */

void board_deepsleep_with_stopmode(uint32_t secs)
{
  if (secs > UINT32_MAX / 1000)
    secs = UINT32_MAX / 1000;

  board_deepsleep_with_stopmode_msecs(secs * 1000);
}
