/************************************************************************************
 * arch/arm/src/stm32/stm32_rtcc.c
 *
 *   Copyright (C) 2012-2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <stdbool.h>
#include <time.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <arch/board/board.h>

#include "up_arch.h"

#include "stm32_rcc.h"
#include "stm32_pwr.h"
#include "stm32_rtc.h"
#include "stm32_exti.h"

#if CONFIG_RTC_PERIODIC_AUTORELOAD_WAKEUP
#include "stm32_exti_wakeup.h"
#endif

#ifdef CONFIG_RTC

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/
/* This RTC implementation supports only date/time RTC hardware */

#ifndef CONFIG_RTC_DATETIME
#  error "CONFIG_RTC_DATETIME must be set to use this driver"
#endif

#ifdef CONFIG_RTC_HIRES
#  error "CONFIG_RTC_HIRES must NOT be set with this driver"
#endif

#ifndef CONFIG_STM32_PWR
#  error "CONFIG_STM32_PWR must selected to use this driver"
#endif

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_RTC
#endif

#ifdef CONFIG_STM32_STM32L15XX
#  if defined(CONFIG_RTC_HSECLOCK)
#    error "RTC with HSE clock not yet implemented for STM32L15XXX"
#  elif defined(CONFIG_RTC_LSICLOCK)
#    error "RTC with LSI clock not yet implemented for STM32L15XXX"
#  endif
#endif

#if !defined(CONFIG_RTC_MAGIC)
# define CONFIG_RTC_MAGIC (0xfacefeee)
#endif

#if !defined(CONFIG_RTC_MAGIC_REG)
# define CONFIG_RTC_MAGIC_REG (0)
#endif

/* Constants ************************************************************************/

#define SYNCHRO_TIMEOUT  (0x00020000)
#define INITMODE_TIMEOUT (0x00010000)
#define RTC_MAGIC        CONFIG_RTC_MAGIC
#define RTC_MAGIC_REG    STM32_RTC_BKR(CONFIG_RTC_MAGIC_REG)

/* Proxy definitions to make the same code work for all the STM32 series ************/

#if defined(CONFIG_STM32_STM32L15XX)
# define STM32_RCC_XXX       STM32_RCC_CSR
# define RCC_XXX_YYYRST      RCC_CSR_RTCRST
# define RCC_XXX_RTCEN       RCC_CSR_RTCEN
# define RCC_XXX_RTCSEL_MASK RCC_CSR_RTCSEL_MASK
# define RCC_XXX_RTCSEL_LSE  RCC_CSR_RTCSEL_LSE
# define RCC_XXX_RTCSEL_LSI  RCC_CSR_RTCSEL_LSI
# define RCC_XXX_RTCSEL_HSE  RCC_CSR_RTCSEL_HSE
#else
# define STM32_RCC_XXX       STM32_RCC_BDCR
# define RCC_XXX_YYYRST      RCC_BDCR_BDRST
# define RCC_XXX_RTCEN       RCC_BDCR_RTCEN
# define RCC_XXX_RTCSEL_MASK RCC_BDCR_RTCSEL_MASK
# define RCC_XXX_RTCSEL_LSE  RCC_BDCR_RTCSEL_LSE
# define RCC_XXX_RTCSEL_LSI  RCC_BDCR_RTCSEL_LSI
# define RCC_XXX_RTCSEL_HSE  RCC_BDCR_RTCSEL_HSE
#endif

/* Debug ****************************************************************************/

#ifdef CONFIG_DEBUG_RTC
#  define rtcdbg    dbg
#  define rtcvdbg   vdbg
#  define rtclldbg  lldbg
#  define rtcllvdbg llvdbg
#else
#  define rtcdbg(x...)
#  define rtcvdbg(x...)
#  define rtclldbg(x...)
#  define rtcllvdbg(x...)
#endif

/************************************************************************************
 * Private Types
 ************************************************************************************/

/************************************************************************************
 * Private Data
 ************************************************************************************/

/* Callback to use when the alarm expires */

#ifdef CONFIG_RTC_ALARM
static alarmcb_t g_alarmcb;
#endif
#ifdef CONFIG_RTC_PERIODIC_AUTORELOAD_WAKEUP
static wakeupcb_t g_wakeupcb;
#endif

/************************************************************************************
 * Public Data
 ************************************************************************************/

/* g_rtc_enabled is set true after the RTC has successfully initialized */

volatile bool g_rtc_enabled = false;

/* g_stm32_rtc_resumed is set true if RTC was active over reset */

bool g_stm32_rtc_resumed = false;

/* g_stm32_rtc_resumed_time_set contains UTC when RTC clock was setup. */

time_t g_stm32_rtc_resumed_time_set;

/************************************************************************************
 * Private Functions
 ************************************************************************************/
/************************************************************************************
 * Name: rtc_dumpregs
 *
 * Description:
 *    Disable RTC write protection
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_RTC
static void rtc_dumpregs(FAR const char *msg)
{
  rtclldbg("%s:\n", msg);
  rtclldbg("      TR: %08x\n", getreg32(STM32_RTC_TR));
  rtclldbg("      DR: %08x\n", getreg32(STM32_RTC_DR));
  rtclldbg("      CR: %08x\n", getreg32(STM32_RTC_CR));
  rtclldbg("     ISR: %08x\n", getreg32(STM32_RTC_ISR));
  rtclldbg("    PRER: %08x\n", getreg32(STM32_RTC_PRER));
  rtclldbg("    WUTR: %08x\n", getreg32(STM32_RTC_WUTR));
#ifndef CONFIG_STM32_STM32F30XX
  rtclldbg("  CALIBR: %08x\n", getreg32(STM32_RTC_CALIBR));
#endif
  rtclldbg("  ALRMAR: %08x\n", getreg32(STM32_RTC_ALRMAR));
  rtclldbg("  ALRMBR: %08x\n", getreg32(STM32_RTC_ALRMBR));
  rtclldbg("  SHIFTR: %08x\n", getreg32(STM32_RTC_SHIFTR));
  rtclldbg("    TSTR: %08x\n", getreg32(STM32_RTC_TSTR));
  rtclldbg("    TSDR: %08x\n", getreg32(STM32_RTC_TSDR));
  rtclldbg("   TSSSR: %08x\n", getreg32(STM32_RTC_TSSSR));
  rtclldbg("    CALR: %08x\n", getreg32(STM32_RTC_CALR));
  rtclldbg("   TAFCR: %08x\n", getreg32(STM32_RTC_TAFCR));
  rtclldbg("ALRMASSR: %08x\n", getreg32(STM32_RTC_ALRMASSR));
  rtclldbg("ALRMBSSR: %08x\n", getreg32(STM32_RTC_ALRMBSSR));
  rtclldbg("MAGICREG: %08x\n", getreg32(RTC_MAGIC_REG));
}
#else
#  define rtc_dumpregs(msg)
#endif

/************************************************************************************
 * Name: rtc_dumptime
 *
 * Description:
 *    Disable RTC write protection
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_RTC
static void rtc_dumptime(FAR const struct tm *tp, FAR const char *msg)
{
  rtclldbg("%s:\n", msg);
  rtclldbg("  tm_sec: %08x\n", tp->tm_sec);
  rtclldbg("  tm_min: %08x\n", tp->tm_min);
  rtclldbg(" tm_hour: %08x\n", tp->tm_hour);
  rtclldbg(" tm_mday: %08x\n", tp->tm_mday);
  rtclldbg("  tm_mon: %08x\n", tp->tm_mon);
  rtclldbg(" tm_year: %08x\n", tp->tm_year);
}
#else
#  define rtc_dumptime(tp, msg)
#endif

/************************************************************************************
 * Name: rtc_wprunlock
 *
 * Description:
 *    Disable RTC write protection
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static void rtc_wprunlock(void)
{
  /* Enable write access to the backup domain (RTC registers, RTC backup data
   * registers and backup SRAM).
   */

  stm32_pwr_enablebkp(true);

  /* The following steps are required to unlock the write protection on all the
   * RTC registers (except for RTC_ISR[13:8], RTC_TAFCR, and RTC_BKPxR).
   *
   * 1. Write 0xCA into the RTC_WPR register.
   * 2. Write 0x53 into the RTC_WPR register.
   *
   * Writing a wrong key re-activates the write protection.
   */

  putreg32(0xca, STM32_RTC_WPR);
  putreg32(0x53, STM32_RTC_WPR);
}

/************************************************************************************
 * Name: rtc_wprunlock
 *
 * Description:
 *    Enable RTC write protection
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static inline void rtc_wprlock(void)
{
  /* Writing any wrong key re-activates the write protection. */

  putreg32(0xff, STM32_RTC_WPR);

  /* Disable write access to the backup domain (RTC registers, RTC backup data
   * registers and backup SRAM).
   */

  stm32_pwr_enablebkp(false);
}

/************************************************************************************
 * Name: rtc_synchwait
 *
 * Description:
 *   Waits until the RTC Time and Date registers (RTC_TR and RTC_DR) are
 *   synchronized with RTC APB clock.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

static int rtc_synchwait(void)
{
  volatile uint32_t timeout;
  uint32_t regval;
  int ret;

  /* Disable the write protection for RTC registers */

  rtc_wprunlock();

  /* Clear Registers synchronization flag (RSF) */

  regval  = getreg32(STM32_RTC_ISR);
  regval &= ~RTC_ISR_RSF;
  putreg32(regval, STM32_RTC_ISR);

  /* Now wait the registers to become synchronised */

  ret = -ETIMEDOUT;
  for (timeout = 0; timeout < SYNCHRO_TIMEOUT; timeout++)
    {
      regval = getreg32(STM32_RTC_ISR);
      if ((regval & RTC_ISR_RSF) != 0)
        {
          /* Synchronized */

          ret = OK;
          break;
        }
    }

  /* Re-enable the write protection for RTC registers */

  rtc_wprlock();
  return ret;
}

/************************************************************************************
 * Name: rtc_enterinit
 *
 * Description:
 *   Enter RTC initialization mode.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

static int rtc_enterinit(void)
{
  volatile uint32_t timeout;
  uint32_t regval;
  int ret;

  /* Check if the Initialization mode is already set */

  regval = getreg32(STM32_RTC_ISR);

  ret = OK;
  if ((regval & RTC_ISR_INITF) == 0)
    {
      /* Set the Initialization mode */

      putreg32(RTC_ISR_INIT, STM32_RTC_ISR);

      /* Wait until the RTC is in the INIT state (or a timeout occurs) */

      ret = -ETIMEDOUT;
      for (timeout = 0; timeout < INITMODE_TIMEOUT; timeout++)
        {
          regval = getreg32(STM32_RTC_ISR);
          if ((regval & RTC_ISR_INITF) != 0)
            {
              ret = OK;
              break;
            }
        }
    }

  return ret;
}

/************************************************************************************
 * Name: rtc_exitinit
 *
 * Description:
 *   Exit RTC initialization mode.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

static void rtc_exitinit(void)
{
  uint32_t regval;

  regval = getreg32(STM32_RTC_ISR);
  regval &= ~(RTC_ISR_INIT);
  putreg32(regval, STM32_RTC_ISR);
}

/************************************************************************************
 * Name: rtc_bin2bcd
 *
 * Description:
 *   Converts a 2 digit binary to BCD format
 *
 * Input Parameters:
 *   value - The byte to be converted.
 *
 * Returned Value:
 *   The value in BCD representation
 *
 ************************************************************************************/

static uint32_t rtc_bin2bcd(int value)
{
  uint32_t msbcd = 0;

  while (value >= 10)
    {
      msbcd++;
      value -= 10;
    }

  return (msbcd << 4) | value;
}

/************************************************************************************
 * Name: rtc_bin2bcd
 *
 * Description:
 *   Convert from 2 digit BCD to binary.
 *
 * Input Parameters:
 *   value - The BCD value to be converted.
 *
 * Returned Value:
 *   The value in binary representation
 *
 ************************************************************************************/

static int rtc_bcd2bin(uint32_t value)
{
  uint32_t tens = (value >> 4) * 10;
  return (int)(tens + (value & 0x0f));
}

/************************************************************************************
 * Name: rtc_setup
 *
 * Description:
 *   Performs first time configuration of the RTC.  A special value written into
 *   back-up register 0 will prevent this function from being called on sub-sequent
 *   resets or power up.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

static int rtc_setup(void)
{
  uint32_t regval;
  int ret;

  /* Disable the write protection for RTC registers */

  rtc_wprunlock();

  /* Set Initialization mode */

  ret = rtc_enterinit();
  if (ret == OK)
    {
      /* Set the 24 hour format by clearing the FMT bit in the RTC
       * control register
       */

      regval = getreg32(STM32_RTC_CR);
      regval &= ~RTC_CR_FMT;
      putreg32(regval, STM32_RTC_CR);

      /* Configure RTC pre-scaler with the required values */

#ifdef CONFIG_RTC_HSECLOCK
      /* For a 1 MHz clock this yields 0.9999360041 Hz on the second
       * timer - which is pretty close.
       */

      putreg32(((uint32_t)7182 << RTC_PRER_PREDIV_S_SHIFT) |
              ((uint32_t)0x7f << RTC_PRER_PREDIV_A_SHIFT),
              STM32_RTC_PRER);
#else
      /* Correct values for 32.768 KHz LSE clock and inaccurate LSI clock */

      putreg32(((uint32_t)0xff << RTC_PRER_PREDIV_S_SHIFT) |
              ((uint32_t)0x7f << RTC_PRER_PREDIV_A_SHIFT),
              STM32_RTC_PRER);
#endif

      /* Exit RTC initialization mode */

      rtc_exitinit();
    }

  /* Re-enable the write protection for RTC registers */

  rtc_wprlock();

  return ret;
}

/************************************************************************************
 * Name: rtc_resume
 *
 * Description:
 *   Called when the RTC was already initialized on a previous power cycle.  This
 *   just brings the RTC back into full operation.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

static void rtc_resume(void)
{
#ifdef CONFIG_RTC_ALARM
  uint32_t regval;

  /* Clear the RTC alarm flags */

  regval  = getreg32(STM32_RTC_ISR);
  regval &= ~(RTC_ISR_ALRAF|RTC_ISR_ALRBF);
  putreg32(regval, STM32_RTC_ISR);

  /* Clear the EXTI Line 17 Pending bit (Connected internally to RTC Alarm) */

  putreg32((1 << 17), STM32_EXTI_PR);
#endif
}

/************************************************************************************
 * Name: rtc_interrupt
 *
 * Description:
 *    RTC interrupt service routine
 *
 * Input Parameters:
 *   irq - The IRQ number that generated the interrupt
 *   context - Architecture specific register save information.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value on failure.
 *
 ************************************************************************************/

#if CONFIG_RTC_ALARM || CONFIG_RTC_PERIODIC_AUTORELOAD_WAKEUP
static int rtc_interrupt(int irq, void *context)
{
  uint32_t regval = 0;

  regval = getreg32(STM32_RTC_ISR);

#ifdef CONFIG_RTC_PERIODIC_AUTORELOAD_WAKEUP
  if (irq == STM32_IRQ_RTC_WKUP)
    {
      regval &= ~RTC_ISR_WUTF;
      putreg32(regval, STM32_RTC_ISR);

      if (g_wakeupcb != NULL)
        g_wakeupcb();
    }
#endif

#ifdef CONFIG_RTC_ALARM
  if (irq == STM32_IRQ_RTCALRM)
    {
#  ifdef CONFIG_RTC_USE_ALARM_A
      regval &= ~RTC_ISR_ALRAF;
#  else
      regval &= ~RTC_ISR_ALRBF;
#  endif
      putreg32(regval, STM32_RTC_ISR);

      if (g_alarmcb != NULL)
        g_alarmcb();
    }
#endif

  return OK;
}
#endif

/************************************************************************************
 * Name: rtc_set_wcksel
 *
 * Description:
 *    Sets RTC wakeup clock selection value
 *
 * Input Parameters:
 *   type - RTC divider's type.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value on failure.
 *
 ************************************************************************************/

#ifdef CONFIG_RTC_PERIODIC_AUTORELOAD_WAKEUP
static void rtc_set_wcksel(unsigned int wucksel)
{
  uint32_t regval = 0;

  regval = getreg32(STM32_RTC_CR);
  regval &= ~RTC_CR_WUCKSEL_MASK;
  regval |= wucksel;
  putreg32(regval, STM32_RTC_CR);
}
#endif

/************************************************************************************
 * Name: rtc_write_alarm_value
 *
 * Description:
 *   Calculates and sets alarm based on current time.
 *
 * Input Parameters:
 *   alarm_sec - time to trigger alarm depending on current time
 *                       in seconds.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int rtc_write_alarm_value(time_t alarm_sec)
{
  struct tm cur_tm;
  struct tm *tm_p = NULL;
  uint32_t regval = 0;
  uint32_t tm_mask = 0;
  time_t current_epoch = 0;
  time_t alarm_time = 0;
  int ret = OK;

  ret = up_rtc_getdatetime(&cur_tm);
  current_epoch = mktime(&cur_tm);

  alarm_time = current_epoch + alarm_sec;

  tm_p = gmtime(&alarm_time);

  tm_mask = (!tm_p->tm_mday ? RTC_ALRMR_MSK4 : 0);
  tm_mask |= (!tm_p->tm_hour ? RTC_ALRMR_MSK3 : 0);
  tm_mask |= (!tm_p->tm_min ? RTC_ALRMR_MSK2 : 0);
  tm_mask |= (!tm_p->tm_sec ? RTC_ALRMR_MSK1 : 0);

  regval = tm_mask;
  regval |= (rtc_bin2bcd(tm_p->tm_mday / 10) << RTC_ALRMR_DT_SHIFT) | (rtc_bin2bcd(tm_p->tm_mday % 10) << RTC_ALRMR_DU_SHIFT);
  regval |= (rtc_bin2bcd(tm_p->tm_hour / 10) << RTC_ALRMR_HT_SHIFT) | (rtc_bin2bcd(tm_p->tm_hour % 10) << RTC_ALRMR_HU_SHIFT);
  regval |= (rtc_bin2bcd(tm_p->tm_min / 10) << RTC_ALRMR_MNT_SHIFT) | (rtc_bin2bcd(tm_p->tm_min % 10) << RTC_ALRMR_MNU_SHIFT);
  regval |= (rtc_bin2bcd(tm_p->tm_sec / 10) << RTC_ALRMR_ST_SHIFT)  | (rtc_bin2bcd(tm_p->tm_sec % 10) << RTC_ALRMR_SU_SHIFT);

#ifdef CONFIG_RTC_USE_ALARM_A
  putreg32(regval, STM32_RTC_ALRMAR);
#else
  putreg32(regval, STM32_RTC_ALRMBR);
#endif

  rtclldbg("Value put in alarm reg: 0x%08X, time mask: 0x%08X\n", regval, tm_mask);

  return ret;
}
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#ifdef CONFIG_RTC_PERIODIC_AUTORELOAD_WAKEUP

/************************************************************************************
 * Name: up_rtc_setperiodicwakeup
 *
 * Description:
 *    Sets RTC periodic autoreload wakeup
 *
 * Input Parameters:
 *   millisecs     - Time to sleep in seconds before wakeup and auto-reloading.
 *   callback      - the function to call when the alarm expires.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value on failure.
 *
 ************************************************************************************/

int up_rtc_setperiodicwakeup(unsigned int millisecs, wakeupcb_t callback)
{
  const uint32_t rtc_div16_max_msecs = 16 * 1000 * 0xffffU / STM32_LSE_FREQUENCY;
  unsigned int wutr_val;
  int ret = OK;
  int timeout = 0;
  uint32_t regval = 0;
  unsigned int secs = millisecs / 1000;

  rtclldbg("Set autoreload wakeup\n");

  /* Check input variables for correctness. */

  if (millisecs == 0)
    {
      /* Wakeup must be greater than zero. */

      return -EINVAL;
    }

  /* Lets use RTC wake-up with 0.001 sec to ~18 hour range. */

  if (secs > 0xffffU)
    {
      /* More than max. */

      secs = 0xffffU;
      millisecs = secs * 1000;
    }

  rtc_wprunlock();

  /* Clear WUTE in RTC_CR to disable the wakeup timer */

  regval = getreg32(STM32_RTC_CR);
  regval &= ~RTC_CR_WUTE;
  rtclldbg("RTC_CR: 0x%08x\n", regval);
  putreg32(regval, STM32_RTC_CR);

  /* Poll WUTWF until it is set in RTC_ISR (takes around 2 RTCCLK clock cycles) */

  ret = -ETIMEDOUT;
  for (timeout = 0; timeout < SYNCHRO_TIMEOUT; timeout++)
    {
      regval = getreg32(STM32_RTC_ISR);
      if ((regval & RTC_ISR_WUTWF) != 0)
        {
          /* Synchronized */

          ret = OK;
          break;
        }
    }

  /* Set callback function pointer. */

  g_wakeupcb = callback;

  if (millisecs <= rtc_div16_max_msecs)
    {
      unsigned int ticks;

      /* Select wake-up with 32768/16 hz counter. */

      rtc_set_wcksel(RTC_CR_WUCKSEL_RTCDIV16);

      /* Get number of ticks. */

      ticks = millisecs * STM32_LSE_FREQUENCY / (16 * 1000);

      /* Wake-up is after WUT+1 ticks. */

      wutr_val = ticks - 1;
    }
  else
    {
      /* Select wake-up with 1hz counter. */

      rtc_set_wcksel(RTC_CR_WUCKSEL_CKSPRE);

      /* Wake-up is after WUT+1 ticks. */

      wutr_val = secs - 1;
    }

  /* Program the wakeup auto-reload value WUT[15:0], and the wakeup clock
   * selection.
   */

  putreg32(wutr_val, STM32_RTC_WUTR);

  stm32_exti_wakeup(true, false, false, rtc_interrupt);

  regval = getreg32(STM32_RTC_CR);
  regval |= RTC_CR_WUTIE | RTC_CR_WUTE;
  putreg32(regval, STM32_RTC_CR);

  /* Just in case resets the WUTF flag in RTC_ISR */

  regval = getreg32(STM32_RTC_ISR);
  regval &= ~RTC_ISR_WUTF;
  putreg32(regval, STM32_RTC_ISR);

  rtc_wprlock();

  return ret;
}

/************************************************************************************
 * Name: up_rtc_cancelperiodicwakeup
 *
 * Description:
 *    Turns off RTC periodic autoreload wakeup
 *
 * Input Parameters:
 *   none.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value on failure.
 *
 ************************************************************************************/

int up_rtc_cancelperiodicwakeup(void)
{
  int ret = OK;
  int timeout = 0;
  uint32_t regval = 0;

  rtclldbg("Turns off autoreload wakeup\n");

  stm32_exti_wakeup(true, false, false, NULL);

  rtc_wprunlock();

  /* Clear WUTE in RTC_CR to disable the wakeup timer */

  regval = getreg32(STM32_RTC_CR);
  regval &= ~RTC_CR_WUTE;
  regval &= ~RTC_CR_WUTIE;
  rtclldbg("RTC_CR: 0x%08x\n", regval);
  putreg32(regval, STM32_RTC_CR);

  /* Poll WUTWF until it is set in RTC_ISR (takes around 2 RTCCLK clock cycles) */

  ret = -ETIMEDOUT;
  for (timeout = 0; timeout < SYNCHRO_TIMEOUT; timeout++)
    {
      regval = getreg32(STM32_RTC_ISR);
      if ((regval & RTC_ISR_WUTWF) != 0)
        {
          /* Synchronized */

          ret = OK;
          break;
        }
    }

  /* Clears RTC_WUTR register */

  regval = getreg32(STM32_RTC_WUTR);
  regval &= ~RTC_WUTR_MASK;
  putreg32(regval, STM32_RTC_WUTR);

  rtc_wprlock();

  return ret;
}

#endif /* CONFIG_RTC_PERIODIC_AUTORELOAD_WAKEUP */

/************************************************************************************
 * Name: up_rtcinitialize
 *
 * Description:
 *   Initialize the hardware RTC per the selected configuration.  This function is
 *   called once during the OS initialization sequence
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

int up_rtcinitialize(void)
{
  uint32_t regval, clksrc, tr_bkp, dr_bkp;
  int ret;
  int maxretry = 10;
  int nretry = 0;

  /* Clocking for the PWR block must be provided.  However, this is done
   * unconditionally in stm32f40xxx_rcc.c on power up.  This done unconditionally
   * because the PWR block is also needed to set the internal voltage regulator for
   * maximum performance.
   */

  rtc_dumpregs("On reset");

  /* Enable write access to the backup domain (RTC registers, RTC
   * backup data registers and backup SRAM).
   */

  stm32_pwr_enablebkp(true);

  /* Select the clock source */
  /* Save the token before losing it when resetting */

  regval = getreg32(RTC_MAGIC_REG);

  if (regval != RTC_MAGIC)
    {
      uint32_t bkregs[STM32_RTC_BKCOUNT];
      int i;

      /* Backup backup-registers before RTC reset. */

      for (i = 0; i < STM32_RTC_BKCOUNT; i++)
        {
          bkregs[i] = getreg32(STM32_RTC_BKR(i));
        }

      /* We might be changing RTCSEL - to ensure such changes work, we must reset the
       * backup domain (having backed up the RTC_MAGIC token)
       */

      modifyreg32(STM32_RCC_XXX, 0, RCC_XXX_YYYRST);
      modifyreg32(STM32_RCC_XXX, RCC_XXX_YYYRST, 0);

      /* Restore backup-registers, except RTC related. */

      for (i = 0; i < STM32_RTC_BKCOUNT; i++)
        {
          if (RTC_MAGIC_REG == STM32_RTC_BKR(i))
            {
              continue;
            }

          if (STM32_RTC_BKR(i) == STM32_RTC_BK1R)
            {
              continue;
            }

          if (STM32_RTC_BKR(i) == STM32_RTC_BK2R)
            {
              continue;
            }

          putreg32(bkregs[i], STM32_RTC_BKR(i));
        }

      /* Some boards do not have the external 32khz oscillator installed, for those
       * boards we must fallback to the crummy internal RC clock or the external high
       * rate clock
       */

#ifdef CONFIG_RTC_HSECLOCK
      /* Use the HSE clock as the input to the RTC block */

      modifyreg32(STM32_RCC_XXX, RCC_XXX_RTCSEL_MASK, RCC_XXX_RTCSEL_HSE);

#elif defined(CONFIG_RTC_LSICLOCK)
      /* Since RCC was reseted above, need to re-enable LSI. */

      stm32_rcc_enablelsi();

      /* Use the LSI clock as the input to the RTC block */

      modifyreg32(STM32_RCC_XXX, RCC_XXX_RTCSEL_MASK, RCC_XXX_RTCSEL_LSI);

#elif defined(CONFIG_RTC_LSECLOCK)
      /* Since RCC was reseted above, need to re-enable LSE. */

      stm32_pwr_enablebkp(false);
      stm32_rcc_enablelse();
      stm32_pwr_enablebkp(true);

      /* Use the LSE clock as the input to the RTC block */

      modifyreg32(STM32_RCC_XXX, RCC_XXX_RTCSEL_MASK, RCC_XXX_RTCSEL_LSE);
#endif
      /* Enable the RTC Clock by setting the RTCEN bit in the RCC register */

      modifyreg32(STM32_RCC_XXX, 0, RCC_XXX_RTCEN);
    }
  else /* The RTC is already in use: check if the clock source is changed */
    {
      clksrc = getreg32(STM32_RCC_XXX);

#if defined(CONFIG_RTC_HSECLOCK)
      if ((clksrc & RCC_XXX_RTCSEL_MASK) != RCC_XXX_RTCSEL_HSE)
#elif defined(CONFIG_RTC_LSICLOCK)
      if ((clksrc & RCC_XXX_RTCSEL_MASK) != RCC_XXX_RTCSEL_LSI)
#elif defined(CONFIG_RTC_LSECLOCK)
      if ((clksrc & RCC_XXX_RTCSEL_MASK) != RCC_XXX_RTCSEL_LSE)
#endif
        {
          tr_bkp = getreg32(STM32_RTC_TR);
          dr_bkp = getreg32(STM32_RTC_DR);
          modifyreg32(STM32_RCC_XXX, 0, RCC_XXX_YYYRST);
          modifyreg32(STM32_RCC_XXX, RCC_XXX_YYYRST, 0);

#if defined(CONFIG_RTC_HSECLOCK)
          /* Change to the new clock as the input to the RTC block */

          modifyreg32(STM32_RCC_XXX, RCC_XXX_RTCSEL_MASK, RCC_XXX_RTCSEL_HSE);

#elif defined(CONFIG_RTC_LSICLOCK)
          modifyreg32(STM32_RCC_XXX, RCC_XXX_RTCSEL_MASK, RCC_XXX_RTCSEL_LSI);

#elif defined(CONFIG_RTC_LSECLOCK)
          modifyreg32(STM32_RCC_XXX, RCC_XXX_RTCSEL_MASK, RCC_XXX_RTCSEL_LSE);
#endif

          putreg32(tr_bkp,STM32_RTC_TR);
          putreg32(dr_bkp,STM32_RTC_DR);

          /* Remember that the RTC is initialized */

          putreg32(RTC_MAGIC, RTC_MAGIC_REG);

          /* Enable the RTC Clock by setting the RTCEN bit in the RCC register */

          modifyreg32(STM32_RCC_XXX, 0, RCC_XXX_RTCEN);
        }
    }

  /* Disable write access to the backup domain (RTC registers, RTC backup
   * data registers and backup SRAM).
   */

  stm32_pwr_enablebkp(false);

  /* Loop, attempting to initialize/resume the RTC.  This loop is necessary
   * because it seems that occasionally it takes longer to initialize the RTC
   * (the actual failure is in rtc_synchwait()).
   */

  do
    {
      /* Wait for the RTC Time and Date registers to be synchronized with RTC APB
       * clock.
       */

      ret = rtc_synchwait();

      /* Check that rtc_syncwait() returned successfully */

      switch (ret)
        {
          case OK:
            {
              rtclldbg("rtc_syncwait() okay\n");
              break;
            }

          default:
            {
              rtclldbg("rtc_syncwait() failed (%d)\n", ret);
              break;
            }
        }
    }
  while (ret != OK && ++nretry < maxretry);

  /* Check if the one-time initialization of the RTC has already been
   * performed. We can determine this by checking if the magic number
   * has been writing to to back-up date register DR0.
   */

  if (regval != RTC_MAGIC)
    {
      rtclldbg("Do setup\n");

      /* Perform the one-time setup of the LSE clocking to the RTC */

      ret = rtc_setup();

      /* Remember that the RTC is initialized */

      stm32_pwr_enablebkp(true);
      putreg32(RTC_MAGIC, RTC_MAGIC_REG);
      stm32_pwr_enablebkp(false);

      g_stm32_rtc_resumed = false;
    }
  else
    {
      rtclldbg("Do resume\n");

      /* RTC already set-up, just resume normal operation */

      rtc_resume();

      /* Was RTC setup by application on previous run? */

      stm32_pwr_enablebkp(true);
      g_stm32_rtc_resumed = (getreg32(STM32_RTC_BK1R) == RTC_MAGIC + 1);
      if (g_stm32_rtc_resumed)
        g_stm32_rtc_resumed_time_set = getreg32(STM32_RTC_BK2R);
      stm32_pwr_enablebkp(false);

      rtc_dumpregs("Did resume");
    }

  if (ret != OK && nretry > 0)
    {
      rtclldbg("setup/resume ran %d times and failed with %d\n",
                nretry, ret);
      return -ETIMEDOUT;
    }

  /* Configure RTC interrupt to catch alarm interrupts. All RTC interrupts are
   * connected to the EXTI controller.  To enable the RTC Alarm interrupt, the
   * following sequence is required:
   *
   * 1. Configure and enable the EXTI Line 17 in interrupt mode and select the
   *    rising edge sensitivity.
   * 2. Configure and enable the RTC_Alarm IRQ channel in the NVIC.
   * 3. Configure the RTC to generate RTC alarms (Alarm A or Alarm B).
   */

  g_rtc_enabled = true;
  rtc_dumpregs("After Initialization");
  return OK;
}

/************************************************************************************
 * Name: stm32_rtc_getdatetime_with_subseconds
 *
 * Description:
 *   Get the current date and time from the date/time RTC.  This interface
 *   is only supported by the date/time RTC hardware implementation.
 *   It is used to replace the system timer.  It is only used by the RTOS during
 *   initialization to set up the system time when CONFIG_RTC and CONFIG_RTC_DATETIME
 *   are selected (and CONFIG_RTC_HIRES is not).
 *
 *   NOTE: Some date/time RTC hardware is capability of sub-second accuracy.  That
 *   sub-second accuracy is returned through 'nsec'.
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *   nsec - The location to return the subsecond time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_HAVE_RTC_SUBSECONDS
int stm32_rtc_getdatetime_with_subseconds(FAR struct tm *tp, FAR long *nsec)
#else
int up_rtc_getdatetime(FAR struct tm *tp)
#endif
{
#ifdef CONFIG_STM32_HAVE_RTC_SUBSECONDS
  uint32_t ssr;
#endif
  uint32_t dr;
  uint32_t tr;
  uint32_t tmp;

  /* Sample the data time registers.  There is a race condition here... If we sample
   * the time just before midnight on December 31, the date could be wrong because
   * the day rolled over while were sampling. Thus loop for checking overflow here
   * is needed.  There is a race condition with subseconds too. If we sample TR
   * register just before second rolling and subseconds are read at wrong second,
   * we get wrong time.
   */

  do
    {
      dr  = getreg32(STM32_RTC_DR);
      tr  = getreg32(STM32_RTC_TR);
#ifdef CONFIG_STM32_HAVE_RTC_SUBSECONDS
      ssr = getreg32(STM32_RTC_SSR);
      tmp = getreg32(STM32_RTC_TR);
      if (tmp != tr)
        {
          continue;
        }
#endif
      tmp = getreg32(STM32_RTC_DR);
      if (tmp == dr)
        {
          break;
        }
    }
  while (1);

  rtc_dumpregs("Reading Time");

  /* Convert the RTC time to fields in struct tm format.  All of the STM32
   * All of the ranges of values correspond between struct tm and the time
   * register.
   */

  tmp = (tr & (RTC_TR_SU_MASK|RTC_TR_ST_MASK)) >> RTC_TR_SU_SHIFT;
  tp->tm_sec = rtc_bcd2bin(tmp);

  tmp = (tr & (RTC_TR_MNU_MASK|RTC_TR_MNT_MASK)) >> RTC_TR_MNU_SHIFT;
  tp->tm_min = rtc_bcd2bin(tmp);

  tmp = (tr & (RTC_TR_HU_MASK|RTC_TR_HT_MASK)) >> RTC_TR_HU_SHIFT;
  tp->tm_hour = rtc_bcd2bin(tmp);

  /* Now convert the RTC date to fields in struct tm format:
   * Days: 1-31 match in both cases.
   * Month: STM32 is 1-12, struct tm is 0-11.
   * Years: STM32 is 00-99, struct tm is years since 1900.
   * WeekDay: STM32 is 1 = Mon - 7 = Sun
   *
   * Issue:  I am not sure what the STM32 years mean.  Are these the
   * years 2000-2099?  I'll assume so.
   */

  tmp = (dr & (RTC_DR_DU_MASK|RTC_DR_DT_MASK)) >> RTC_DR_DU_SHIFT;
  tp->tm_mday = rtc_bcd2bin(tmp);

  tmp = (dr & (RTC_DR_MU_MASK|RTC_DR_MT)) >> RTC_DR_MU_SHIFT;
  tp->tm_mon = rtc_bcd2bin(tmp) - 1;

  tmp = (dr & (RTC_DR_YU_MASK|RTC_DR_YT_MASK)) >> RTC_DR_YU_SHIFT;
  tp->tm_year = rtc_bcd2bin(tmp) + 100;

#if defined(CONFIG_TIME_EXTENDED)
  tmp = (dr & RTC_DR_WDU_MASK) >> RTC_DR_WDU_SHIFT;
  tp->tm_wday = tmp % 7;
  tp->tm_yday = tp->tm_mday + clock_daysbeforemonth(tp->tm_mon, clock_isleapyear(tp->tm_year + 1900));
  tp->tm_isdst = 0
#endif

#ifdef CONFIG_STM32_HAVE_RTC_SUBSECONDS
  /* Return RTC sub-seconds if no configured and if a non-NULL value
   * of nsec has been provided to receive the sub-second value.
   */

  if (nsec)
    {
      uint32_t prediv_s;
      uint32_t usecs;

      prediv_s   = getreg32(STM32_RTC_PRER) & RTC_PRER_PREDIV_S_MASK;
      prediv_s >>= RTC_PRER_PREDIV_S_SHIFT;

      ssr &= RTC_SSR_MASK;

      /* Maximum prediv_s is 0x7fff, thus we can multiply by 100000 and
       * still fit 32-bit unsigned integer.
       */

      usecs = (((prediv_s - ssr) * 100000) / (prediv_s + 1)) * 10;
      *nsec = usecs * 1000;
    }
#endif /* CONFIG_STM32_HAVE_RTC_SUBSECONDS */

  rtc_dumptime(tp, "Returning");
  return OK;
}

/************************************************************************************
 * Name: up_rtc_getdatetime
 *
 * Description:
 *   Get the current date and time from the date/time RTC.  This interface
 *   is only supported by the date/time RTC hardware implementation.
 *   It is used to replace the system timer.  It is only used by the RTOS during
 *   initialization to set up the system time when CONFIG_RTC and CONFIG_RTC_DATETIME
 *   are selected (and CONFIG_RTC_HIRES is not).
 *
 *   NOTE: Some date/time RTC hardware is capability of sub-second accuracy.  That
 *   sub-second accuracy is lost in this interface.  However, since the system time
 *   is reinitialized on each power-up/reset, there will be no timing inaccuracy in
 *   the long run.
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_HAVE_RTC_SUBSECONDS
int up_rtc_getdatetime(FAR struct tm *tp)
{
  return stm32_rtc_getdatetime_with_subseconds(tp, NULL);
}
#endif

/************************************************************************************
 * Name: up_rtc_getdatetime_with_subseconds
 *
 * Description:
 *   Get the current date and time from the date/time RTC.  This interface
 *   is only supported by the date/time RTC hardware implementation.
 *   It is used to replace the system timer.  It is only used by the RTOS during
 *   initialization to set up the system time when CONFIG_RTC and CONFIG_RTC_DATETIME
 *   are selected (and CONFIG_RTC_HIRES is not).
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *   nsec - The location to return the subsecond time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_HAVE_RTC_SUBSECONDS
int up_rtc_getdatetime_with_subseconds(FAR struct tm *tp, FAR long *nsec)
{
  return stm32_rtc_getdatetime_with_subseconds(tp, nsec);
}
#endif

/************************************************************************************
 * Name: stm32_rtc_setdatetime
 *
 * Description:
 *   Set the RTC to the provided time. RTC implementations which provide
 *   up_rtc_getdatetime() (CONFIG_RTC_DATETIME is selected) should provide this
 *   function.
 *
 * Input Parameters:
 *   tp - the time to use
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

int stm32_rtc_setdatetime(FAR const struct tm *tp)
{
  uint32_t tr;
  uint32_t dr;
  int ret;

  rtc_dumptime(tp, "Setting time");

  /* Then write the broken out values to the RTC */

  /* Convert the struct tm format to RTC time register fields.  All of the STM32
   * All of the ranges of values correspond between struct tm and the time
   * register.
   */

  tr = (rtc_bin2bcd(tp->tm_sec)  << RTC_TR_SU_SHIFT) |
       (rtc_bin2bcd(tp->tm_min)  << RTC_TR_MNU_SHIFT) |
       (rtc_bin2bcd(tp->tm_hour) << RTC_TR_HU_SHIFT);
  tr &= ~RTC_TR_RESERVED_BITS;

  /* Now convert the fields in struct tm format to the RTC date register fields:
   * Days: 1-31 match in both cases.
   * Month: STM32 is 1-12, struct tm is 0-11.
   * Years: STM32 is 00-99, struct tm is years since 1900.
   * WeekDay: STM32 is 1 = Mon - 7 = Sun
   * Issue:  I am not sure what the STM32 years mean.  Are these the
   * years 2000-2099?  I'll assume so.
   */

  dr = (rtc_bin2bcd(tp->tm_mday) << RTC_DR_DU_SHIFT) |
       ((rtc_bin2bcd(tp->tm_mon + 1))  << RTC_DR_MU_SHIFT) |
#if defined(CONFIG_TIME_EXTENDED)
       ((tp->tm_wday == 0 ? 7 : (tp->tm_wday & 7))  << RTC_DR_WDU_SHIFT) |
#endif
       ((rtc_bin2bcd(tp->tm_year - 100)) << RTC_DR_YU_SHIFT);

  dr &= ~RTC_DR_RESERVED_BITS;

  /* Disable the write protection for RTC registers */

  rtc_wprunlock();

  /* Set Initialization mode */

  ret = rtc_enterinit();
  if (ret == OK)
    {
      /* Set the RTC TR and DR registers */

      putreg32(tr, STM32_RTC_TR);
      putreg32(dr, STM32_RTC_DR);

      /* Exit Initialization mode and wait for the RTC Time and Date
       * registers to be synchronized with RTC APB clock.
       */

      rtc_exitinit();
      ret = rtc_synchwait();
    }

  /* Re-enable the write protection for RTC registers */

  rtc_wprlock();
  rtc_dumpregs("New time setting");
  return ret;
}

/************************************************************************************
 * Name: up_rtc_settime
 *
 * Description:
 *   Set the RTC to the provided time.  All RTC implementations must be able to
 *   set their time based on a standard timespec.
 *
 * Input Parameters:
 *   tp - the time to use
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

int up_rtc_settime(FAR const struct timespec *tp)
{
  FAR struct tm newtime;
  int ret;

  /* Break out the time values (not that the time is set only to units of seconds) */

  (void)gmtime_r(&tp->tv_sec, &newtime);
  ret = stm32_rtc_setdatetime(&newtime);

  if (ret == OK)
    {
      /*
       * Save mark in backup register so we know that RTC clock was supplied
       * by application.
       */

      stm32_pwr_enablebkp(true);
      putreg32(RTC_MAGIC + 1, STM32_RTC_BK1R);
      putreg32(tp->tv_sec, STM32_RTC_BK2R);
      stm32_pwr_enablebkp(false);

      DEBUGASSERT(sizeof(tp->tv_sec) == 4);
    }

  return ret;
}

/************************************************************************************
 * Name: stm32_rtc_setalarm
 *
 * Description:
 *   Set up an alarm.  Up to two alarms can be supported (ALARM A and ALARM B).
 *
 * Input Parameters:
 *   tp - the time to set the alarm
 *   callback - the function to call when the alarm expires.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

#ifdef CONFIG_RTC_ALARM
int stm32_rtc_setalarm(FAR const struct timespec *tp, alarmcb_t callback)
{
  int ret = OK;
  int timeout = 0;
  uint32_t regval = 0;
  time_t secs = (time_t) tp->tv_sec;


  rtclldbg("Set alarm interrupt\n");

  rtc_wprunlock();

  /* Clear ALRBE or ALRAE(depends on CONFIG_RTC_USE_ALARM_A) in RTC_CR to disable an alarm */

  regval = getreg32(STM32_RTC_CR);
#ifdef CONFIG_RTC_USE_ALARM_A
  regval &= ~RTC_CR_ALRAE;

  /* Poll ALRAWF until it is set in RTC_ISR (takes around 2 RTCCLK clock cycles) */

  ret = -ETIMEDOUT;
  for (timeout = 0; timeout < SYNCHRO_TIMEOUT; timeout++)
        {
          regval = getreg32(STM32_RTC_ISR);
          if ((regval & RTC_ISR_ALRAWF) != 0)
                {
                  /* Synchronized */

                  ret = OK;
                  break;
                }
        }
#else
  regval &= ~RTC_CR_ALRBE;

  /* Poll ALRBWF until it is set in RTC_ISR (takes around 2 RTCCLK clock cycles) */

  ret = -ETIMEDOUT;
  for (timeout = 0; timeout < SYNCHRO_TIMEOUT; timeout++)
        {
          regval = getreg32(STM32_RTC_ISR);
          if ((regval & RTC_ISR_ALRBWF) != 0)
                {
                  /* Synchronized */

                  ret = OK;
                  break;
                }
        }
#endif

  rtclldbg("RTC_CR: 0x%08x\n", regval);
  putreg32(regval, STM32_RTC_CR);

  if (g_alarmcb == NULL)
        {
          g_alarmcb = callback;
        }

  /* Program the Alarm A or Alarm B registers (RTC_ALRMASSR/RTC_ALRMAR or RTC_ALRMBSSR/RTC_ALRMBR) */

  (void)rtc_write_alarm_value(secs);

  stm32_exti_alarm(true, false, false, rtc_interrupt);

  /* Set ALRAE or ALRBE in the RTC_CR register to enable Alarm A or Alarm B again */

  regval = getreg32(STM32_RTC_CR);

#ifdef CONFIG_RTC_USE_ALARM_A
  regval |= RTC_CR_ALRAE | RTC_CR_ALRAIE;
#else
  regval |= RTC_CR_ALRBE | RTC_CR_ALRBIE;
#endif

  putreg32(regval, STM32_RTC_CR);

  /* Just in case resets the ALRAF/ALRBF flag in RTC_ISR */
  regval = getreg32(STM32_RTC_ISR);

#ifdef CONFIG_RTC_USE_ALARM_A
  regval &= ~RTC_ISR_ALRAF;
#else
  regval &= ~RTC_ISR_ALRBF;
#endif

  putreg32(regval, STM32_RTC_ISR);

  rtc_wprlock();

  rtc_dumpregs("IN ALARM REGISTERS");

  return ret;
}
#endif

/************************************************************************************
 * Name: up_rtc_turnoff_alarm_int
 *
 * Description:
 *   Disable an alarm.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

#ifdef CONFIG_RTC_ALARM
int up_rtc_turnoff_alarm_int(void)
{
  int ret = OK;
  int timeout = 0;
  uint32_t regval = 0;

  rtclldbg("Turns off autoreload wakeup\n");

  stm32_exti_alarm(true, false, false, NULL);

  rtc_wprunlock();

  regval = getreg32(STM32_RTC_CR);
#ifdef CONFIG_RTC_USE_ALARM_A
  regval &= ~RTC_CR_ALRAE;

  /* Poll ALRAWF until it is set in RTC_ISR (takes around 2 RTCCLK clock cycles) */

  ret = -ETIMEDOUT;
  for (timeout = 0; timeout < SYNCHRO_TIMEOUT; timeout++)
        {
          regval = getreg32(STM32_RTC_ISR);
          if ((regval & RTC_ISR_ALRAWF) != 0)
                {
                  /* Synchronized */

                  ret = OK;
                  break;
                }
        }
#else
  regval &= ~RTC_CR_ALRBE;

  /* Poll ALRBWF until it is set in RTC_ISR (takes around 2 RTCCLK clock cycles) */

  ret = -ETIMEDOUT;
  for (timeout = 0; timeout < SYNCHRO_TIMEOUT; timeout++)
        {
          regval = getreg32(STM32_RTC_ISR);
          if ((regval & RTC_ISR_ALRBWF) != 0)
                {
                  /* Synchronized */

                  ret = OK;
                  break;
                }
        }
#endif
  rtclldbg("RTC_CR: 0x%08x\n", regval);
  putreg32(regval, STM32_RTC_CR);

  rtc_wprlock();

  return ret;
}
#endif

#endif /* CONFIG_RTC */
