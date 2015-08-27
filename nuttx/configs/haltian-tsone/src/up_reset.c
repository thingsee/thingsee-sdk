/****************************************************************************
 * config/haltian-tsone/src/up_reset.c
 * arch/arm/src/board/up_reset.c
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
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <nuttx/arch.h>

#include <arch/board/board.h>
#include <arch/board/board-reset.h>
#include <arch/board/board-gpio.h>
#include <arch/board/board-pwrctl.h>
#include <arch/board/board-eeprom.h>

#include "haltian-tsone.h"
#include "up_chipselect.h"
#include "up_gpio.h"

#include "stm32.h"
#include "stm32_rcc.h"
#include "stm32_rtc.h"
#include "stm32_pm.h"
#include "nvic.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifndef BOARD_HAS_BOOTLOADER
static void rcc_deinit(void)
{
  uint32_t regval;

  /* Make sure that all devices are out of reset */

  putreg32(0, STM32_RCC_AHBRSTR);
  putreg32(0, STM32_RCC_APB2RSTR);
  putreg32(0, STM32_RCC_APB1RSTR);

  /* Disable all clocking (other than to FLASH) */

  putreg32(RCC_AHBENR_FLITFEN, STM32_RCC_AHBENR);
  putreg32(0, STM32_RCC_APB2ENR);
  putreg32(0, STM32_RCC_APB1ENR);

  /* Set MSION bit */

  regval = getreg32(STM32_RCC_CR);
  regval |= RCC_CR_MSION;
  putreg32(regval, STM32_RCC_CR);
  //while ((getreg32(STM32_RCC_CFGR) & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_MSI);

  /* Reset SW[1:0], HPRE[3:0], PPRE1[2:0], PPRE2[2:0], MCOSEL[2:0] and MCOPRE[2:0] bits */

  regval = getreg32(STM32_RCC_CFGR);
  regval &= ~(RCC_CFGR_SW_MASK | RCC_CFGR_HPRE_MASK | RCC_CFGR_PPRE1_MASK |
              RCC_CFGR_PPRE2_MASK | RCC_CFGR_MCOSEL_MASK | RCC_CFGR_MCOPRE_MASK);
  putreg32(regval, STM32_RCC_CFGR);

  /* Reset HSION, HSEON, CSSON and PLLON bits */

  regval = getreg32(STM32_RCC_CR);
  regval &= ~(RCC_CR_HSION | RCC_CR_HSEON | RCC_CR_CSSON | RCC_CR_PLLON);
  putreg32(regval, STM32_RCC_CR);

  /* Reset HSEBYP bit */

  regval = getreg32(STM32_RCC_CR);
  regval &= ~(RCC_CR_HSEBYP);
  putreg32(regval, STM32_RCC_CR);

  /* Reset PLLSRC, PLLMUL[3:0] and PLLDIV[1:0] bits */

  regval  = getreg32(STM32_RCC_CFGR);
  regval &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL_MASK | RCC_CFGR_PLLDIV_MASK);
  putreg32(regval, STM32_RCC_CFGR);

  /* Disable all interrupts */

  putreg32(0, STM32_RCC_CIR);

  /* Go to the (default) voltage range 2 */

  stm32_pwr_setvos(PWR_CR_VOS_SCALE_2);

  /* Reset the FLASH controller to 32-bit mode, no wait states.
   *
   * First, program the new number of WS to the LATENCY bit in Flash access
   * control register (FLASH_ACR)
   */

  regval  = getreg32(STM32_FLASH_ACR);
  regval &= ~FLASH_ACR_LATENCY;
  putreg32(regval, STM32_FLASH_ACR);

  /* Check that the new number of WS is taken into account by reading FLASH_ACR */

  /* Program the 32-bit access by clearing ACC64 in FLASH_ACR */

  regval &= ~FLASH_ACR_ACC64;
  putreg32(regval, STM32_FLASH_ACR);
}
#endif /* CONFIG_BOARD_NO_BOOTLOADER */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_rtc_time_is_set
 *
 * Description:
 *   Is the RTC clock set.
 *
 ****************************************************************************/

bool board_rtc_time_is_set(time_t *when_was_set)
{
  time_t time = getreg32(STM32_RTC_BK2R);

  if (!time)
    return false;

  if (when_was_set != NULL)
    {
      *when_was_set = time;
    }

  return true;
}

/****************************************************************************
 * Name: board_reset_get_reason
 *
 * Description:
 *   Get reset reason flags, and clear reset flags from HW.
 *
 ****************************************************************************/

uint32_t board_reset_get_reason(bool clear)
{
  uint32_t regval;
  uint32_t reason = 0;

  /* Check reset flags. */

  regval = getreg32(STM32_RCC_CSR);

  if (regval & RCC_CSR_LPWRRSTF)
    reason |= BOARD_RESET_REASON_LOW_POWER;
  if (regval & RCC_CSR_WWDGRSTF)
    reason |= BOARD_RESET_REASON_WINDOW_WATCHDOG;
  if (regval & RCC_CSR_IWDGRSTF)
    reason |= BOARD_RESET_REASON_INDEPENDENT_WATCHDOG;
  if (regval & RCC_CSR_SFTRSTF)
    reason |= BOARD_RESET_REASON_SOFTWARE;
  if (regval & RCC_CSR_PORRSTF)
    reason |= BOARD_RESET_REASON_POR_PDR;
  if (regval & RCC_CSR_PINRSTF)
    reason |= BOARD_RESET_REASON_NRST_PIN;
  if (regval & RCC_CSR_OBLRSTF)
    reason |= BOARD_RESET_REASON_OPTIONS_BYTES_LOADING;

#if defined(CONFIG_HARDFAULT_MAGIC_BKREG) && defined(CONFIG_HARDFAULT_MAGIC)
  /* Check hard-fault magic from backup register. */

  regval = getreg32(CONFIG_HARDFAULT_MAGIC_BKREG);
  if (regval == CONFIG_HARDFAULT_MAGIC)
    {
      reason |= BOARD_RESET_REASON_HARDFAULT;

      if (clear)
        {
          DEBUGASSERT((getreg32(STM32_PWR_CR) & PWR_CR_DBP) == 0);
          stm32_pwr_enablebkp(true);
          putreg32(0x0, CONFIG_HARDFAULT_MAGIC_BKREG);
          stm32_pwr_enablebkp(false);
        }
    }
#endif

#if defined(CONFIG_STANDBYMODE_MAGIC_BKREG)
  if (g_board_wakeup_from_standby)
    reason |= BOARD_RESET_REASON_STANDBY_WAKEUP;
#endif

  /* Clear reset flags. */

  if (clear)
    modifyreg32(STM32_RCC_CSR, 0, RCC_CSR_RMVF);

  return reason;
}

/****************************************************************************
 * Name: up_add_assert_count
 *
 * Description:
 *   Increase count in assert counter
 *
 ****************************************************************************/

#if defined(CONFIG_ASSERT_COUNT_BKREG)
unsigned int up_add_assert_count(unsigned int val)
{
  uint32_t regval;
  uint32_t count;

  /* Read backup register. */

  regval = getreg32(CONFIG_ASSERT_COUNT_BKREG);

  /* Check if assert count is set, and extract count if it is. */
  if ((regval & CONFIG_ASSERT_COUNT_MAGIC_MASK) == CONFIG_ASSERT_COUNT_MAGIC)
    count = regval & CONFIG_ASSERT_COUNT_MASK;
  else
    count = 0;

  /* Update count. */

  count = (count + val) & CONFIG_ASSERT_COUNT_MASK;
  if ((count | CONFIG_ASSERT_COUNT_MAGIC) != regval)
    {
      DEBUGASSERT((getreg32(STM32_PWR_CR) & PWR_CR_DBP) == 0);
      stm32_pwr_enablebkp(true);
      putreg32(count | CONFIG_ASSERT_COUNT_MAGIC, CONFIG_ASSERT_COUNT_BKREG);
      stm32_pwr_enablebkp(false);
    }

  return count;
}
#endif

/****************************************************************************
 * Name: board_reset_assert_count
 *
 * Description:
 *   Reset assert counter
 *
 ****************************************************************************/

void board_reset_assert_count(void)
{
#if defined(CONFIG_ASSERT_COUNT_BKREG)
  DEBUGASSERT((getreg32(STM32_PWR_CR) & PWR_CR_DBP) == 0);
  stm32_pwr_enablebkp(true);
  putreg32(0x0, CONFIG_ASSERT_COUNT_BKREG);
  stm32_pwr_enablebkp(false);
#endif
}

/****************************************************************************
 * Name: up_set_hardfault_magic
 *
 * Description:
 *   Set hard-fault magic to backup register.
 *
 ****************************************************************************/

#if defined(CONFIG_HARDFAULT_MAGIC_BKREG) && defined(CONFIG_HARDFAULT_MAGIC)
void up_set_hardfault_magic(void)
{
  DEBUGASSERT((getreg32(STM32_PWR_CR) & PWR_CR_DBP) == 0);
  stm32_pwr_enablebkp(true);
  putreg32(CONFIG_HARDFAULT_MAGIC, CONFIG_HARDFAULT_MAGIC_BKREG);
  stm32_pwr_enablebkp(false);
}
#endif

/****************************************************************************
 * Name: board_crashdump
 *
 * Description:
 *   If CONFIG_BOARD_CRASHDUMP is selected then up_asseert will call out to
 *   board_crashdump prior to calling exit in the case of an assertion failure.
 *   Or in the case of a hardfault looping indefinitely. board_crashdump then
 *   has a chance to save the state of the machine. The provided
 *   board_crashdump should save as much information as it can about the cause
 *   of the fault and then most likely reset the system.
 *
 ****************************************************************************/

void board_crashdump(uint32_t currentsp, void *tcb, const uint8_t *filename,
                     int lineno)
{
  struct tcb_s *rtcb = (struct tcb_s*)tcb;
  char buf[80];

#if defined(CONFIG_ASSERT_COUNT_BKREG)
  up_add_assert_count(1);
#endif

  snprintf(buf, sizeof(buf), "!file:%s line:%d task:<0x%08lx>:%s\n",
          filename, lineno, (unsigned long)rtcb->entry.main,
#if CONFIG_TASK_NAME_SIZE > 0
          rtcb->name
#else
          NULL
#endif
          );

  board_eeprom_write_section(BOARD_EEPROM_SECTION_ENGINE_ASSERT_CRASH_DATA, 0,
                             buf, strlen(buf) + 1);

  up_mdelay(10);

  board_systemreset();
}

/****************************************************************************
 * Name: board_systemreset
 *
 * Description:
 *   Perform MCU reset.
 *
 ****************************************************************************/

void board_systemreset(void)
{
  /* SDCard needs to be left powered-off state for short time. Otherwise
   * bootloader has trouble mounting filesystem.
   */

  gpio_initialize_sdcard_pins();
  up_mdelay(10);

  /* Clear standby flags before reseting. */

  modifyreg32(STM32_PWR_CR, PWR_CR_CSBF | PWR_CR_CWUF, 0);

  /* Clear reset flags. */

  modifyreg32(STM32_RCC_CSR, 0, RCC_CSR_RMVF);

  /* Do system reset. */

  up_systemreset();
}

/****************************************************************************
 * Name: board_reset_to_system_bootloader
 *
 * Description:
 *   Perform MCU reset to system bootloader / DFU flash mode (from userspace)
 *
 ****************************************************************************/

void board_reset_to_system_bootloader(void)
{
  dbg("Reseting device to USB DFU bootloader mode...\n");
  usleep(100 * 1000);

  sched_lock();

  /* Power-off display. */
#ifdef CONFIG_THINGSEE_DISPLAY_MODULE
  board_lcdoff();
#endif
  (void)irqsave();

  up_reset_to_system_bootloader();
}

/****************************************************************************
 * Name: up_reset_to_system_bootloader
 *
 * Description:
 *   Perform MCU reset to system bootloader / DFU flash mode (from kernel)
 *
 ****************************************************************************/

void up_reset_to_system_bootloader(void)
{
#ifdef BOARD_HAS_BOOTLOADER

  /* Reset assert count to avoid rentry to DFU. */

  board_reset_assert_count();

  /* Reset to DFU mode through bootloader. */

  DEBUGASSERT((getreg32(STM32_PWR_CR) & PWR_CR_DBP) == 0);
  stm32_pwr_enablebkp(true);
  putreg32(BOARD_SYSMEM_BASE_ADDR, CONFIG_BOOTLOADER_ADDR_BKREG);
  stm32_pwr_enablebkp(false);

  board_systemreset();

#else /* CONFIG_BOARD_NO_BOOTLOADER */

  const uintptr_t sysmem_base = BOARD_SYSMEM_BASE_ADDR;
  const char *sysmem = (const char *)sysmem_base;
  void (* const sysmem_resetvector)(void) = *(const void **)(sysmem + 4);
  uint32_t regval = 0;

  board_reset_assert_count();
  up_mdelay(10);

  /*
   * http://www.youtube.com/watch?v=cvKC-4tCRgw
   *
   * Calling the SystemMemory Bootloader:
   * 1. Shutdown any tasks.
   * 2. Switch to MSI.
   * 3. Disable interrupts.
   * 4. Set main stack pointer to its' default value.
   * 5. Load the program counter with the SystemMemory reset vector
   */

#if defined(CONFIG_USBDEV) && defined (CONFIG_STM32_USB)
  /*
   * Additionally we need to turn off USB and wait briefly for host to detect
   * USB disconnect.
   */

  /* Reset USB peripheral */

  regval = getreg32(STM32_RCC_APB1RSTR);
  putreg32(regval | RCC_APB1RSTR_USBRST, STM32_RCC_APB1RSTR);
  putreg32(regval, STM32_RCC_APB1RSTR);

  up_mdelay(100);
#else
  (void)regval;
#endif

  /* Clear standby flags before going to DFU mode. */

  modifyreg32(STM32_PWR_CR, PWR_CR_CSBF | PWR_CR_CWUF, 0);

  /* Clear reset flags. */

  modifyreg32(STM32_RCC_CSR, 0, RCC_CSR_RMVF);

  rcc_deinit();

  putreg32(0, NVIC_SYSTICK_CTRL);
  putreg32(0, NVIC_SYSTICK_RELOAD);
  putreg32(0, NVIC_SYSTICK_CURRENT);

  /* Set Main Stack Pointer to default value. */
  __asm__ __volatile__
    (
      "\tmsr msp, %0\n"
      :
      : "r" (0x20001000)
      : "memory");

  /* Jump to SystemMemory Bootloader. */

  sysmem_resetvector();

  while (true);
#endif /* CONFIG_BOARD_NO_BOOTLOADER */
}
