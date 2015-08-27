/************************************************************************************
 * configs/haltian-tsone/src/up_boot.c
 * arch/arm/src/board/up_boot.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#include <debug.h>
#include <stdio.h>

#include <nuttx/board.h>
#include <arch/board/board.h>
#include <arch/board/board-pwrctl.h>
#include <arch/board/board-reset.h>
#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>

#include "up_arch.h"
#include "haltian-tsone.h"
#include "up_chipselect.h"
#include "up_modem.h"
#include "up_gpio.h"

#include "stm32_flash.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) lowsyslog(__VA_ARGS__)
#  else
#    define message(...) printf(__VA_ARGS__)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message lowsyslog
#  else
#    define message printf
#  endif
#endif

/************************************************************************************
 * Definitions
 ************************************************************************************/

#if defined(CONFIG_ARCH_CHIP_STM32L162VE) && \
    defined(CONFIG_BOARD_ENABLE_FIX_FOR_INVALID_OPTION_BYTES)
#  define FLASH_KEY1      0x8C9DAEBF
#  define FLASH_KEY2      0x13141516

#  define EEPROM_KEY1     0x89ABCDEF
#  define EEPROM_KEY2     0x02030405

#  define FLASH_OPTKEY1   0xFBEAD9C8
#  define FLASH_OPTKEY2   0x24252627
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

#if defined(CONFIG_ARCH_CHIP_STM32L162VE) && \
    defined(CONFIG_BOARD_ENABLE_FIX_FOR_INVALID_OPTION_BYTES)

static void stm32_eeprom_unlock(void)
{
  while (getreg32(STM32_FLASH_SR) & FLASH_SR_BSY);

  if (getreg32(STM32_FLASH_PECR) & FLASH_PECR_PELOCK)
    {
      /* Unlock sequence */

      putreg32(EEPROM_KEY1, STM32_FLASH_PEKEYR);
      putreg32(EEPROM_KEY2, STM32_FLASH_PEKEYR);
    }
}

static void stm32_eeprom_lock(void)
{
  modifyreg32(STM32_FLASH_PECR, 0, FLASH_PECR_PELOCK);
}

static void stm32_ob_unlock(void)
{
  stm32_eeprom_unlock();

  /* Unlocking the option bytes block access */
  putreg32(FLASH_OPTKEY1, STM32_FLASH_OPTKEYR);
  putreg32(FLASH_OPTKEY2, STM32_FLASH_OPTKEYR);
}

static void stm32_ob_lock(void)
{
  modifyreg32(STM32_FLASH_PECR, 0, FLASH_PECR_OPTLOCK);
  stm32_eeprom_lock();
}

static void up_check_and_restore_valid_optionbytes(void)
{
  static const uint32_t reset_optb[][2] =  {
    /* Default option bytes for STM32L162VE. */
    { 0x1FF80004, 0xFF0700F8 },
    { 0x1FF80008, 0xFFFF0000 },
    { 0x1FF8000C, 0xFFFF0000 },
    { 0x1FF80010, 0xFFFF0000 },
    { 0x1FF80014, 0xFFFF0000 },
    { 0x1FF80018, 0xFFFF0000 },
    { 0x1FF8001C, 0xFFFF0000 },
    { 0x1FF80080, 0xFFFF0000 },
    { 0x1FF80084, 0xFFFF0000 },
    { 0x1FF80000, 0xFF5500AA },
    { 0x0, 0x0 },
  };
  int i;
  char buf[64];
  const char *str;
  int modcount = 0;

  if (!(getreg32(STM32_FLASH_SR) & (FLASH_SR_OPTVERR | FLASH_SR_OPTVERRUSR)))
    return;

  snprintf(buf, sizeof(buf), "\nSTM32_FLASH_SR: %08X\n", getreg32(STM32_FLASH_SR));

  str = buf;
  while (*str)
    up_lowputc(*str++);

  for (i = 0; reset_optb[i][0] != 0x0; i++)
    {
      snprintf(buf, sizeof(buf), "%08X: %08X\n",
               reset_optb[i][0],
               *(const uint32_t *)reset_optb[i][0]);

      str = buf;
      while (*str)
        up_lowputc(*str++);
    }

  stm32_ob_unlock();

  /* Clear pending status flags. */

  putreg32(FLASH_SR_WRPERR | FLASH_SR_PGAERR |
           FLASH_SR_SIZERR | FLASH_SR_OPTVERR |
           FLASH_SR_OPTVERRUSR | FLASH_SR_RDERR, STM32_FLASH_SR);

  for (i = 0; reset_optb[i][0] != 0x0; i++)
    {
      if (*(const uint32_t *)reset_optb[i][0] == reset_optb[i][1])
        continue;

      snprintf(buf, sizeof(buf), "Programming %08X: %08X => %08X\n",
               reset_optb[i][0],
               *(const uint32_t *)reset_optb[i][0],
               reset_optb[i][1]);

      str = buf;
      while (*str)
        up_lowputc(*str++);

      putreg32(reset_optb[i][1], reset_optb[i][0]);

      /* ... and wait to complete. */

      while (getreg32(STM32_FLASH_SR) & FLASH_SR_BSY);

      /* Verify */

      if (getreg32(STM32_FLASH_SR) & (FLASH_SR_WRPERR | FLASH_SR_PGAERR |
                                      FLASH_SR_SIZERR | FLASH_SR_RDERR))
        {
          snprintf(buf, sizeof(buf), "Failed to program %08X!\n",
                   reset_optb[i][0]);

          str = buf;
          while (*str)
            up_lowputc(*str++);

          goto err;
        }

      if (getreg32(reset_optb[i][0]) != reset_optb[i][1])
        {
          snprintf(buf, sizeof(buf), "Failed to verify %08X, reads %08X.\n",
                   reset_optb[i][0], getreg32(reset_optb[i][0]));

          str = buf;
          while (*str)
            up_lowputc(*str++);

          goto err;
        }

      modcount++;
    }

  putreg32(FLASH_SR_OPTVERR | FLASH_SR_OPTVERRUSR, STM32_FLASH_SR);

  stm32_ob_lock();

  if (modcount == 0)
    return;

err:
  while (1)
    up_systemreset();
}

#else

static void up_check_and_restore_valid_optionbytes(void)
{
  /* Do nothing. */
}

#endif

static void print_reset_reason(void)
{
  uint32_t breason;

  breason = board_reset_get_reason(false);

  up_lowputc('_');
  if (breason & BOARD_RESET_REASON_LOW_POWER)
    up_lowputc('L');
  if (breason & BOARD_RESET_REASON_WINDOW_WATCHDOG)
    up_lowputc('W');
  if (breason & BOARD_RESET_REASON_INDEPENDENT_WATCHDOG)
    up_lowputc('I');
  if (breason & BOARD_RESET_REASON_SOFTWARE)
    up_lowputc('S');
  if (breason & BOARD_RESET_REASON_POR_PDR)
    up_lowputc('P');
  if (breason & BOARD_RESET_REASON_NRST_PIN)
    up_lowputc('R');
  if (breason & BOARD_RESET_REASON_OPTIONS_BYTES_LOADING)
    up_lowputc('O');
  if (breason & BOARD_RESET_REASON_HARDFAULT)
    up_lowputc('H');
  if (breason & BOARD_RESET_REASON_STANDBY_WAKEUP)
    up_lowputc('^');
  up_lowputc('_');
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the initialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

void stm32_boardinitialize(void)
{
#ifdef BOARD_HAS_BOOTLOADER
  /* Check if IWDG is enabled by bootloader. This is the case when
   * CONFIG_BOOTLOADER_NOWD_BKREG has lowest bit cleared.
   */

  if ((getreg32(CONFIG_BOOTLOADER_NOWD_BKREG) & 1) == 0)
    {
      int i;

      /* Bootloader has enabled IWDG. This happens after POR-reset (since
       * backup-registers have been reset to zero) and after firmware update.
       *
       * Disabling IWDG is used as early boot firmware check (new firmware
       * started). We now enable NOWD flag and do reset loop with ADDR value
       * set to application firmware.
       *
       * If firmware does not manage to do early boot and gets stuck, IWDG will
       * reset device and bootloader will either attempt to flash backup
       * firmware or will reset loop by IWDG until 'current boot try count'
       * reaches threshold and bootloader boots device to DFU-mode.
       */

      for (i = 0; i < 4; i++)
        up_lowputc('>');

      /* Enable BKREG writing. */

      stm32_pwr_enablebkp(true);

      /* Make bootloader disable IWDG. */

      putreg32(1, CONFIG_BOOTLOADER_NOWD_BKREG);

      /* Setup bootloader to jump directly to firmware. */

      putreg32(BOARD_FIRMWARE_BASE_ADDR, CONFIG_BOOTLOADER_ADDR_BKREG);

      /* Disable BKREG writing. */

      stm32_pwr_enablebkp(false);

      /* Do system reset. */

      board_systemreset();
    }
#endif /* BOARD_HAS_BOOTLOADER */

#if defined(CONFIG_ASSERT_COUNT_BKREG)
  /* Check value of assert counter (note: does not enable/disable BKREG access
   * with argument==0). */

  if (up_add_assert_count(0) >= CONFIG_ASSERT_COUNT_TO_DFU_MODE)
    {
      const char *str = "\nToo many ASSERT resets; something wrong with software. Forcing DFU mode!\n";

      while (*str)
        up_lowputc(*str++);

      up_mdelay(500);
      up_reset_to_system_bootloader();
    }
#endif

  /* Setup GPIOs based on HW version. */

  up_configure_dynamic_gpios();

  /* Configure MCU so that debugging is possible in idle modes. */

#ifdef CONFIG_STM32_KEEP_CORE_CLOCK_ENABLED_IN_IDLE_MODES
  uint32_t cr = getreg32(STM32_DBGMCU_CR);

  cr |= DBGMCU_CR_STANDBY | DBGMCU_CR_STOP | DBGMCU_CR_SLEEP;
  putreg32(cr, STM32_DBGMCU_CR);
#endif

#ifdef CONFIG_BOARD_MCO_SYSCLK
  /* Output SYSCLK to MCO pin for clock measurements. */

  stm32_configgpio(GPIO_MCO);
  stm32_mcodivconfig(RCC_CFGR_MCOSEL_SYSCLK, RCC_CFGR_MCOPRE_DIV4);
#endif

  /* Configure on-board LEDs if LED support has been selected. */

#ifdef CONFIG_ARCH_LEDS
  board_led_initialize();
#endif

  /* Configure chip-select pins. */

  board_initialize_chipselects();

  /* Configure SDcard pins (needs to be after chip-select init, to pull SDcard
   * CS down). */

  gpio_initialize_sdcard_pins();

  /* Configure power control pins. */

  board_initialize_pwrctl_pins();

  /* Initialize modem gpios. */

  up_modem_initialize_gpios();

  /* Initialize unused gpio pads. */

  gpio_initialize_unused_pads();

  /* Enable BKREG writing. */

  stm32_pwr_enablebkp(true);

  /* Reset 'current boot try count' for bootloader. */

  putreg32(0, CONFIG_BOOTLOADER_CBTC_BKREG);

  /* Make bootloader disable IWDG. */

  putreg32(1, CONFIG_BOOTLOADER_NOWD_BKREG);

  /* Check if we need to enter standby/power-off mode. */

  up_boot_standby_mode();

  /* Disable BKREG writing. */

  stm32_pwr_enablebkp(false);

  /* Force enable capsense and 9-axis for duration
   * of I2C initialization. After bus initialization
   * is done, this is undone and drivers will take care
   * of requesting power for themselves.
   */

  board_pwrctl_get(PWRCTL_SWITCH_CAPSENSE_SENSOR);
  board_pwrctl_get(PWRCTL_SWITCH_9AXIS_INERTIAL_SENSOR);

  /* Flash mass-erase can leave option-bytes is bad shape, restore defaults if
   * needed. */

  up_check_and_restore_valid_optionbytes();

  /* Configure SPI chip selects if 1) SPI is not disabled, and 2) the weak function
   * stm32_spiinitialize() has been brought into the link.
   */

#if defined(CONFIG_STM32_SPI1) || defined(CONFIG_STM32_SPI2) || defined(CONFIG_STM32_SPI3)
  if (stm32_spiinitialize)
    {
      stm32_spiinitialize();
    }
#endif

  /* Give early information about reset reason. */

  print_reset_reason();
}

/****************************************************************************
 * Name: board_initialize
 *
 * Description:
 *   If CONFIG_BOARD_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_initialize().  board_initialize() will be
 *   called immediately after up_intiialize() is called and just before the
 *   initial application is started.  This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_INITIALIZE
void board_initialize(void)
{
#ifdef BOARD_HAS_BOOTLOADER
  dbg("Bootloader status:\n");
  dbg(" ADDR: %08X\n", getreg32(CONFIG_BOOTLOADER_ADDR_BKREG));
  dbg(" BRSN: %08X\n", getreg32(CONFIG_BOOTLOADER_BRSN_BKREG));
  dbg(" NOWD: %08X\n", getreg32(CONFIG_BOOTLOADER_NOWD_BKREG));
  dbg(" FWUS: %08X\n", getreg32(CONFIG_BOOTLOADER_FWUS_BKREG));
  dbg(" CBTC: %08X\n", getreg32(CONFIG_BOOTLOADER_CBTC_BKREG));
  dbg(" CUTC: %08X\n", getreg32(CONFIG_BOOTLOADER_CUTC_BKREG));
  dbg(" RES0: %08X\n", getreg32(CONFIG_BOOTLOADER_RESERVED0_BKREG));
#endif

  /* Perform board initialization */

  (void)up_bringup();
}
#endif /* CONFIG_BOARD_INITIALIZE */
