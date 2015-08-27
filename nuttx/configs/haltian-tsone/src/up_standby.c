/****************************************************************************
 * configs/haltian-tsone/src/up_standby.c
 * arch/arm/src/board/up_standby.c
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
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
#include <debug.h>
#include <sched.h>
#include <nuttx/arch.h>
#include <nuttx/mmcsd.h>

#include <arch/board/board-device.h>
#include <arch/board/board-gpio.h>
#include <arch/board/board-reset.h>
#include <arch/board/board.h>
#include "chip.h"

#include "stm32.h"
#include "stm32_pm.h"
#include "stm32_uart.h"
#include "haltian-tsone.h"
#include "up_gpio.h"

#ifdef CONFIG_STM32_DFU
#include <up_internal.h>
#endif

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

bool g_board_wakeup_from_standby = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void standby_do_trace(char ch)
{
  int i;

  /* Stop mode disables HSE/HSI/PLL and wake happens with default system
   * clock (MSI, 2Mhz). Reconfigure clocks to enable USART2.
   */

  stm32_clockconfig();

  /* Stop mode exit traces. */

  for (i = 0; i < 4; i++)
    up_lowputc(ch);
}

static void hwwdg_irq_in_standby(void)
{
  /* Debug trace. */

  standby_do_trace('W');

  /* We kick watchdog here as no application is running,
     but device is considered functional during standby. */

  board_hwwdg_kick();
}

static void clear_rst_stdby_flags(void)
{
  /* Clear standby flags before reseting. */

  modifyreg32(STM32_PWR_CR, PWR_CR_CSBF | PWR_CR_CWUF, 0);

  /* Clear reset flags. */

  modifyreg32(STM32_RCC_CSR, 0, RCC_CSR_RMVF);
}

static int button_pressed_down_handler(int irq, FAR void *context)
{
  standby_do_trace('B');

  clear_rst_stdby_flags();

  /* Do system reset. */

  up_systemreset();

  for(;;);
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_go_to_standby
 *
 * Description:
 *   Enter low-power standby mode. Note: this is done through reset with
 *   RTC backup register used to mark special boot condition.
 *
 ****************************************************************************/

void board_go_to_standby(void)
{
  /* We cannot power-off display from interrrupt. */

  if (!up_interrupt_context())
    {
      sched_lock();

      /* Power-off display. */
#ifdef CONFIG_THINGSEE_DISPLAY_MODULE
      board_lcdoff();
#endif
  }

  DEBUGASSERT((getreg32(STM32_PWR_CR) & PWR_CR_DBP) == 0);
  stm32_pwr_enablebkp(true);

  /* Setup bootloader to jump directly to firmware. */

  putreg32(BOARD_FIRMWARE_BASE_ADDR, CONFIG_BOOTLOADER_ADDR_BKREG);

  /* Setup backup register for standby mode (checked after reset in boot-up
   * routine).
   */

  putreg32(CONFIG_STANDBYMODE_MAGIC, CONFIG_STANDBYMODE_MAGIC_BKREG);

  stm32_pwr_enablebkp(false);

  lldbg("Driving MCU to standby mode (with wake-up by power-button)...\n");
  up_mdelay(250);

  board_systemreset();
}

/****************************************************************************
 * Name: up_boot_standby_mode
 *
 * Description:
 *   Handling for low-power standby mode.
 *
 ****************************************************************************/

void up_boot_standby_mode(void)
{
  uint32_t gpio_off_mask;
  uint32_t regval;
  int i;

  DEBUGASSERT((getreg32(STM32_PWR_CR) & PWR_CR_DBP) != 0);

  /*
   * This function is called from up_boot.c, stm32_boardinitialize(). OS is not
   * running yet. Regulators and chip-selects have been initialized. IWDG not
   * enabled. If board has HWWDG, we must enable it from this function before
   * going to standby mode.
   */

  regval = getreg32(CONFIG_STANDBYMODE_MAGIC_BKREG);
  if (regval != CONFIG_STANDBYMODE_MAGIC)
    {
      g_board_wakeup_from_standby = (regval == CONFIG_STANDBYMODE_MAGIC + 1);

      /* Standby mode was not requested, continue with regular boot. */

      putreg32(0, CONFIG_STANDBYMODE_MAGIC_BKREG);

      for (i = 0; i < 4; i++)
        up_lowputc('r');

      return;
    }

#if defined(CONFIG_STM32_DFU) || defined (CONFIG_BOARD_HALTIAN_HWWDG)
  up_irqinitialize();
#endif

  /* Go into standby mode. */

  putreg32(CONFIG_STANDBYMODE_MAGIC + 1, CONFIG_STANDBYMODE_MAGIC_BKREG);

  for (i = 0; i < 4; i++)
    up_lowputc('s');

  /* Configure SDcard pins. */

  gpio_initialize_sdcard_pins();

  /* Configure display GPIOs. */

  gpio_off_mask = ~(GPIO_PUPD_MASK | GPIO_MODE_MASK | GPIO_OUTPUT_SET);
  stm32_configgpio(GPIO_CHIP_SELECT_DISPLAY);
  stm32_configgpio(GPIO_LCD_SSD1309_CMDDATA);
  stm32_configgpio(GPIO_LCD_SSD1309_RESET);
  stm32_configgpio((GPIO_SPI2_MOSI & gpio_off_mask) | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT);
  stm32_configgpio((GPIO_SPI2_SCK & gpio_off_mask) | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT);

  /* Reconfigure GPIO's for stop mode (most pins are setup as analog input). */

  up_reconfigure_gpios_for_pmstop();

  /* Setup bootloader to jump directly to firmware. */

  putreg32(BOARD_FIRMWARE_BASE_ADDR, CONFIG_BOOTLOADER_ADDR_BKREG);

  /* Now disable backup register access. If following interrupt handlers
   * access backup registers, they need to make sure to re-enable access. */

  stm32_pwr_enablebkp(false);

  /* We must kick HWWDG even from standby mode, else it resets device. */

  board_wdginitialize_autokick(hwwdg_irq_in_standby);

  /* Setup 'power'-button as EXTI for wake-up. */

  stm32_configgpio(GPIO_BTN_POWERKEY);
  stm32_gpiosetevent(GPIO_BTN_POWERKEY, true, false, true,
                     button_pressed_down_handler);

  /* Our standby-mode is really ARM core's stop-mode.
     Enter stop-mode with MCU internal regulator in low-power mode. */

  while (true)
    {
      (void)stm32_pmstop(true);

      standby_do_trace('p');
    }
}
