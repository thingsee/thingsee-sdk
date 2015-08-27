/************************************************************************************
 * configs/haltian-tsone/src/up_chipselect.c
 * arch/arm/src/board/up_chipselect.c
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

#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <debug.h>
#include <assert.h>
#include <errno.h>

#include <arch/board/board.h>
#include "stm32.h"
#include "stm32_syscfg.h"

#include "haltian-tsone.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

/************************************************************************************
 * Configuration
 ************************************************************************************/

static const uint32_t cs_gpios[CONFIG_NUM_CHIP_SELECT_PINS] = {
  [CHIP_SELECT_SPI1_WLAN]          = GPIO_CHIP_SELECT_WLAN,
  [CHIP_SELECT_SPI2_DISPLAY]       = GPIO_CHIP_SELECT_DISPLAY,
  [CHIP_SELECT_SPI3_SDCARD]        = GPIO_CHIP_SELECT_SDCARD,
};

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/* Initialize chip-select GPIO pins and set output to deassert state. */
void board_initialize_chipselects(void)
{
  irqstate_t saved_state;
  unsigned int i;

  assert(ARRAY_SIZE(cs_gpios) == CONFIG_NUM_CHIP_SELECT_PINS);

  saved_state = irqsave();

  for (i = 0; i < CONFIG_NUM_CHIP_SELECT_PINS; i++)
    {
      /* Initialize GPIO. */
      stm32_configgpio(cs_gpios[i]);
    }

  irqrestore(saved_state);
}

/* Assert/Deassert chip select. */
void board_set_chip_select(enum e_board_chip_select_pins id, bool selected)
{
  irqstate_t saved_state;
  bool assert_high;
  bool pin_set;

  assert(id < CONFIG_NUM_CHIP_SELECT_PINS);

  /* If default state for pin is low, asserted state is high. */
  assert_high = !(cs_gpios[id] & GPIO_OUTPUT_SET);

  pin_set = selected ? assert_high : !assert_high;

  saved_state = irqsave();
  stm32_gpiowrite(cs_gpios[id], pin_set);
  irqrestore(saved_state);
}

/* Set output for all chip-select pins to deassert state. */
void board_chip_select_deassert_all(void)
{
  board_initialize_chipselects();
}
