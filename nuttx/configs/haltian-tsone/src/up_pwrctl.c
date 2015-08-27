/************************************************************************************
 * configs/haltian-tsone/src/up_pwrctl.c
 * arch/arm/src/board/up_pwrctl.c
 *
 *   Copyright (C) 2014-2015 Haltian Ltd. All rights reserved.
 *
 * Authors:
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

#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <arch/board/board-pwrctl.h>
#include "stm32.h"
#include "stm32_syscfg.h"

#include "haltian-tsone.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

struct pwrctl_s {
    /* Reference count for pwrctl */
    uint8_t ref_count;

    /* Has pwrctl woken-up/stabilized? */
    bool has_stabilized;
};

struct pwrctl_gpio_s {
    /* GPIO for power control */
    uint32_t pincfg;

    /* Time to stabilize power-on in µseconds. */
    uint16_t poweron_time_usec;
} packed_struct;

/************************************************************************************
 * Configuration
 ************************************************************************************/

/* Wait 5 msec after turning pwrctl on. */
#define REGULATOR_STARTUP_DELAY_USEC 5000

/* T_I2CBOOT (max) time for CapSense controller. LSM9DS1 seems to need less. */
#define REGULATOR_STARTUP_DELAY_I2C_USEC 15000

enum e_pwrctl_pins {
  PWRCTL_PIN_REGULATOR_GPS = 0,
  PWRCTL_PIN_REGULATOR_WLAN,
  PWRCTL_PIN_REGULATOR_SDCARD,
  PWRCTL_PIN_REGULATOR_BLUETOOTH,
  PWRCTL_PIN_REGULATOR_DISPLAY,
  PWRCTL_PIN_SWITCH_MODEM,
  PWRCTL_PIN_SWITCH_CAPSENSE_9AXIS,
  PWRCTL_PIN_SWITCH_ADC_VBAT,

  __PWRCTL_PIN_MAX,
  CONFIG_NUM_PWRCTL_PIN = __PWRCTL_PIN_MAX,
};

static const enum e_pwrctl_pins pwrctl_to_pin[] = {
  [PWRCTL_REGULATOR_GPS]                = PWRCTL_PIN_REGULATOR_GPS,
  [PWRCTL_REGULATOR_WLAN]               = PWRCTL_PIN_REGULATOR_WLAN,
  [PWRCTL_REGULATOR_SDCARD]             = PWRCTL_PIN_REGULATOR_SDCARD,
  [PWRCTL_REGULATOR_BLUETOOTH]          = PWRCTL_PIN_REGULATOR_BLUETOOTH,
  [PWRCTL_REGULATOR_DISPLAY]            = PWRCTL_PIN_REGULATOR_DISPLAY,
  [PWRCTL_REGULATOR_FLASH]              = __PWRCTL_PIN_MAX,
  [PWRCTL_REGULATOR_EXTRA]              = __PWRCTL_PIN_MAX,
  [PWRCTL_SWITCH_MODEM]                 = PWRCTL_PIN_SWITCH_MODEM,
  [PWRCTL_SWITCH_CAPSENSE_SENSOR]       = PWRCTL_PIN_SWITCH_CAPSENSE_9AXIS,
  [PWRCTL_SWITCH_9AXIS_INERTIAL_SENSOR] = PWRCTL_PIN_SWITCH_CAPSENSE_9AXIS,
  [PWRCTL_SWITCH_ADC_VBAT_MEASUREMENT]  = PWRCTL_PIN_SWITCH_ADC_VBAT,
};

/************************************************************************************
 * Private Data
 ************************************************************************************/

static struct pwrctl_s pwrctls[CONFIG_NUM_PWRCTL_PIN];
static struct
{
  bool initialized:1;
  bool forced_all_off:1;
} g_pwrctl =
  {
    false, false
  };

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static void get_pwrctl_gpio(enum e_pwrctl_pins pin, struct pwrctl_gpio_s *out)
{
  switch (pin)
    {
      case PWRCTL_PIN_REGULATOR_GPS:
        out->pincfg            = GPIO_REGULATOR_GPS;
        out->poweron_time_usec = REGULATOR_STARTUP_DELAY_USEC;
        break;
      case PWRCTL_PIN_REGULATOR_WLAN:
        out->pincfg            = GPIO_REGULATOR_WLAN;
        out->poweron_time_usec = REGULATOR_STARTUP_DELAY_USEC;
        break;
      case PWRCTL_PIN_REGULATOR_SDCARD:
        out->pincfg            = GPIO_REGULATOR_SDCARD;
        out->poweron_time_usec = REGULATOR_STARTUP_DELAY_USEC;
        break;
      case PWRCTL_PIN_REGULATOR_BLUETOOTH:
        out->pincfg            = GPIO_REGULATOR_BLUETOOTH;
        out->poweron_time_usec = REGULATOR_STARTUP_DELAY_USEC;
        break;
      case PWRCTL_PIN_REGULATOR_DISPLAY:
        out->pincfg            = GPIO_REGULATOR_DISPLAY;
        out->poweron_time_usec = REGULATOR_STARTUP_DELAY_USEC;
        break;
      case PWRCTL_PIN_SWITCH_MODEM:
        out->pincfg            = GPIO_PWR_SWITCH_MODEM;
        out->poweron_time_usec = REGULATOR_STARTUP_DELAY_USEC;
        break;
      case PWRCTL_PIN_SWITCH_CAPSENSE_9AXIS:
        out->pincfg            = GPIO_PWR_SWITCH_CAPSENSE_9AXIS;
        out->poweron_time_usec = REGULATOR_STARTUP_DELAY_I2C_USEC;
        break;
      case PWRCTL_PIN_SWITCH_ADC_VBAT:
        out->pincfg            = GPIO_PWR_SWITCH_VBAT_ADC;
        out->poweron_time_usec = 0;
        break;
      default:
        DEBUGASSERT(false);
        break;
    }
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/* Initialize pwrctl GPIO pins and set initial output low. */
void board_initialize_pwrctl_pins(void)
{
  const unsigned int num_pwrctl = CONFIG_NUM_PWRCTL_PIN;
  unsigned int i;
  struct pwrctl_gpio_s pwrctl_gpio;
  irqstate_t saved_state;

  if (g_pwrctl.initialized)
    return;

  assert(ARRAY_SIZE(pwrctls) == CONFIG_NUM_PWRCTL_PIN);

  saved_state = irqsave();

  for (i = 0; i < num_pwrctl; i++)
    {
      get_pwrctl_gpio(i, &pwrctl_gpio);

      /* Initialize GPIO. */
      stm32_configgpio(pwrctl_gpio.pincfg);
    }

  g_pwrctl.initialized = true;

  irqrestore(saved_state);
}

/* Force all power-control gpios to power-off state. */
void board_pwrctl_all_off(void)
{
  const unsigned int num_pwrctl = CONFIG_NUM_PWRCTL_PIN;
  unsigned int i;
  struct pwrctl_gpio_s pwrctl_gpio;
  irqstate_t saved_state;

  saved_state = irqsave();

  for (i = 0; i < num_pwrctl; i++)
    {
      get_pwrctl_gpio(i, &pwrctl_gpio);

      /* Initialize GPIO. */
      stm32_configgpio(pwrctl_gpio.pincfg);
    }

  g_pwrctl.initialized = true;
  g_pwrctl.forced_all_off = true;

  irqrestore(saved_state);
}

/* Get pwrctl for use. Powers on the pwrctl when necessary. */
void board_pwrctl_get(enum e_board_pwrctl pwrctl_id)
{
  enum e_pwrctl_pins pwrctl_pin;
  struct pwrctl_s *pwrctl;
  struct pwrctl_gpio_s pwrctl_gpio;
  bool is_first_user = false;
  bool do_delay = false;
  irqstate_t saved_state;

  assert(g_pwrctl.initialized);
  assert(!g_pwrctl.forced_all_off);
  assert(pwrctl_id < ARRAY_SIZE(pwrctl_to_pin) && pwrctl_id >= 0);

  /* Map pwrctl to pwrctl_pin. */
  pwrctl_pin = pwrctl_to_pin[pwrctl_id];
  assert(pwrctl_pin >= 0);

  if (pwrctl_pin >= CONFIG_NUM_PWRCTL_PIN)
    {
      /* No pin available for this pwrctl on this board. */
      return;
    }

  pwrctl = &pwrctls[pwrctl_pin];
  assert(pwrctl->ref_count < UINT8_MAX);

  get_pwrctl_gpio(pwrctl_pin, &pwrctl_gpio);

  saved_state = irqsave();

  /* Turn pwrctl on, if it is not already. */
  if (pwrctl->ref_count++ == 0)
    {
      stm32_gpiowrite(pwrctl_gpio.pincfg, true);

      /* pwrctl needs small delay to stabilize. */
      do_delay = true;
      is_first_user = true;
      pwrctl->has_stabilized = false;
    }
  else
    {
      /* Check if another task just started the pwrctl. */
      if (!pwrctl->has_stabilized)
        {
          /* Another task started pwrctl and is still sleeping until pwrctl
           * has stabilized. We need to sleep until the other task has set
           * 'in_startup_delay' false. Easy way to achieve this is to just sleep
           * same amount of time.
           */
          do_delay = true;
        }
    }

  irqrestore(saved_state);

  /* Need to wait pwrctl to stabilize? */
  if (do_delay)
    {
      if (pwrctl_gpio.poweron_time_usec > 0)
        up_udelay(pwrctl_gpio.poweron_time_usec);

      if (is_first_user)
        {
          /* Mark pwrctl as stable. */
          saved_state = irqsave();
          pwrctl->has_stabilized = true;
          irqrestore(saved_state);
        }
    }
}

/* Put pwrctl from use. Powers down the pwrctl when no users left. */
void board_pwrctl_put(enum e_board_pwrctl pwrctl_id)
{
  enum e_pwrctl_pins pwrctl_pin;
  struct pwrctl_s *pwrctl;
  struct pwrctl_gpio_s pwrctl_gpio;
  irqstate_t saved_state;

  assert(g_pwrctl.initialized);
  assert(!g_pwrctl.forced_all_off);
  assert(pwrctl_id < ARRAY_SIZE(pwrctl_to_pin) && pwrctl_id >= 0);

  /* Map pwrctl to pwrctl_pin. */
  pwrctl_pin = pwrctl_to_pin[pwrctl_id];
  assert(pwrctl_pin >= 0);

  if (pwrctl_pin >= CONFIG_NUM_PWRCTL_PIN)
    {
      /* No pin available for this pwrctl on this board. */
      return;
    }

  pwrctl = &pwrctls[pwrctl_pin];
  assert(pwrctl->ref_count > 0);
  assert(pwrctl->has_stabilized);

  get_pwrctl_gpio(pwrctl_pin, &pwrctl_gpio);

  saved_state = irqsave();

  /* Turn pwrctl off, if it is not already. */
  if (--pwrctl->ref_count == 0)
    {
      stm32_gpiowrite(pwrctl_gpio.pincfg, false);
      pwrctl->has_stabilized = false;
    }

  irqrestore(saved_state);
}

