/************************************************************************************
 * configs/sam3u-ek/src/sam_touchscreen.c
 *
 *   Copyright (C) 2011-2013 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/input/touchscreen.h>
#include <nuttx/input/ads7843e.h>

#include "sam_gpio.h"
#include "sam3u-ek.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifdef CONFIG_INPUT_ADS7843E
#ifndef CONFIG_INPUT
#  error "Touchscreen support requires CONFIG_INPUT"
#endif

#ifndef CONFIG_SAM34_SPI0
#  error "Touchscreen support requires CONFIG_SAM34_SPI0"
#endif

#ifndef CONFIG_SAM34_GPIOA_IRQ
#  error "Touchscreen support requires CONFIG_SAM34_GPIOA_IRQ"
#endif

#ifndef CONFIG_ADS7843E_FREQUENCY
#  define CONFIG_ADS7843E_FREQUENCY 500000
#endif

#ifndef CONFIG_ADS7843E_SPIDEV
#  define CONFIG_ADS7843E_SPIDEV TSC_CSNUM
#endif

#if CONFIG_ADS7843E_SPIDEV != TSC_CSNUM
#  error "CONFIG_ADS7843E_SPIDEV must have the same value as TSC_CSNUM"
#endif

#ifndef CONFIG_ADS7843E_DEVMINOR
#  define CONFIG_ADS7843E_DEVMINOR 0
#endif

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the ADS7843E driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.  If possible,
 * interrupts should be configured on both rising and falling edges
 * so that contact and loss-of-contact events can be detected.
 *
 * attach  - Attach the ADS7843E interrupt handler to the GPIO interrupt
 * enable  - Enable or disable the GPIO interrupt
 * clear   - Acknowledge/clear any pending GPIO interrupt
 * pendown - Return the state of the pen down GPIO input
 */

static int  tsc_attach(FAR struct ads7843e_config_s *state, xcpt_t isr);
static void tsc_enable(FAR struct ads7843e_config_s *state, bool enable);
static void tsc_clear(FAR struct ads7843e_config_s *state);
static bool tsc_busy(FAR struct ads7843e_config_s *state);
static bool tsc_pendown(FAR struct ads7843e_config_s *state);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the ADS7843E
 * driver.  This structure provides information about the configuration
 * of the ADS7843E and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active. The
 * memory must be writable because, under certain circumstances, the driver
 * may modify frequency or X plate resistance values.
 */

static struct ads7843e_config_s g_tscinfo =
{
  .frequency = CONFIG_ADS7843E_FREQUENCY,

  .attach    = tsc_attach,
  .enable    = tsc_enable,
  .clear     = tsc_clear,
  .busy      = tsc_busy,
  .pendown   = tsc_pendown,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the ADS7843E driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.  If possible,
 * interrupts should be configured on both rising and falling edges
 * so that contact and loss-of-contact events can be detected.
 *
 * attach  - Attach the ADS7843E interrupt handler to the GPIO interrupt
 * enable  - Enable or disable the GPIO interrupt
 * clear   - Acknowledge/clear any pending GPIO interrupt
 * pendown - Return the state of the pen down GPIO input
 */

static int tsc_attach(FAR struct ads7843e_config_s *state, xcpt_t isr)
{
  /* Attach the ADS7843E interrupt */

  ivdbg("Attaching %p to IRQ %d\n", isr, SAM_TCS_IRQ);
  return irq_attach(SAM_TCS_IRQ, isr);
}

static void tsc_enable(FAR struct ads7843e_config_s *state, bool enable)
{
  /* Attach and enable, or detach and disable */

  ivdbg("IRQ:%d enable:%d\n", SAM_TCS_IRQ, enable);
  if (enable)
    {
      sam_gpioirqenable(SAM_TCS_IRQ);
    }
  else
    {
      sam_gpioirqdisable(SAM_TCS_IRQ);
    }
}

static void tsc_clear(FAR struct ads7843e_config_s *state)
{
  /* Does nothing */
}

static bool tsc_busy(FAR struct ads7843e_config_s *state)
{
#if defined(CONFIG_DEBUG_INPUT) && defined(CONFIG_DEBUG_VERBOSE)
  static bool last = (bool)-1;
#endif

  /* BUSY is high impedance when CS is high (not selected).  When CS is
   * is low, BUSY is active high.
   */

  bool busy = sam_gpioread(GPIO_TCS_BUSY);
#if defined(CONFIG_DEBUG_INPUT) && defined(CONFIG_DEBUG_VERBOSE)
  if (busy != last)
    {
      ivdbg("busy:%d\n", busy);
      last = busy;
    }
#endif

  return busy;
}

static bool tsc_pendown(FAR struct ads7843e_config_s *state)
{
  /* The /PENIRQ value is active low */

  bool pendown = !sam_gpioread(GPIO_TCS_IRQ);
  ivdbg("pendown:%d\n", pendown);
  return pendown;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_tsc_setup
 *
 * Description:
 *   Each board that supports a touchscreen device must provide this function.
 *   This function is called by application-specific, setup logic to
 *   configure the touchscreen device.  This function will register the driver
 *   as /dev/inputN where N is the minor device number.
 *
 * Input Parameters:
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_tsc_setup(int minor)
{
  FAR struct spi_dev_s *dev;
  int ret;

  idbg("minor %d\n", minor);
  DEBUGASSERT(minor == 0);

  /* Configure and enable the ADS7843E interrupt pin as an input */

  (void)sam_configgpio(GPIO_TCS_BUSY);
  (void)sam_configgpio(GPIO_TCS_IRQ);

  /* Configure the PIO interrupt */

  sam_gpioirq(GPIO_TCS_IRQ);

  /* Get an instance of the SPI interface for the touchscreen chip select */

  dev = up_spiinitialize(TSC_CSNUM);
  if (!dev)
    {
      idbg("Failed to initialize SPI chip select %d\n", TSC_CSNUM);
      return -ENODEV;
    }

  /* Initialize and register the SPI touschscreen device */

  ret = ads7843e_register(dev, &g_tscinfo, CONFIG_ADS7843E_DEVMINOR);
  if (ret < 0)
    {
      idbg("Failed to initialize SPI chip select %d\n", TSC_CSNUM);
      /* up_spiuninitialize(dev); */
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: board_tsc_teardown
 *
 * Description:
 *   Each board that supports a touchscreen device must provide this function.
 *   This function is called by application-specific, setup logic to
 *   uninitialize the touchscreen device.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void board_tsc_teardown(void)
{
  /* No support for un-initializing the touchscreen ADS7843E device yet */
}

#endif /* CONFIG_INPUT_ADS7843E */

