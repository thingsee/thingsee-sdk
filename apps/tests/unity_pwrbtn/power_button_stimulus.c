/****************************************************************************
 * apps/examples/unity_pwrbtn/power_button_stimulus.c
 * Simulates power button states
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
 *   Author: Roman Saveljev <roman.saveljev@haltian.com>
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
#include "power_button_stimulus.h"

#include <nuttx/testing/unity_fixture.h>
#include <unistd.h>

#include "stm32.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define GPIO_BTN_POWERKEY       (GPIO_INPUT | GPIO_EXTI | GPIO_FLOAT | GPIO_PORTC | GPIO_PIN13)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: power_button_stimulus_engage
 *
 * Description:
 *   Configures power button GPIO for output, so events can be simulated
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   Driven value is read back and triggers interrupts
 *
 ****************************************************************************/
void power_button_stimulus_engage(void)
{
  stm32_configgpio((GPIO_BTN_POWERKEY & ~GPIO_INPUT) | GPIO_OUTPUT);
}

/****************************************************************************
 * Name: power_button_stimulus_disengage
 *
 * Description:
 *   Configures power button GPIO to default setting
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   None
 *
 ****************************************************************************/
void power_button_stimulus_disengage(void)
{
  stm32_configgpio(GPIO_BTN_POWERKEY);
}

/****************************************************************************
 * Name: power_button_stimulus_depress
 *
 * Description:
 *   Activates power button
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   There is a pull-down on the button's GPIO
 *
 ****************************************************************************/
void power_button_stimulus_depress(void)
{
  stm32_gpiowrite(GPIO_BTN_POWERKEY, true);
}

/****************************************************************************
 * Name: power_button_stimulus_release
 *
 * Description:
 *   Deactivates power button
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   There is a pull-down on the button's GPIO
 *
 ****************************************************************************/
void power_button_stimulus_release(void)
{
  stm32_gpiowrite(GPIO_BTN_POWERKEY, false);
}
