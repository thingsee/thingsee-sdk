/****************************************************************************
 * apps/examples/unity_pwrbtn/button_events.c
 * Simple tests for power button device driver node
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
#include <nuttx/testing/unity_fixture.h>
#include <stdio.h>
#include <unistd.h>

#include "defines.h"
#include "power_button_stimulus.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/
static FILE* device;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
TEST_GROUP(ButtonEvents);

/****************************************************************************
 * Name: ButtonEvents test group setup
 *
 * Description:
 *   Setup function executed before each testcase in this test group
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
TEST_SETUP(ButtonEvents)
{
  device = fopen(PWR_BTN_NODE, "r");
  TEST_ASSERT_NOT_NULL(device);
  power_button_stimulus_release();
  power_button_stimulus_engage();
  usleep(5000);
}

/****************************************************************************
 * Name: ButtonEvents test group teardown
 *
 * Description:
 *   Setup function executed before each testcase in this test group
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
TEST_TEAR_DOWN(ButtonEvents)
{
  if (device != NULL)
  {
    fclose(device);
    device = NULL;
  }
  power_button_stimulus_disengage();
}

/****************************************************************************
 * Name: Depressing
 *
 * Description:
 *   Simulate button depression and verify that device node reads it is active
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
TEST(ButtonEvents, Depressing)
{
  power_button_stimulus_depress();
  char state;
  READ_STATE(device, &state);
  TEST_ASSERT_BUTTON_ON(state);
}

/****************************************************************************
 * Name: Releasing
 *
 * Description:
 *   Simulate button release and verify that device node says it is not pressed
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
TEST(ButtonEvents, Releasing)
{
  power_button_stimulus_release();
  char state;
  READ_STATE(device, &state);
  TEST_ASSERT_BUTTON_OFF(state);
}

/****************************************************************************
 * Name: SwitchThousandTimes
 *
 * Description:
 *   Repeat press-release cycle 1000 times and check correct state every time
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
TEST(ButtonEvents, SwitchThousandTimes)
{
  int i;
  int count_depressed = 0;
  int count_released = 0;
  char state;
  for (i = 0; i < 1000; i++)
  {
    power_button_stimulus_depress();
    READ_STATE(device, &state);
    if (IS_BUTTON_ON(state))
      count_depressed++;
    power_button_stimulus_release();
    READ_STATE(device, &state);
    if (!IS_BUTTON_ON(state))
      count_released++;
  }
  TEST_ASSERT_EQUAL_INT(1000, count_depressed);
  TEST_ASSERT_EQUAL_INT(1000, count_released);
}

