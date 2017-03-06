/****************************************************************************
 * apps/examples/unity_pwrbtn/device_node_access.c
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
#include <apps/testing/unity_fixture.h>
#include <stdio.h>

#include "defines.h"

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

TEST_GROUP(DeviceNodeAccess);

/****************************************************************************
 * Name: DeviceNodeAccess test group setup
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
TEST_SETUP(DeviceNodeAccess)
{
}

TEST_TEAR_DOWN(DeviceNodeAccess)
{
  if (device != NULL)
  {
    fclose(device);
    device = NULL;
  }
}

/****************************************************************************
 * Name: CanRead
 *
 * Description:
 *   Test PWR_BTN_NODE can be opened for reading
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
TEST(DeviceNodeAccess, CanRead)
{
  device = fopen(PWR_BTN_NODE, "r");
  TEST_ASSERT_NOT_NULL(device);
}

/****************************************************************************
 * Name: CanNotWrite
 *
 * Description:
 *   Test PWR_BTN_NODE can not be opened for writing
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
TEST(DeviceNodeAccess, CanNotWrite)
{
  device = fopen(PWR_BTN_NODE, "w");
  TEST_ASSERT_NULL(device);
}

/****************************************************************************
 * Name: CanNotAppend
 *
 * Description:
 *   Test PWR_BTN_NODE can not be opened for appending
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
TEST(DeviceNodeAccess, CanNotAppend)
{
  device = fopen(PWR_BTN_NODE, "a");
  TEST_ASSERT_NULL(device);
}

/****************************************************************************
 * Name: CanNotReadWrite
 *
 * Description:
 *   Test PWR_BTN_NODE can not be opened for read-write
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
TEST(DeviceNodeAccess, CanNotReadWrite)
{
  device = fopen(PWR_BTN_NODE, "w+");
  TEST_ASSERT_NULL(device);
}
