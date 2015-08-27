/****************************************************************************
 * apps/examples/unity_pwrbtn/device_node_perms.c
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
#include <sys/stat.h>
#include <errno.h>

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
static struct stat st;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

TEST_GROUP(DeviceNodePermissions);

/****************************************************************************
 * Name: DeviceNodePermissions test group setup
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
TEST_SETUP(DeviceNodePermissions)
{
  int ret;
  ret = stat(PWR_BTN_NODE, &st);
  TEST_ASSERT_EQUAL_INT(0, ret);
}

TEST_TEAR_DOWN(DeviceNodePermissions)
{
}

/****************************************************************************
 * Name: Character
 *
 * Description:
 *   Test PWR_BTN_NODE is a character device
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
TEST(DeviceNodePermissions, Character)
{
  TEST_ASSERT_TRUE(S_ISCHR(st.st_mode));
}

/****************************************************************************
 * Name: Readable
 *
 * Description:
 *   Test PWR_BTN_NODE is readable by everyone
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
TEST(DeviceNodePermissions, Readable)
{
  TEST_ASSERT_EQUAL_INT(S_IROTH, st.st_mode & S_IROTH);
  TEST_ASSERT_EQUAL_INT(S_IRGRP, st.st_mode & S_IRGRP);
  TEST_ASSERT_EQUAL_INT(S_IRUSR, st.st_mode & S_IRUSR);
}

/****************************************************************************
 * Name: Readable
 *
 * Description:
 *   Test PWR_BTN_NODE is not writable by anyone
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
TEST(DeviceNodePermissions, NotWritable)
{
  TEST_ASSERT_EQUAL_INT(0, st.st_mode & S_IWOTH);
  TEST_ASSERT_EQUAL_INT(0, st.st_mode & S_IWGRP);
  TEST_ASSERT_EQUAL_INT(0, st.st_mode & S_IWUSR);
}

/****************************************************************************
 * Name: Readable
 *
 * Description:
 *   Test PWR_BTN_NODE is not executable by anyone
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
TEST(DeviceNodePermissions, NotExecutable)
{
  TEST_ASSERT_EQUAL_INT(0, st.st_mode & S_IXOTH);
  TEST_ASSERT_EQUAL_INT(0, st.st_mode & S_IXGRP);
  TEST_ASSERT_EQUAL_INT(0, st.st_mode & S_IXUSR);
}
