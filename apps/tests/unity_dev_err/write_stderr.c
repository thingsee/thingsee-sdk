/****************************************************************************
 * apps/examples/unity_dev_err/write_stderr.c
 * Write /dev/err and verify it goes into host process STDERR
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
#include <fcntl.h>
#include <string.h>

#include "host_fs_helper.h"
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
static int device;
static int host_fd;
static int old_stderr;
static const char tmp_file_template[] = "/tmp/XXXXXX";
static const char test_string[] = "i am a test string";

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
TEST_GROUP(WriteStderr);

/****************************************************************************
 * Name: WriteStderr test group setup
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
TEST_SETUP(WriteStderr)
{
  device = -1;
  old_stderr = -1;
  host_fd = -1;

  /* Create temporary file */
  char tmp_file_name[sizeof(tmp_file_template)];
  strcpy(tmp_file_name, tmp_file_template);
  host_fd = host_fs_mkstemp(tmp_file_name);
  TEST_ASSERT_TRUE(host_fd >= 0);

  /* Make it STDERR */
  old_stderr = host_fs_reopen_stderr(host_fd);
  TEST_ASSERT_TRUE(old_stderr >= 0);

  /* Open /dev/err */
  device = open(DEV_ERR_NODE, O_WRONLY);
  TEST_ASSERT_TRUE(device >= 0);
}

/****************************************************************************
 * Name: WriteStderr test group tear down
 *
 * Description:
 *   Tear down function executed after each testcase in this test group
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
TEST_TEAR_DOWN(WriteStderr)
{
  if (device >= 0)
    {
      close(device);
      device = -1;
    }
  if (old_stderr >= 0)
    {
      host_fs_reopen_stderr(old_stderr);
      old_stderr = -1;
    }
  if (host_fd >= 0)
    {
      host_fs_close(host_fd);
      host_fd = -1;
    }
}

/****************************************************************************
 * Name: CheckContents
 *
 * Description:
 *   Check data written to /dev/err goes directly into STDERR
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
TEST(WriteStderr, CheckContents)
{
  char receiver[sizeof(test_string)];
  TEST_ASSERT_EQUAL(sizeof(test_string), write(device, test_string, sizeof(test_string)));
  TEST_ASSERT_EQUAL(0, host_fs_lseek(host_fd, 0, SEEK_SET));
  TEST_ASSERT_EQUAL(sizeof(test_string), host_fs_read(host_fd, receiver, sizeof(test_string)));
  TEST_ASSERT_EQUAL_UINT8_ARRAY(test_string, receiver, sizeof(test_string));
}
