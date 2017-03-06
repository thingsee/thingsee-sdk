/****************************************************************************
 * apps/tests/unity_conman/default_2g.c
 * Request connection with default 2G parameters
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
#include <apps/system/conman.h>
#include <apps/testing/unity_fixture.h>
#include <stdbool.h>
#include <string.h>

#include "default_conman.h"

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
static struct conman_client_s g_client;
static bool g_initialized;
static uint32_t g_connection_id;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
TEST_GROUP(Default2G);

/****************************************************************************
 * Name: Default2G test group setup
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
TEST_SETUP(Default2G)
{
  int ret;

  g_connection_id = CONMAN_CONNID_CLEAR;

  g_initialized = false;
  ret = conman_client_init(&g_client);
  TEST_ASSERT_EQUAL(OK, ret);
  g_initialized = true;

  ret = conman_client_set_connections_config(&g_client, DEFAULT_2G_CONNECTION);
  TEST_ASSERT_EQUAL(OK, ret);
}

/****************************************************************************
 * Name: Default2G test group tear down
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
TEST_TEAR_DOWN(Default2G)
{
  int ret;

  if (g_connection_id != CONMAN_CONNID_CLEAR)
    {
      ret = conman_client_destroy_connection(&g_client, g_connection_id);
      assert(ret == OK);
    }

  if (g_initialized)
    {
      conman_client_uninit(&g_client);
    }
}

/****************************************************************************
 * Name: SuccessStatus
 *
 * Description:
 *   Verify reported connection state on success
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
TEST_WITH_TIMEOUT(Default2G, SuccessStatus, 17000)
{
  int ret;
  struct conman_status_s status;
  const int MAX_OPER_NAME = sizeof(status.info.cellu.oper_name);
  const int MAX_IMEI = sizeof(status.info.cellu.imei);
  const int MAX_IMSI = sizeof(status.info.cellu.imsi);

  ret = conman_client_request_connection(&g_client, CONMAN_2G, &g_connection_id);
  TEST_ASSERT_EQUAL(OK, ret);

  wait_conman_final_connection_state(&g_client);

  ret = conman_client_get_connection_status(&g_client, &status);
  TEST_ASSERT_EQUAL(OK, ret);

  TEST_ASSERT_EQUAL(CONMAN_2G, status.conn_type);
  TEST_ASSERT_EQUAL(CONMAN_STATUS_ESTABLISHED, status.status);
  TEST_ASSERT_EQUAL(CONMAN_INFO_CELLULAR, status.info_type);

  TEST_ASSERT_NOT_EQUAL(0, strnlen(status.info.cellu.oper_name, MAX_OPER_NAME));
  TEST_ASSERT_NOT_EQUAL(MAX_OPER_NAME, strnlen(status.info.cellu.oper_name, MAX_OPER_NAME));

  TEST_ASSERT_NOT_EQUAL(0, strnlen(status.info.cellu.imei, MAX_IMEI));
  TEST_ASSERT_NOT_EQUAL(MAX_IMEI, strnlen(status.info.cellu.imei, MAX_IMEI));

  TEST_ASSERT_NOT_EQUAL(0, strnlen(status.info.cellu.imsi, MAX_IMSI));
  TEST_ASSERT_NOT_EQUAL(MAX_IMEI, strnlen(status.info.cellu.imsi, MAX_IMSI));

  TEST_ASSERT_NOT_EQUAL(0, status.info.cellu.ipaddr.s_addr);
}

/****************************************************************************
 * Name: Fiddle
 *
 * Description:
 *   Fiddles with the connection
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
TEST_WITH_TIMEOUT(Default2G, Fiddle, 240000)
{
  int i;
  int ret;
  enum conman_status_type_e status;

  for (i = 0; i < 3; i++)
  {
    ret = conman_client_request_connection(&g_client, CONMAN_2G, &g_connection_id);
    TEST_ASSERT_EQUAL(OK, ret);
    status = wait_conman_final_connection_state(&g_client);
    TEST_ASSERT_EQUAL(CONMAN_STATUS_ESTABLISHED, status);

    ret = conman_client_destroy_connection(&g_client, g_connection_id);
    TEST_ASSERT_EQUAL(OK, ret);
    status = wait_conman_final_connection_state(&g_client);
    TEST_ASSERT_EQUAL(CONMAN_STATUS_OFF, status);
    g_connection_id = CONMAN_CONNID_CLEAR;
  }
}
