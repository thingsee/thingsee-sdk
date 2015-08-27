/****************************************************************************
 * apps/tests/unity_conman/client.c
 * Try out conman client library
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
#include <nuttx/testing/unity_fixture.h>
#include <stdbool.h>

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

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
TEST_GROUP(Client);

/****************************************************************************
 * Name: Client test group setup
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
TEST_SETUP(Client)
{
  int ret;

  g_initialized = false;
  ret = conman_client_init(&g_client);
  TEST_ASSERT_EQUAL(OK, ret);

  g_initialized = true;
}

/****************************************************************************
 * Name: Client test group tear down
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
TEST_TEAR_DOWN(Client)
{
  if (g_initialized)
    {
      conman_client_uninit(&g_client);
    }
}

/****************************************************************************
 * Name: ConfigDefaultWiFi
 *
 * Description:
 *   Configuring conman for default WiFi must succeed
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
TEST(Client, ConfigDefaultWiFi)
{
  int ret;

  ret = conman_client_set_connections_config(&g_client, DEFAULT_WLAN_CONNECTION);
  TEST_ASSERT_EQUAL(OK, ret);
}

/****************************************************************************
 * Name: ConfigDefaultData
 *
 * Description:
 *   Configuring conman for default data must succeed
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
TEST(Client, ConfigDefaultData)
{
  int ret;

  ret = conman_client_set_connections_config(&g_client, DEFAULT_2G_CONNECTION);
  TEST_ASSERT_EQUAL(OK, ret);
}

/****************************************************************************
 * Name: NoConnectionStatus
 *
 * Description:
 *   Verify connection status before any connection/configuration
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
TEST_WITH_TIMEOUT(Client, NoConnectionStatus, 5000)
{
  int ret;
  struct conman_status_s status;

  wait_conman_final_connection_state(&g_client);

  ret = conman_client_get_connection_status(&g_client, &status);
  TEST_ASSERT_EQUAL(OK, ret);

  TEST_ASSERT_EQUAL(CONMAN_NONE, status.conn_type);
  TEST_ASSERT_EQUAL(CONMAN_STATUS_OFF, status.status);
  TEST_ASSERT_EQUAL(CONMAN_INFO_NONE, status.info_type);
}
