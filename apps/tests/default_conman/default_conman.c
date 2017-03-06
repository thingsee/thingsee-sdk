/****************************************************************************
 * apps/tests/default_conman/default_conman.c
 * Reusable functions to call connection manager from test applications
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
#include "default_conman.h"

#include <nuttx/config.h>
#include <stdbool.h>
#include <apps/testing/unity_fixture.h>

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

/****************************************************************************
 * Public Data
 ****************************************************************************/
const char DEFAULT_WLAN_CONNECTION[] =
    "{"
      "\"connections\": ["
        "{"
          "\"wifiConnections\": ["
            "{"
              "\"ssid\": \"" CONFIG_TESTS_DEFAULT_CONMAN_DEFAULT_WLAN_SSID "\","
              "\"password\": \"" CONFIG_TESTS_DEFAULT_CONMAN_DEFAULT_WLAN_PASS "\","
              "\"connectionId\": 1"
#ifndef CONFIG_TESTS_DEFAULT_CONMAN_DEFAULT_WLAN_ENCRYPTION_NONE
              ",\"encryption\": \"" CONFIG_TESTS_DEFAULT_CONMAN_DEFAULT_WLAN_ENCRYPTION "\""
#endif
            "}"
          "]"
        "}"
      "]"
    "}";

const char DEFAULT_2G_CONNECTION[] =
    "{"
      "\"connections\": ["
        "{"
          "\"cellularConnections\": ["
            "{"
              "\"accessPointName\": \"" CONFIG_TESTS_DEFAULT_CONMAN_DEFAULT_2G_APN "\","
              "\"connectionId\": 1"
            "}"
          "]"
        "}"
      "]"
    "}";

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wait_conman_final_connection_state
 *
 * Description:
 *   Waits until connection status is terminal
 *
 * Input Parameters:
 *   client - initialized conman client structure
 *
 * Returned Value:
 *   Final connection state
 *
 * Assumptions/Limitations:
 *   Fails test immediately, when connection status fetching fails
 *
 ****************************************************************************/
enum conman_status_type_e wait_conman_final_connection_state(struct conman_client_s* client)
{
  int ret;
  struct conman_status_s status;
  bool goon = true;

  while (goon)
  {
    usleep(300000);
    ret = conman_client_get_connection_status(client, &status);
    TEST_ASSERT_EQUAL(OK, ret);
    goon =
        status.status == CONMAN_STATUS_ESTABLISHING ||
        status.status == CONMAN_STATUS_DESTROYING;
  }

  return status.status;
}
