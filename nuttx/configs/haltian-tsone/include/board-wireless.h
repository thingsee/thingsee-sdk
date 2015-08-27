/************************************************************************************
 * configs/haltian-tsone/include/board-wireless.h
 * include/arch/board/board-wireless.h
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

#ifndef __CONFIG_HALTIAN_TSONE_B1_INCLUDE_BOARD__WIRELESS_H_
#define __CONFIG_HALTIAN_TSONE_B1_INCLUDE_BOARD__WIRELESS_H_

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

#include <nuttx/wireless/cc3000.h>
#include <nuttx/wireless/cc3000/wlan.h>

/* Flags for cc3000_connect_to_accesspoint */

#define CC3000_CONN_FLG_NO_IP_ADDR_WAIT (1 << 0)

/*****************************************************************************
 * Name: cc3000_set_power
 *
 * Description:
 *   Set power ON/OFF for CC3000 module
 *
 ****************************************************************************/

void cc3000_set_power(bool on);

/*****************************************************************************
 * Name: cc3000_connect_to_accesspoint_with_stopfn
 *
 * Description:
 *   Attempt connection to access-point.
 *
 * Input Parameters:
 *   security Security type for accesspoint
 *   ssid     SSID for AP
 *   wpa_key  Plaintext WPA2-PSK key for AP
 *
 * Returned Value:
 *   True when successfully executed
 *
 ****************************************************************************/

bool cc3000_connect_to_accesspoint_with_stopfn(
    int security, const char *ssid, const char *key, unsigned int timeout_msec,
    int flags, bool (*stopfn)(void *priv), void *priv);

/*****************************************************************************
 * Name: cc3000_connect_to_accesspoint
 *
 * Description:
 *   Attempt connection to access-point.
 *
 * Input Parameters:
 *   security Security type for accesspoint
 *   ssid     SSID for AP
 *   wpa_key  Plaintext WPA2-PSK key for AP
 *
 * Returned Value:
 *   True when successfully executed
 *
 ****************************************************************************/

static inline bool cc3000_connect_to_accesspoint(int security, const char *ssid,
                                                 const char *key,
                                                 unsigned int timeout_msec,
                                                 int flags)
{
  return cc3000_connect_to_accesspoint_with_stopfn(security, ssid, key,
                                                   timeout_msec, flags,
                                                   NULL, NULL);
}

/*****************************************************************************
 * Name: cc3000_disconnect_from_accesspoint
 *
 * Description:
 *   Disconnect form access-point.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   True when successfully executed
 *
 ****************************************************************************/

bool cc3000_disconnect_from_accesspoint(void);

/*****************************************************************************
 * Name: cc3000_initialize
 *
 * Description:
 *   Initialize CC3000 wifi module (hw, then sw)
 *
 * Returned Value:
 *   True when successfully executed
 *
 ****************************************************************************/

bool cc3000_initialize(int (*debugout_fn)(const char *fmt, ...),
                       void (*async_callback)(long event_type, char *data, uint8_t length));

/*****************************************************************************
 * Name: cc3000_uninitialize
 *
 * Description:
 *   Uniitialize CC3000 wifi module
 *
 * Returned Value:
 *   True when successfully executed
 *
 ****************************************************************************/

bool cc3000_uninitialize(void);

#endif /* __CONFIG_HALTIAN_TSONE_B1_INCLUDE_BOARD__WIRELESS_H_ */
