/****************************************************************************
 * include/example_app_connectivity.h
 *
 * Copyright (C) 2016 Haltian Ltd.
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
 * Authors:
 *   Jussi Kivilinna <jussi.kivilinna@haltian.com>
 *
 ****************************************************************************/

#ifndef __EXAMPLE_APP_INCLUDE_CONNECTIVITY_H__
#define __EXAMPLE_APP_INCLUDE_CONNECTIVITY_H__

#include <stdint.h>
#include <stdbool.h>

struct exapp_sms_s
{
  const char *number;
  const char *content;
};

int __exapp_connectivity_init(void);
int __exapp_connectivity_uninit(void);
int __exapp_connectivity_destroy(uint32_t delay_s);

int __exapp_connectivity_initial_request(void);
int __exapp_connectivity_request(void);

int __exapp_connectivity_poke_connection(void);

int __exapp_connectivity_send_sms(const struct exapp_sms_s * const sms);
int __exapp_connectivity_get_imei(char **imei);
int __exapp_connectivity_request_cell_environment(void);

int __exapp_connectivity_wifi_scan(void);

#endif /* __EXAMPLE_APP_INCLUDE_CONNECTIVITY_H__ */
