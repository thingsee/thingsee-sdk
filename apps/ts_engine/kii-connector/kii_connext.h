/****************************************************************************
 * apps/ts_engine/kii-connector/kii_connext.h
 *
 * Copyright (C) 2015 Haltian Ltd. All rights reserved.
 * Author: Timo Voutilainen <timo.voutilainen@haltian.com>
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

#ifndef __APPS_TS_ENGINE_CONNECTORS_KII_CONNEXT_H
#define __APPS_TS_ENGINE_CONNECTORS_KII_CONNEXT_H

#include "connector.h"
#include "conn_comm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/
typedef struct
{
  uint32_t connector_id;
  char *connector_name;
  char *protocol;
  char *app_id;
  char *app_key;
  char *access_token;
  char *thing_type;
  char *thing_id;
  char *vendor_thing_id;
  char *password;
  char *kii_vendor;
  char *bucket;
}kii_cloud_params_s;

typedef enum kii_get_ext_status
{
  KII_EXT_OK = 0,
  KII_EXT_WAITING,
  KII_EXT_FAILED
}kii_get_ext_status_t;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

kii_get_ext_status_t kii_get_vendor_thing_id(char **vendor_thing_id);

kii_get_ext_status_t kii_get_configuration(kii_cloud_params_s *cloud_params,   con_str_t *con);

 #endif /* __APPS_TS_ENGINE_CONNECTORS_KII_CONNEXT_H */
