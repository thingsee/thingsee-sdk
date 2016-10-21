/****************************************************************************
 * include/example_app_gps.h
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

#ifndef __EXAMPLE_APP_INCLUDE_GPS_H__
#define __EXAMPLE_APP_INCLUDE_GPS_H__

#include <stdint.h>
#include <stdbool.h>

struct gps_config_s
{
  uint32_t rate;
  uint32_t watchdog_secs;
};

struct location_s
{
  double latitude;
  double longitude;
  time_t time;       /* UTC */
  int32_t altitude;  /* meters */
  uint32_t speed;    /* km/h */
  uint32_t accuracy; /* meters */
  int16_t heading;   /* degrees */
  bool valid;
};

typedef int (*exapp_gps_callback_t)(struct location_s *location, void *priv);

int exapp_gps_init(const struct gps_config_s *config,
                   exapp_gps_callback_t callback, void * priv,
                   bool initial_fix);
int exapp_gps_uninit(void);

#endif /* __EXAMPLE_APP_INCLUDE_GPS_H__ */
