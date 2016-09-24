/****************************************************************************
 * apps/system/conman/conman_wifi_stubs.c
 *
 *   Copyright (C) 2015-2016 Haltian Ltd. All rights reserved.
 *   Authors: Pekka Niemimaa <pekka.niemimaa@haltian.com>
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
#include <nuttx/config.h>
#include "conman_dbg.h"
#include "conman_internal.h"


int __conman_cc3000_request_connection(struct conman_s *conman,
                                       enum conman_connection_type_e type)
{
  (void)conman;
  (void)type;
  conman_dbg("no WiFi configs available\n");
  return ERROR;
}

int __conman_cc3000_get_status_connection(struct conman_s *conman,
                                          struct conman_status_s *status)
{
  (void)conman;
  status->status = CONMAN_STATUS_OFF;
  return ERROR;
}

bool __conman_cc3000_is_destroying(struct conman_s *conman)
{
  (void)conman;
  return false;
}

int __conman_cc3000_wifi_scan(struct conman_s *conman)
{
  (void)conman;
  return ERROR;
}

unsigned int __conman_cc3000_get_max_pollfds(struct conman_s *conman)
{
  (void)conman;
  return 0;
}

void __conman_cc3000_setup_pollfds(struct conman_s *conman,
                                   struct pollfd *pfds, int maxfds,
                                   int *fds_pos, int *min_timeout)
{
  (void)conman;
  (void)pfds;
  (void)maxfds;
  (void)fds_pos;
  (void)min_timeout;
}

void __conman_cc3000_handle_pollfds(struct conman_s *conman,
                                    struct pollfd *pfds)
{
  (void)conman;
  (void)pfds;
}
