/****************************************************************************
 * apps/ts_engine/engine/client.h
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
 *   Author: Pekka Ervasti <pekka.ervasti@haltian.com>
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

#ifndef __APPS_TS_ENGINE_ENGINE_CLIENT_H__
#define __APPS_TS_ENGINE_ENGINE_CLIENT_H__

#define TS_ENGINE_CTRL_FIFO_IN		"/dev/ts_engine_ctrl_fifo_in"
#define TS_ENGINE_CTRL_FIFO_OUT		"/dev/ts_engine_ctrl_fifo_out"

enum ts_engine_client_cmd
{
    TS_ENGINE_READ_DEVICE_PROPERTY = 1,
    TS_ENGINE_READ_CLOUD_PROPERTY,
    TS_ENGINE_WRITE_CLOUD_PROPERTY,
    TS_ENGINE_READ_PROFILE,
    TS_ENGINE_WRITE_PROFILE,
    TS_ENGINE_WRITE_PROFILE_SHM,
    TS_ENGINE_RELOAD_PROFILE,
    TS_ENGINE_STOP,
    TS_ENGINE_PAUSE,
    TS_ENGINE_CONTINUE,
    TS_ENGINE_CONNECTOR_RESULT_SHM,
    TS_ENGINE_PING,
};

struct __attribute__ ((__packed__)) ts_engine_client_hdr
{
  uint8_t id;
  int status;
  size_t length;
};

struct __attribute__ ((__packed__)) ts_engine_client_req
{
  struct ts_engine_client_hdr hdr;

  char data[];
};

struct __attribute__ ((__packed__)) ts_engine_client_resp
{
  struct ts_engine_client_hdr hdr;

  char data[];
};

struct ts_engine_client
{
  int fdin;
  int fdout;

  struct ts_engine_client_resp resp;

  struct
  {
    char *data;
  } payload;
};

struct __attribute__ ((__packed__)) ts_engine_shm_obj
{
  void *addr;
  size_t len;
};

int ts_engine_client_init(struct ts_engine_client *client);

void ts_engine_client_uninit(struct ts_engine_client *client);

int ts_engine_client_read_profile(struct ts_engine_client *client);

int ts_engine_client_read_cloud_property(struct ts_engine_client *client);

int ts_engine_client_read_device_property(struct ts_engine_client *client);

int ts_engine_client_write_profile(struct ts_engine_client *client,
    const char * const data, size_t len);

int ts_engine_client_write_profile_shm(struct ts_engine_client *client,
    char * const data, size_t len);

int ts_engine_client_write_cloud_property(struct ts_engine_client *client,
    const char * const data, size_t len);

int ts_engine_client_reload_profile(struct ts_engine_client *client);

int ts_engine_client_stop(struct ts_engine_client *client);

int ts_engine_client_pause(struct ts_engine_client *client);

int ts_engine_client_continue(struct ts_engine_client *client);

int ts_engine_client_ping(struct ts_engine_client *client);

int ts_engine_client_connector_result_shm(struct ts_engine_client *client,
                                          void * const data, size_t len);

int __ts_engine_client_init(int *fdin, int *fdout);

#endif
