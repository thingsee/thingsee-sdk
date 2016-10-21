/****************************************************************************
 * apps/ts_engine/engine/client.c
 *
 * Copyright (C) 2015-2016 Haltian Ltd. All rights reserved.
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
 *   Pekka Ervasti <pekka.ervasti@haltian.com>
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <pthread.h>

#include "eng_dbg.h"
#include "main.h"
#include "client.h"
#include "util.h"

static pthread_mutex_t g_client_mutex = PTHREAD_MUTEX_INITIALIZER;

static void free_payload(struct ts_engine_client *client)
{
  if (client->payload.data)
    {
      free(client->payload.data);
      client->payload.data = NULL;
    }
}

static int send_req(struct ts_engine_client *client, uint8_t id,
    const void * const data, size_t len)
{
  struct ts_engine_client_req req;
  int ret;

  req.hdr.id = id;
  if (data)
    {
      req.hdr.length = len;
    }
  else
    {
      req.hdr.length = 0;
    }

  ret = __ts_engine_full_write(client->fdin, &req, sizeof(req));
  if (ret < 0)
    {
      eng_dbg ("__ts_engine_full_write failed\n");
      return ERROR;
    }

  if (req.hdr.length)
    {
      ret = __ts_engine_full_write(client->fdin, data, len);
      if (ret < 0)
        {
          eng_dbg ("__ts_engine_full_write failed\n");
          return ERROR;
        }
    }

  return OK;
}

static int receive_resp(struct ts_engine_client *client)
{
  struct ts_engine_client_resp *resp = &client->resp;
  int ret;

  ret = __ts_engine_full_read(client->fdout, resp, sizeof(*resp));
  if (ret != sizeof(*resp))
    {
      eng_dbg("read header failed: %d\n", ret);
      return ERROR;
    }

  if (resp->hdr.status != OK)
    {
      eng_dbg("resp->hdr.status: %d\n", resp->hdr.status);
    }

  if (resp->hdr.length)
    {
      free_payload(client);

      client->payload.data = malloc(resp->hdr.length);
      if (!client->payload.data)
        {
          perror("malloc");
          return ERROR;
        }

      ret = __ts_engine_full_read(client->fdout, client->payload.data,
          resp->hdr.length);
      if (ret < 0)
        {
          eng_dbg("__ts_engine_full_read failed\n");
          free_payload(client);
          return ERROR;
        }
    }

  return OK;
}

static int send_and_receive(struct ts_engine_client *client, uint8_t id,
    const void * const data, size_t len)
{
  int ret;

  pthread_mutex_lock(&g_client_mutex);
  ret = send_req(client, id, data, len);
  if (ret < 0)
    {
      eng_dbg("send_req failed\n");
      pthread_mutex_unlock(&g_client_mutex);
      return ERROR;
    }

  ret = receive_resp(client);
  if (ret < 0)
    {
      eng_dbg("receive_resp failed\n");
    }
  pthread_mutex_unlock(&g_client_mutex);

  return ret;
}

int ts_engine_client_init(struct ts_engine_client *client)
{
  memset(client, 0, sizeof(*client));

  client->fdin = open(TS_ENGINE_CTRL_FIFO_IN, O_WRONLY);
  if (client->fdin < 0)
    {
      perror("open");
      return ERROR;
    }

  client->fdout = open(TS_ENGINE_CTRL_FIFO_OUT, O_RDONLY);
  if (client->fdout < 0)
    {
      perror("open");
      close(client->fdin);
      return ERROR;
    }

  return OK;
}

void ts_engine_client_uninit(struct ts_engine_client *client)
{
  close(client->fdin);
  close(client->fdout);

  free_payload(client);
}

int ts_engine_client_read_profile(struct ts_engine_client *client)
{
  int ret;

  ret = send_and_receive(client, TS_ENGINE_READ_PROFILE, NULL, 0);
  if (ret < 0)
    {
      eng_dbg("send_and_receive failed\n");
      return ERROR;
    }

  return OK;
}

int ts_engine_client_read_cloud_property(struct ts_engine_client *client)
{
  int ret;

  ret = send_and_receive(client, TS_ENGINE_READ_CLOUD_PROPERTY, NULL, 0);
  if (ret < 0)
    {
      eng_dbg("send_and_receive failed\n");
      return ERROR;
    }

  return OK;
}

int ts_engine_client_read_device_property(struct ts_engine_client *client)
{
  int ret;

  ret = send_and_receive(client, TS_ENGINE_READ_DEVICE_PROPERTY, NULL, 0);
  if (ret < 0)
    {
      eng_dbg("send_and_receive failed\n");
      return ERROR;
    }

  return OK;
}

int ts_engine_client_write_profile(struct ts_engine_client *client,
    const char * const data, size_t len)
{
  int ret;

  ret = send_and_receive(client, TS_ENGINE_WRITE_PROFILE, data, len);
  if (ret < 0)
    {
      eng_dbg("send_and_receive failed\n");
      return ERROR;
    }

  return OK;
}

int ts_engine_client_write_profile_shm(struct ts_engine_client *client,
    char * const data, size_t len)
{
  int ret;
  struct ts_engine_shm_obj shm;

  shm.addr = data;
  shm.len = len;

  ret = send_and_receive(client, TS_ENGINE_WRITE_PROFILE_SHM, &shm, sizeof(shm));
  if (ret < 0)
    {
      eng_dbg("send_and_receive failed\n");
      return ERROR;
    }

  return OK;
}

int ts_engine_client_write_cloud_property(struct ts_engine_client *client,
    const char * const data, size_t len)
{
  int ret;

  ret = send_and_receive(client, TS_ENGINE_WRITE_CLOUD_PROPERTY, data, len);
  if (ret < 0)
    {
      eng_dbg("send_and_receive failed\n");
      return ERROR;
    }

  return OK;
}

int ts_engine_client_reload_profile(struct ts_engine_client *client)
{
  int ret;

  ret = send_and_receive(client, TS_ENGINE_RELOAD_PROFILE, NULL, 0);
  if (ret < 0)
    {
      eng_dbg("send_and_receive failed\n");
      return ERROR;
    }

  return OK;
}

int ts_engine_client_stop(struct ts_engine_client *client)
{
  int ret;

  ret = send_and_receive(client, TS_ENGINE_STOP, NULL, 0);
  if (ret < 0)
    {
      eng_dbg("send_and_receive failed\n");
      return ERROR;
    }

  return OK;
}

int ts_engine_client_pause(struct ts_engine_client *client)
{
  int ret;

  ret = send_and_receive(client, TS_ENGINE_PAUSE, NULL, 0);
  if (ret < 0)
    {
      eng_dbg("send_and_receive failed\n");
      return ERROR;
    }

  return OK;
}

int ts_engine_client_continue(struct ts_engine_client *client)
{
  int ret;

  ret = send_and_receive(client, TS_ENGINE_CONTINUE, NULL, 0);
  if (ret < 0)
    {
      eng_dbg("send_and_receive failed\n");
      return ERROR;
    }

  return OK;
}

int ts_engine_client_ping(struct ts_engine_client *client)
{
  int ret;

  ret = send_and_receive(client, TS_ENGINE_PING, NULL, 0);
  if (ret < 0)
    {
      eng_dbg("send_and_receive failed\n");
      return ERROR;
    }

  return OK;
}

int ts_engine_client_connector_result_shm(struct ts_engine_client *client,
                                          void * const data, size_t len)
{
  struct ts_engine_shm_obj shm;
  int ret;

  shm.addr = data;
  shm.len = len;

  ret = send_and_receive(client, TS_ENGINE_CONNECTOR_RESULT_SHM, &shm, sizeof(shm));
  if (ret < 0)
    {
      eng_dbg("send_and_receive failed\n");
      return ERROR;
    }

  return OK;
}

int __ts_engine_client_init(int *fdin, int *fdout)
{
  int ret;

  ret = mkfifo(TS_ENGINE_CTRL_FIFO_IN, 0);
  if (ret != OK)
    {
      perror("mkfifo");
      return ERROR;
    }

  *fdin = open(TS_ENGINE_CTRL_FIFO_IN, O_RDWR); /* O_RDONLY would block */
  if (*fdin < 0)
    {
      perror("open");
      return ERROR;
    }

  ret = mkfifo(TS_ENGINE_CTRL_FIFO_OUT, 0);
  if (ret != OK)
    {
      perror("mkfifo");
      return ERROR;
    }

  *fdout = open(TS_ENGINE_CTRL_FIFO_OUT, O_WRONLY);
  if (*fdout < 0)
    {
      perror("open");
      return ERROR;
    }

  return OK;
}
