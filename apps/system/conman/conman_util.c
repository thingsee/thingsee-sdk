/****************************************************************************
 * apps/system/conman/conman_util.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <unistd.h>
#include <poll.h>
#include <errno.h>

#include <conman_dbg.h>
#include "conman_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __conman_util_block_read
 *
 * Description:
 *   Reads from the file desciptor making sure all requested data is received.
 *
 * Input Parameters:
 *   fd     : file descriptor to read from
 *   buffer : buffer to read the data to
 *   len    : number of bytes to read
 *
 * Returned Value:
 *   OK if success.
 *   ERROR if failed.
 *
 ****************************************************************************/

int __conman_util_block_read(int fd, void *buffer, size_t len)
{
  int toread;
  int ret;
  char *buf;

  buf = (char *)buffer;
  toread = len;

  do
    {
      ret = recv(fd, buf + len - toread, toread, 0);
      if (ret < 0)
        {
          conman_dbg("recv failed %d\n", errno);
          return ERROR;
        }

      toread -= ret;
    } while (toread > 0);

  return OK;
}

/****************************************************************************
 * Name: __conman_util_block_write
 *
 * Description:
 *   Writes to the file desciptor making sure all requested data is written.
 *
 * Input Parameters:
 *   fd     : file descriptor to write to
 *   buffer : buffer to read the data from
 *   len    : number of bytes to write
 *
 * Returned Value:
 *   OK if success.
 *   ERROR if failed.
 *
 ****************************************************************************/

int __conman_util_block_write(int fd, const void * const buffer, size_t len)
{
  int towrite;
  int ret;
  char *buf;

  if (len == 0)
    {
      return OK;
    }

  buf = (char *)buffer;
  towrite = len;

  do
    {
      ret = send(fd, buf + len - towrite, towrite, 0);
      if (ret < 0)
        {
          conman_dbg("send failed %d\n", errno);
          return ERROR;
        }

      towrite -= ret;
    } while (towrite > 0);

  return OK;
}

/****************************************************************************
 * Name: __conman_send_resp
 *
 * Description:
 *   Sends a reply to the client by writing it to the out fifo.
 *
 * Input Parameters:
 *   conman  : connection manager handle
 *   id      : message id
 *   respval : response value
 *
 * Returned Value:
 *   OK    : no errors
 *   ERROR : failure
 *
 ****************************************************************************/

int __conman_send_resp(struct conman_s *conman, uint8_t id,
                     enum conman_resp_value respval, void *data,
                     size_t datalen)
{
  struct conman_resp_hdr hdr;
  int ret;

  hdr.head.id = id;
  hdr.head.len = datalen;
  hdr.respval = respval;

  ret = __conman_util_block_write(conman->servingsd, &hdr, sizeof(hdr));
  if (ret < 0)
    {
      return ERROR;
    }

  ret = __conman_util_block_write(conman->servingsd, data, hdr.head.len);
  if (ret < 0)
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: __conman_send_boardcast_event
 *
 * Description:
 *   Send boardcast event to all open client sockets that have events enabled.
 *
 * Input Parameters:
 *   conman  : connection manager handle
 *   type    : event type
 *
 ****************************************************************************/

void __conman_send_boardcast_event(struct conman_s *conman,
                                   enum conman_msgs_ids type,
                                   const void *payload,
                                   size_t payloadlen)
{
  struct conman_sd_entry_s *client;
  struct conman_resp_hdr hdr = {};
  int send_count = 0;
  int ret;

  hdr.head.id = type;
  hdr.head.len = payloadlen;
  hdr.respval = CONMAN_RESP_EVENT;

  for (client = (struct conman_sd_entry_s *)sq_peek(&conman->server.sds);
       client != NULL;
       client = (struct conman_sd_entry_s *)sq_next(&client->entry))
    {
      if (!client->events_enabled)
        {
          continue;
        }

      ret = __conman_util_block_write(client->sd, &hdr, sizeof(hdr));
      if (ret < 0)
        {
          continue;
        }

      ret = __conman_util_block_write(client->sd, payload, hdr.head.len);
      if (ret < 0)
        {
          continue;
        }

      send_count++;
    }

  conman_dbg("send boardcast event (type=%d) to %d clients.\n",
             type, send_count);
}

/****************************************************************************
 * Name: __conman_flushfd
 *
 * Description:
 *   Flushes the file descriptor by reading it empty.
 *
 * Input Parameters:
 *   fd : file descriptor
 *
 ****************************************************************************/

void __conman_flushfd(int fd)
{
  int ret;
  char tmp;

  do
    {
      struct pollfd pfd;

      pfd.fd = fd;
      pfd.events = POLLIN;

      ret = poll(&pfd, 1, 0);
      if (ret == 1 && (pfd.revents & POLLIN))
        {
          ret = recv(fd, &tmp, 1, 0);
        }
      else
        {
          break;
        }
    } while (true);
}
