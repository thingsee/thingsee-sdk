/****************************************************************************
 * apps/system/conman/conman_server.c
 *
 *   Copyright (C) 2015-2016 Haltian Ltd. All rights reserved.
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

#include <stdlib.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <string.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <errno.h>

#include "conman_dbg.h"
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
 * Name: __conman_server_initialize
 *
 * Description:
 *
 * Input Parameters:
 *   conman   : connection manager handle
 *   max_pfds : output max number of socket descriptors
 *
 * Returned Value:
 *   OK    : no errors
 *   ERROR : failure
 *
 ****************************************************************************/

int __conman_server_initialize(struct conman_s *conman, int *max_pfds)
{
  struct sockaddr_un myaddr;
  socklen_t addrlen;
  int ret;

  /* Create a new Unix domain socket */

  conman->listensd = socket(PF_LOCAL, SOCK_STREAM, 0);
  if (conman->listensd < 0)
    {
      conman_dbg("server: socket failure: %d\n", errno);
      return ERROR;
    }

  /* Bind the socket to a local address */

  addrlen = strlen(CONFIG_CONMAN_LISTEN_SOCKET_ADDR);
  if (addrlen > UNIX_PATH_MAX - 1)
    {
      addrlen = UNIX_PATH_MAX - 1;
    }

  myaddr.sun_family = AF_LOCAL;
  strncpy(myaddr.sun_path, CONFIG_CONMAN_LISTEN_SOCKET_ADDR, addrlen);
  myaddr.sun_path[addrlen] = '\0';

  addrlen += sizeof(sa_family_t) + 1;
  ret = bind(conman->listensd, (struct sockaddr*) &myaddr, addrlen);
  if (ret < 0)
    {
      conman_dbg("bind failed: %d\n", errno);
      goto errout_with_listensd;
    }

  /* Listen for connections on the bound socket */

  conman_dbg("Listening connections on %s\n", CONFIG_CONMAN_LISTEN_SOCKET_ADDR);

  ret = listen(conman->listensd, 5);
  if (ret < 0)
    {
      conman_dbg("listen failed: %d\n", errno);
      goto errout_with_listensd;
    }

  *max_pfds = 1 + CONFIG_CONMAN_CLIENTS_MAX;

  sq_init(&conman->server.sds);

  return OK;

errout_with_listensd:

  close(conman->listensd);
  *max_pfds = 0;

  return ERROR;
}

/****************************************************************************
 * Name: __conman_server_setup_pollfds
 *
 * Description:
 *
 * Input Parameters:
 *   conman      : connection manager handle
 *   pfds        :
 *   maxfds      :
 *   fds_pos     :
 *   min_timeout :
 *
 * Returned Value:
 *
 ****************************************************************************/

void __conman_server_setup_pollfds(struct conman_s *conman,
                                   struct pollfd *pfds, int maxfds,
                                   int *fds_pos, int *min_timeout)
{
  struct conman_sd_entry_s *sd_entry;

  sd_entry = (struct conman_sd_entry_s *)sq_peek(&conman->server.sds);

  while (sd_entry)
    {
      pfds[*fds_pos].fd = sd_entry->sd;
      pfds[*fds_pos].events = POLLIN;
      pfds[*fds_pos].revents = 0;
      (*fds_pos)++;

      sd_entry = (struct conman_sd_entry_s *)sq_next(&sd_entry->entry);
    }
}

/****************************************************************************
 * Name: __conman_server_handle_pollin
 *
 * Description:
 *
 * Input Parameters:
 *   conman : connection manager handle
 *   pfd    : pollfd structure
 *
 * Returned Value:
 *
 ****************************************************************************/

void __conman_server_handle_pollin(struct conman_s *conman, struct pollfd *pfd)
{
  int new_sd;
  struct conman_sd_entry_s *sd_entry;

  if (!(pfd->revents & POLLIN))
    return;

  conman_dbg("Attempt accept connection for %s\n",
             CONFIG_CONMAN_LISTEN_SOCKET_ADDR);

  new_sd = accept(conman->listensd, NULL, NULL);
  if (new_sd < 0)
    {
      if (errno != EWOULDBLOCK)
        {
          conman_dbg("accept failed: %d\n", errno);
        }
      return;
    }

  conman_dbg("New incoming connection: %d\n", new_sd);

  sd_entry = calloc(1, sizeof(*sd_entry));
  if (!sd_entry)
    {
      conman_dbg("malloc %d failed\n", sizeof(*sd_entry));
      close(new_sd);
      return;
    }

  sd_entry->sd = new_sd;
  sq_addlast(&sd_entry->entry, &conman->server.sds);
}

/****************************************************************************
 * Name: __conman_server_close_sd
 *
 * Description:
 *
 * Input Parameters:
 *   conman : connection manager handle
 *   sd     : socket descriptor
 *
 * Returned Value:
 *   OK    : no errors
 *   ERROR : failure
 *
 ****************************************************************************/

int __conman_server_close_sd(struct conman_s *conman, int sd)
{
  struct conman_sd_entry_s *sd_entry;

  sd_entry = (struct conman_sd_entry_s *)sq_peek(&conman->server.sds);

  while (sd_entry)
    {
      if (sd_entry->sd == sd)
        {
          conman_dbg("closing client sd: %d\n", sd);

          close(sd);
          sq_rem(&sd_entry->entry, &conman->server.sds);
          free(sd_entry);
          return OK;
        }

      sd_entry = (struct conman_sd_entry_s *)sq_next(&sd_entry->entry);
    }

  return ERROR;
}

/****************************************************************************
 * Name: __conman_server_enable_events_for_client
 *
 * Description:
 *
 * Input Parameters:A
 *   conman : connection manager handle
 *   sd     : socket descriptor
 *
 * Returned Value:
 *   OK    : no errors
 *   ERROR : failure
 *
 ****************************************************************************/

int __conman_server_enable_events_for_client(struct conman_s *conman, int sd)
{
  struct conman_sd_entry_s *sd_entry;

  sd_entry = (struct conman_sd_entry_s *)sq_peek(&conman->server.sds);

  while (sd_entry)
    {
      if (sd_entry->sd == sd)
        {
          sd_entry->events_enabled = true;
          return OK;
        }

      sd_entry = (struct conman_sd_entry_s *)sq_next(&sd_entry->entry);
    }

  return ERROR;
}
