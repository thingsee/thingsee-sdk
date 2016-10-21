/****************************************************************************
 * apps/system/conman/conman_main.c
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

static struct conman_s g_conman;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: conman_main
 *
 * Description:
 *   Connection manager entry point.
 *
 * Input Parameters:
 *   argc : number of arguments in the argv vector
 *   argv : arguments
 *
 * Returned Value:
 *   EXIT_FAILURE on errors
 *
 ****************************************************************************/

int conman_main(int argc, char **argv)
{
  struct conman_s *conman = &g_conman;
  int ubmodem_max_pfds = 0;
  int client_max_pfds = 0;
  struct pollfd *pfds;
  int ret;
  int maxfds = 0;
  int i;

  __conman_config_init(conman);

  ret = __conman_server_initialize(conman, &client_max_pfds);
  if (ret != OK)
    {
      conman_dbg("__conman_server_initialize failed");
      return EXIT_FAILURE;
    }

  maxfds += client_max_pfds; /* client fds */

  ret = __conman_ubmodem_initialize(conman, &ubmodem_max_pfds);
  if (ret != OK)
    {
      conman_dbg("__conman_ubmodem_initialize failed\n");
      return EXIT_FAILURE;
    }

  maxfds += ubmodem_max_pfds; /* modem fds */

  maxfds += __conman_cc3000_get_max_pollfds(conman); /* wifi fds */

  pfds = calloc(maxfds, sizeof(*pfds));
  if (!pfds)
    {
      conman_dbg("ubmodemd_daemon: Out-of-memory, tried to allocate %d.\n",
                 maxfds * sizeof(*pfds));

      return EXIT_FAILURE;
    }

  while (true)
    {
      int ubmodem_pos_fds, ubmodem_num_fds;
      int cc3000_pos_fds, cc3000_num_fds;
      int client_pos_fds, client_num_fds;
      int nfds;
      int timeout;

      nfds = 0;
      timeout = INT_MAX;

      /* Setup pollfd for listening socket */

      pfds[nfds].fd = conman->listensd;
      pfds[nfds].events = POLLIN;
      nfds++;

      /* Setup pollfds for clients and get timeout for timers. */

      client_pos_fds = nfds;
      __conman_server_setup_pollfds(conman, pfds, maxfds, &nfds, &timeout);
      client_num_fds = nfds - client_pos_fds;

      /* Setup pollfds for modem and get timeout for timers. */

      ubmodem_pos_fds = nfds;
      __conman_ubmodem_setup_pollfds(conman, pfds, maxfds, &nfds, &timeout);
      ubmodem_num_fds = nfds - ubmodem_pos_fds;

      /* Setup pollfds for cc3000 and get timeout for timers. */

      cc3000_pos_fds = nfds;
      __conman_cc3000_setup_pollfds(conman, pfds, maxfds, &nfds, &timeout);
      cc3000_num_fds = nfds - cc3000_pos_fds;

      ret = poll(pfds, nfds, timeout == INT_MAX ? -1 : timeout);
      if (ret < 0)
        {
          conman_dbg("poll failed %d\n", errno);
          break;
        }

      if (ret == 0)
        {
          conman_dbg("poll() timed out\n");
          __conman_ubmodem_poll_timedout(conman);
          continue;
        }

      for (i = 0; i < nfds && ret > 0; i++)
        {
          if (pfds[i].revents)
            {
              ret--;

              conman_dbg("poll index: %d, fd: %d, "
                         "events: 0x%02x, revents: 0x%02x\n",
                         i, pfds[i].fd, pfds[i].events, pfds[i].revents);
            }

          if (pfds[i].revents && pfds[i].fd == conman->listensd)
            {
              __conman_server_handle_pollin(conman, &pfds[i]);
            }
          else if (pfds[i].revents && client_num_fds > 0 &&
                   i >= client_pos_fds && i < (client_pos_fds + client_num_fds))
            {
              __conman_ctl_handle_pollin(conman, &pfds[i]);
            }
          else if (pfds[i].revents && cc3000_num_fds > 0 &&
                   i >= cc3000_pos_fds && i < (cc3000_pos_fds + cc3000_num_fds))
            {
              __conman_cc3000_handle_pollfds(conman, &pfds[i]);
            }
          else if (ubmodem_pos_fds == i && ubmodem_num_fds > 0)
            {
              int j;

              for (j = 1; j < ubmodem_num_fds; j++)
                {
                  if (pfds[i + j].revents)
                    {
                      ret--;
                    }
                }

              __conman_ubmodem_handle_pollfds(conman, &pfds[i],
                                              ubmodem_num_fds);

              i += ubmodem_num_fds - 1;
            }
        }
    }

  /* Reached only on poll() error */

  free(pfds);
  return EXIT_FAILURE;
}
