/****************************************************************************
 * net/usrsock/usrsock_socket.c
 *
 *  Copyright (C) 2015 Haltian Ltd. All rights reserved.
 *  Author: Jussi Kivilinna <jussi.kivilinna@haltian.com>
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_USRSOCK)

#include <stdint.h>
#include <string.h>
#include <semaphore.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>

#include <sys/socket.h>
#include <nuttx/net/net.h>
#include <nuttx/net/usrsock.h>

#include "devif/devif.h"
#include "usrsock/usrsock.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint16_t socket_event(FAR struct net_driver_s *dev, FAR void *pvconn,
                             FAR void *pvpriv, uint16_t flags)
{
  FAR struct usrsock_reqstate_s *pstate = pvpriv;
  FAR struct usrsock_conn_s *conn = pvconn;

  if (flags & USRSOCK_EVENT_ABORT)
    {
      ndbg("socket aborted.\n");

      pstate->result = -ENETDOWN;

      /* Stop further callbacks */

      pstate->cb->flags   = 0;
      pstate->cb->priv    = NULL;
      pstate->cb->event   = NULL;

      /* Wake up the waiting thread */

      sem_post(&pstate->recvsem);
    }
  else if (flags & USRSOCK_EVENT_REQ_COMPLETE)
    {
      ndbg("request completed.\n");

      pstate->result = conn->resp.result;
      if (pstate->result >= 0)
        {
          /* We might start getting events for this socket right after
           * returning to daemon, so setup 'conn' already here. */

          conn->state   = USRSOCK_CONN_STATE_READY;
          conn->usockid = pstate->result;
        }

      /* Stop further callbacks */

      pstate->cb->flags   = 0;
      pstate->cb->priv    = NULL;
      pstate->cb->event   = NULL;

      /* Wake up the waiting thread */

      sem_post(&pstate->recvsem);
    }

  return flags;
}

/****************************************************************************
 * Name: do_socket_request
 ****************************************************************************/

static int do_socket_request(FAR struct usrsock_conn_s *conn, int domain,
                             int type, int protocol)
{
  struct usrsock_request_socket_s req = {};
  struct iovec bufs[1];

  /* Prepare request for daemon to read. */

  req.head.reqid = USRSOCK_REQUEST_SOCKET;
  req.domain = domain;
  req.type = type;
  req.protocol = protocol;

  if (req.domain != domain)
    return -EINVAL;
  if (req.type != type)
    return -EINVAL;
  if (req.protocol != protocol)
    return -EINVAL;

  bufs[0].iov_base = (FAR void *)&req;
  bufs[0].iov_len = sizeof(req);

  return usrsockdev_do_request(conn, bufs, ARRAY_SIZE(bufs));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: usrsock_socket
 *
 * Description:
 *   socket() creates an endpoint for communication and returns a socket
 *   structure.
 *
 * Parameters:
 *   domain   (see sys/socket.h)
 *   type     (see sys/socket.h)
 *   protocol (see sys/socket.h)
 *   psock    A pointer to a user allocated socket structure to be initialized.
 *
 * Returned Value:
 *   0 on success; negative error-code on error
 *
 *   EACCES
 *     Permission to create a socket of the specified type and/or protocol
 *     is denied.
 *   EAFNOSUPPORT
 *     The implementation does not support the specified address family.
 *   EINVAL
 *     Unknown protocol, or protocol family not available.
 *   EMFILE
 *     Process file table overflow.
 *   ENFILE
 *     The system limit on the total number of open files has been reached.
 *   ENOBUFS or ENOMEM
 *     Insufficient memory is available. The socket cannot be created until
 *     sufficient resources are freed.
 *   EPROTONOSUPPORT
 *     The protocol type or the specified protocol is not supported within
 *     this domain.
 *
 * Assumptions:
 *
 ****************************************************************************/

int usrsock_socket(int domain, int type, int protocol, FAR struct socket *psock)
{
  struct usrsock_reqstate_s state = {};
  FAR struct usrsock_conn_s *conn;
  net_lock_t save;
  int err;

  /* Allocate the usrsock socket connection structure and save in the new
   * socket instance.
   */
  conn = usrsock_alloc();
  if (!conn)
    {
      /* Failed to reserve a connection structure */

      return -ENOMEM;
    }

  save = net_lock();

  /* Set up event callback for usrsock. */

  err = usrsock_setup_request_callback(conn, &state, socket_event,
                                       USRSOCK_EVENT_ABORT |
                                       USRSOCK_EVENT_REQ_COMPLETE);
  if (err < 0)
    {
      goto errout_free_conn;
    }

  /* Request user-space daemon for new socket. */

  err = do_socket_request(conn, domain, type, protocol);
  if (err < 0)
    {
      goto errout_teardown_callback;
    }

  /* Wait for completion of request. */

  while (net_lockedwait(&state.recvsem) != OK)
    {
      DEBUGASSERT(*get_errno_ptr() == EINTR);
    }

  if (state.result < 0)
    {
      err = state.result;
      goto errout_teardown_callback;
    }

  psock->s_type = SOCK_USRSOCK_TYPE;
  psock->s_domain = PF_USRSOCK_DOMAIN;
  conn->type    = type;
  psock->s_conn = conn;
  conn->crefs   = 1;

  usrsock_teardown_request_callback(&state);

  net_unlock(save);

  return OK;

errout_teardown_callback:
  usrsock_teardown_request_callback(&state);
errout_free_conn:
  usrsock_free(conn);
  net_unlock(save);

  return err;
}

#endif /* CONFIG_NET && CONFIG_NET_USRSOCK */
