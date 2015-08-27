/****************************************************************************
 * apps/thingsee/bluetooth/bluetooth.c
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
 *   Author: Harri Luhtala <harri.luhtala@haltian.com>
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
#include <stdlib.h>
#include <sys/types.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <string.h>
#include <debug.h>
#include <poll.h>

#include <arch/board/board-bt.h>
#include <apps/thingsee/modules/ts_bluetooth.h>

#include "btle.h"
#include "btle_receiver.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char * bluetooth_conn_state_name[__BTLE_STATE_MAX] =
{
  [BTLE_STATE_INIT]                   = "INIT",
  [BTLE_STATE_STARTED]                = "STARTED",
  [BTLE_STATE_ADVERTISING]            = "ADVERTISING",
  [BTLE_STATE_ADVERTISING_NONCONN]    = "ADVERTISING_NONCONN",
  [BTLE_STATE_WAITING]                = "WAITING",
  [BTLE_STATE_WAITING_AFTER_TIMEOUT]  = "WAITING_AFTER_TIMEOUT",
  [BTLE_STATE_CONNECTED]              = "CONNECTED",
  [BTLE_STATE_CONNECTED_ADV]          = "CONNECTED_ADV",
  [BTLE_STATE_ERROR]                  = "ERROR"
};

/* BT LE internal data structure */

static struct btle_s g_btle;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Name: bluetooth_set_nonblocking
 *
 * Description:
 *   Set BT file descriptor non-blocking or blocking
 *
 * Input Parameters:
 *   btle        - BTLE object
 *   set         - Set non-blocking if true; set blocking if false
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
static int bluetooth_set_nonblocking(struct btle_s * const btle, bool const set)
{
  int flags, ret;

  DEBUGASSERT(btle->fd >= 0);

  if (set == btle->is_nonblocking)
    return OK;

  flags = fcntl(btle->fd, F_GETFL, 0);
  if (flags == ERROR)
    return ERROR;

  if (!set)
    flags &= ~O_NONBLOCK;
  else
    flags |= O_NONBLOCK;

  ret = fcntl(btle->fd, F_SETFL, flags);
  if (ret == ERROR)
    return ERROR;

  btle->is_nonblocking = set;

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bluetooth_initialize
 *
 * Description:
 *   Initialize BT LE module
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
struct btle_s *bluetooth_initialize(void)
{
  struct btle_s * const btle = &g_btle;
  int error;
  int ret;

  /* Check if BT is already powered */

  if (btle->fd > 0)
    {
      return btle;
    }

  /* Clear data */

  memset(btle, 0, sizeof(struct btle_s));

  /* Power On BT LE */

  board_bt_power_device_up(true);

  /* Open BT LE serial device */

  btle->fd = board_bt_init_dev();
  error = get_errno();
  if (btle->fd < 0)
    {
      dbg("Failed to open BT LE device: %d\n", error);
      return NULL;
    }

  /* Set file blocking */

  ret = bluetooth_set_nonblocking(btle, false);
  if (ret < 0)
    {
      goto errout_close;
    }

  /* Initialize BT LE receiver */

  ret = btle_receiver_initialize(btle, btle);
  if (ret < 0)
    {
      dbg("returned %d\n", ret);
      goto errout_close;
    }

  return btle;

errout_close:

  /* Close BTLE device */

  close(btle->fd);
  btle->fd = -1;

  return NULL;
}

/****************************************************************************
 * Name: bluetooth_uninitialize
 *
 * Description:
 *   Uninitialize BT LE module
 *
 ****************************************************************************/
void bluetooth_uninitialize(struct btle_s * const btle)
{
  int ret;

  /* Check if BT is powered */

  if (btle->fd < 0)
    {
      return;
    }

  /* Power Off BT LE */

  board_bt_power_device_up(false);

  ret = btle_receiver_reset(btle);
  if (ret)
    {
      dbg("receiver reset failed: %d\n", ret);
    }

  ret = board_bt_deinit_dev(btle->fd);
  if (ret)
    {
      dbg("device deinit failed: %d\n", ret);
    }

  btle->fd = -1;
}

/****************************************************************************
 * Name: bluetooth_callback_register
 *
 * Description:
 *   Register callback function for BT events and data retrieval
 *
 * Input Parameters:
 *   event_mask  - Mask of subscribed events generated with BT macro
 *   callback    - Callback function
 *   priv        - Pointer to private data
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int bluetooth_callback_register(uint32_t const event_mask,
                                bluetooth_callback_t callback,
                                void * const priv)
{
  struct btle_s * const bt = &g_btle;
  struct bluetooth_callback_entry_s * cb;
  uint32_t mask = 0;
  struct bluetooth_callback_entry_s * cb_reuse = NULL;

  /* Check input data */

  if (!callback)
    return ERROR;

  /* Check if callback is already registered */

  cb = (struct bluetooth_callback_entry_s *)sq_peek(&bt->callbacks);
  while (cb)
    {
      if (cb->callback == callback)
        {
          if (cb->event_mask != 0)
            {
              return ERROR;
            }

          /* Reuse unactive callback */

          cb_reuse = cb;
        }
      else
        {
          /* Add to callback events to mask */

          mask |= cb->event_mask;
        }

      /* Move to next callback */

      cb = (struct bluetooth_callback_entry_s *)sq_next(&cb->entry);
    }

  /* Update event mask for all callbacks */

  bt->callback_event_mask = mask | event_mask;

  /* Setup new callback */

  if (cb_reuse)
    {
      sq_rem(&cb_reuse->entry, &bt->callbacks);
      cb = cb_reuse;
    }
  else
    {
      cb = malloc(sizeof(struct bluetooth_callback_entry_s));
      if (cb == NULL)
        {
          return ERROR;
        }
    }

  cb->event_mask = event_mask;
  cb->callback = callback;
  cb->priv = priv;

  /* Add callback to queue */

  sq_addlast(&cb->entry, &bt->callbacks);

  return OK;
}

/****************************************************************************
 * Name: bluetooth_callback_unregister
 *
 * Description:
 *   Unregister callback from BT module
 *
 * Input Parameters:
 *   callback    - Callback function
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/

int bluetooth_callback_unregister(bluetooth_callback_t callback)
{
  struct btle_s * const bt = &g_btle;
  struct bluetooth_callback_entry_s * cb, * cbnext;
  uint32_t mask = 0;
  int status = ERROR;

  /* Check input data */

  if (!callback)
    return ERROR;

  /* Search for callback in queue */

  cb = (struct bluetooth_callback_entry_s *)sq_peek(&bt->callbacks);
  while (cb)
    {
      /* Save next callback entry */
      cbnext = (struct bluetooth_callback_entry_s *)sq_next(&cb->entry);

      if (cb->callback == callback)
        {
          /* Mark callback as non-active. We cannot free callback yet, as
           * unregister might have been called from callback itself.
           */

          cb->event_mask = 0;

          /* Mark successful operation */

          status = OK;
        }
      else
        {
          /* Add to callback events to mask */

          mask |= cb->event_mask;
        }

      /* Move to next callback */

      cb = cbnext;
    }

  /* Update event mask for all callbacks */

  bt->callback_event_mask = mask;

  return status;
}

/****************************************************************************
 * Name: bluetooth_get_state
 *
 * Description:
 *   Get current BT connection state
 *
 * Returned Values:
 *   BT state
 *
 ****************************************************************************/

bluetooth_state_t bluetooth_get_state(void)
{
  struct btle_s const * const btle = &g_btle;

  return btle->state;
}

/****************************************************************************
 * Name: bluetooth_get_conn_state_name
 *
 * Description:
 *  Get connection state name
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/

char const * const bluetooth_get_conn_state_name(bluetooth_state_t state)
{
  if (state < 0 || state >= __BTLE_STATE_MAX)
    return NULL;

  return bluetooth_conn_state_name[state];
}

/****************************************************************************
 * Name: bluetooth_receiver
 *
 * Description:
 *   Handle input from BT LE module
 *
 * Input Parameters:
 *   pfd         - Poll structure
 *   priv        - Private data
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int bluetooth_receiver(const struct pollfd * const pfd, void * const priv)
{
  struct btle_s * const btle = (struct btle_s*)priv;
  uint8_t input_buffer[32];
  ssize_t input_bytes;
  uint8_t * input;
  int flags;
  int ret;
#ifdef BTLE_DEBUG
  uint8_t i;
#endif

  /* Get file flags */
  flags = fcntl(pfd->fd, F_GETFL, 0);
  if (flags == ERROR)
    return ERROR;

  while (1)
    {
      /* Set file non-blocking */

      ret = bluetooth_set_nonblocking(btle, true);

      /* Read bytes from BTLE module */

      input_bytes = read(pfd->fd, input_buffer, sizeof(input_buffer));
      if (input_bytes <= 0)
        break;
#ifdef BTLE_DEBUG
      dbg("input_bytes:%d\n", input_bytes);
      for(i=0; i < input_bytes; i++)
        {
          dbg("0x%02X\n", input_buffer[i]);
        }
#endif
      input = input_buffer;

      /* Clear rx_handler if there are no payload bytes left */

      if (btle->payload_len == 0)
        {
          btle->rx_handler = NULL;
        }

      /* Setup receiver handler according to packet type */

      if (!btle->rx_handler && input_bytes >= 1)
        {
          if (btle_setup_rx_handler(btle, &input, &input_bytes))
            {
              continue;
            }
        }

      /* Set file blocking */

      ret = bluetooth_set_nonblocking(btle, false);
      if (ret < 0)
        return ERROR;

      while (input_bytes)
        {
          if (!btle->payload_len)
            {
              /* Setup handler for the next packet */

              if (btle_setup_rx_handler(btle, &input, &input_bytes))
                {
                  break;
                }
            }

          /* Invoke handler */

          (void)btle->rx_handler(btle, *input++);

          input_bytes--;
          btle->payload_len--;
        }
    }

  /* Set file blocking */

  ret = bluetooth_set_nonblocking(btle, false);

  if (ret < 0)
    return ERROR;

  return OK;
}
