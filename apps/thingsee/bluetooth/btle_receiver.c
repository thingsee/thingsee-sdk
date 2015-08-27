/****************************************************************************
 * apps/thingsee/bluetooth/btle_receiver.c
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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: btle_publish_event
 *
 * Description:
 *   Publish event to registered callbacks
 *
 * Input Parameters:
 *   btle        - BT object
 *   event       - Pointer to published event
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
static void btle_publish_event(struct btle_s * const btle,
                               struct bluetooth_event_s const * const event)
{
  struct bluetooth_callback_entry_s const * cb;
  DEBUGASSERT(btle && event);
#ifdef BTLE_DEBUG
  dbg("-> %d\n", event->id);
#endif

  /* Check if any callback is interested of this event */

  if (!(btle->callback_event_mask & event->id))
    return;

  cb = (struct bluetooth_callback_entry_s *)sq_peek(&btle->callbacks);
  while (cb)
    {
      if ((cb->event_mask & event->id) && cb->callback)
        cb->callback(event, cb->priv);

      cb = (struct bluetooth_callback_entry_s *)sq_next(&cb->entry);
    }
}

/****************************************************************************
 * Name: btle_send_response
 *
 * Description:
 *
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
static int btle_send_response(struct btle_s * const btle,
                       uint8_t seq_num,
                       btle_ack_t reason)
{
  DEBUGASSERT(btle);
  struct btle_ack_msg_s* ack;
  int status = ERROR;
  uint8_t* msg;
  uint8_t i;

  if (btle->fd < 1)
    return status;

  ack = (struct btle_ack_msg_s*)malloc(sizeof(struct btle_ack_msg_s));

  if (!ack)
    {
      dbg("response message alloc failed\n");
      return ERROR;
    }

  /* Frame header for btle tranport */

  ack->type = BTLE_MSG_DATA;
  ack->packet_len = 7;

  /* Encapsulated data */

  ack->msg_id = BTLE_DATA_ACK;
  ack->seq_num = seq_num;
  ack->length = 1;
  ack->reason = reason;

  /* Calculate checksum */

  ack->ck_a = 0;
  ack->ck_b = 0;
  msg = (uint8_t*)ack;

  for (i=0; i < offsetof(struct btle_ack_msg_s, ck_a); i++)
    {
      ack->ck_a = ack->ck_a + msg[i];
      ack->ck_b = ack->ck_a + ack->ck_b;
    }

  status = write(btle->fd, ack, sizeof(struct btle_ack_msg_s));
  if (status != sizeof(struct btle_ack_msg_s))
    {
      status = ERROR;
    }
  else
    {
      status = OK;
    }

  free(ack);

  return status;
}

/****************************************************************************
 * Name: btle_transfer_timeout
 *
 * Description:
 *   BT LE data transfer timeout handling
 *
 ****************************************************************************/
static int btle_transfer_timeout(const int timer_id, void * const priv)
{
  struct btle_s * const btle = (struct btle_s *)priv;

  /* Was this a BTLE message receive timeout */

  if (timer_id == btle->data_receiver.receive_timer_id)
    {
      dbg("state:%u at index:%u/%u\n", btle->data_receiver.state.state,
          btle->data_receiver.state.offset, btle->data_receiver.state.header.length);

      /* Reset timer ID */

      btle->data_receiver.receive_timer_id = -1;

      if( btle_send_response(btle, btle->data_receiver.state.header.seq_num, BTLE_ACK_TIMEOUT) != 0)
        {
          dbg("Response send failed\n");
        }

      /* Free pending BTLE message */

      if (btle->data_receiver.state.state >= BTLE_STATE_MSG_PAYLOAD)
        {
          /* Free allocated message buffer */

          free(btle->data_receiver.state.msg);
        }

      btle->data_receiver.state.state = BTLE_STATE_MSG_WAIT;
      btle->data_receiver.state.ck_a = 0;
      btle->data_receiver.state.ck_b = 0;
    }

  return OK;
}

/****************************************************************************
 * Name: btle_data_received
 *
 * Description:
 *   This function gets called when data frame successfully received.
 *
 * Input Parameters:
 *   receiver    - BTLE object
 *   priv        - Private data
 *   msg         - BTLE message
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
static int btle_data_received(struct btle_data_receiver_s const * const receiver,
                              struct btle_s * const btle,
                              struct btle_msg_s const * const msg)
{
  DEBUGASSERT(receiver && btle && msg);

  switch (receiver->state.header.msg_id)
    {
      case BTLE_DATA_PROFILE:
        {
          struct bluetooth_event_profile_s e;
#ifdef BTLE_DEBUG
          dbg("BTLE_DATA_PROFILE\n");
#endif

          /* publish event */

          e.event.id = BT_EVENT_DEVICE_PROFILE;
          e.size = receiver->state.header.length;
          e.profile = receiver->state.msg->payload;
          btle_publish_event(btle, (struct bluetooth_event_s *)&e);

          break;
        }

        case BTLE_DATA_CLOUD:
        {
          struct bluetooth_event_profile_s e;
#ifdef BTLE_DEBUG
          dbg("BTLE_DATA_CLOUD\n");
#endif

          /* publish event */

          e.event.id = BT_EVENT_CLOUD_PROFILE;
          e.size = receiver->state.header.length;
          e.profile = receiver->state.msg->payload;
          btle_publish_event(btle, (struct bluetooth_event_s *)&e);

          break;
        }

        case BTLE_DATA_TIME:
#ifdef BTLE_DEBUG
          dbg("BTLE_DATA_TIME\n");
#endif
          break;

        default:
          dbg("unknown msg_id\n");
          return ERROR;
    }

    if( btle_send_response(btle, msg->seq_num, BTLE_ACK_OK) != 0)
      {
        dbg("Response send failed\n");
      }

  return OK;
}

/****************************************************************************
 * Name: btle_receive_ind
 *
 * Description:
 *   Receive BTLE indication data
 *
 * Input Parameters:
 *   btle        - Pointer to BTLE structure
 *   data        - Received data
 *
 * Returned Values:
 *   0 (OK) means the data was accepted by receiver
 *  -1 (ERROR) means the data was ignored by receiver and can be sent to next receiver
 *
 ****************************************************************************/
static int btle_receive_ind(struct btle_s * const btle, const uint8_t data)
{
  struct btle_ind_receiver_s * const receiver = &btle->ind_receiver;

  switch (receiver->state)
    {
      case BTLE_IND_STATE_WAIT:
          if(data > BTLE_IND_EVENT_MAX)
            break;

          receiver->state = BTLE_IND_STATE_CONN_STATE;

      case BTLE_IND_STATE_CONN_STATE:
          receiver->msg.conn_state = data;
          receiver->state = BTLE_IND_STATE_GAP_STATE;
          break;

      case BTLE_IND_STATE_GAP_STATE:
        {
          struct bluetooth_event_conn_state_s e;
          receiver->msg.gap_state = data;
          receiver->state = BTLE_IND_STATE_WAIT;
#ifdef BTLE_DEBUG
          dbg("state: %s\n", btle_get_conn_state_name(data));
#endif

          /* BT state transfer from connected to either init or started,
             this indicates that chip has restarted */

          if (btle->state == BTLE_STATE_CONNECTED &&
             (receiver->msg.gap_state == BTLE_STATE_INIT ||
              receiver->msg.gap_state == BTLE_STATE_STARTED))
            {
              dbg("connection dropped, reset receiver\n");

              if (btle_receiver_reset(btle))
              {
                dbg("receiver reset failed!\n");
              }
            }

          btle->state = receiver->msg.gap_state;

          /* publish event */

          e.event.id = BT_EVENT_CONNECTION_STATE;
          e.state = receiver->msg.gap_state;
          btle_publish_event(btle, (struct bluetooth_event_s *)&e);

          break;
        }

      default:
          break;
    }

  return OK;
}

/****************************************************************************
 * Name: btle_receive_hci
 *
 * Description:
 *   Receive BTLE HCI data
 *
 * Input Parameters:
 *   btle        - Pointer to BTLE structure
 *   data        - Received data
 *
 * Returned Values:
 *   0 (OK) means the data was accepted by receiver
 *  -1 (ERROR) means the data was ignored by receiver and can be sent to next receiver
 *
 ****************************************************************************/
static int btle_receive_hci(struct btle_s * const btle, const uint8_t data)
{
  struct btle_data_receiver_s * const receiver = &btle->hci_receiver;

  /* TODO */

  if(!receiver->len)
    return ERROR;

  return OK;
}

/****************************************************************************
 * Name: btle_receive_data
 *
 * Description:
 *   Receive BTLE data
 *
 * Input Parameters:
 *   btle        - Pointer to BTLE structure
 *   data        - Received data
 *
 * Returned Values:
 *   0 (OK) means the data was accepted by receiver
 *  -1 (ERROR) means the data was ignored by receiver and can be sent to next receiver
 *
 ****************************************************************************/

static int btle_receive_data(struct btle_s * const btle, const uint8_t data)
{
  bool invalid;
  struct btle_data_receiver_s * const receiver = &btle->data_receiver;

  if (!receiver)
    return ERROR;

try_again:

  invalid = false;

  if (receiver->state.state >= BTLE_STATE_MSG_HEADER &&
      receiver->state.state <= BTLE_STATE_MSG_PAYLOAD)
    {
      /* Update checksum (8-Bit Fletcher Algorithm) */

      receiver->state.ck_a += data;
      receiver->state.ck_b += receiver->state.ck_a;
    }

  switch (receiver->state.state)
    {
      case BTLE_STATE_MSG_WAIT:
        {
          if (data == BTLE_SYNC_CHAR)
            {
#ifdef BTLE_DEBUG
              dbg("BTLE sync character received: 0x%02x\n", data);
#endif

              receiver->state.state = BTLE_STATE_MSG_HEADER;
              receiver->state.offset = 0;

              /* Start BTLE message receive timeout */
              receiver->receive_timer_id = bluetooth_set_timer(btle,
                                                               BTLE_MSG_RECV_TIMEOUT,
                                                               btle_transfer_timeout,
                                                               btle);

              return OK;
            }

          return ERROR;
        }

      case BTLE_STATE_MSG_HEADER:
        {
          uint8_t *rbuf = (uint8_t*)&receiver->state.header;

          rbuf[receiver->state.offset++] = data;
          if (receiver->state.offset < sizeof(struct btle_msg_s))
            {
              return OK;
            }

#ifdef BTLE_DEBUG
          dbg("Message header: msg_id:0x%02x, seq_num:0x%02x, length:%d\n",
              receiver->state.header.msg_id,
              receiver->state.header.seq_num,
              receiver->state.header.length);
#endif

          if (receiver->state.header.length > MAX_PAYLOAD_LENGTH)
            {
              /* Break out and reset BT LE receiver */

              break;
            }

          receiver->state.msg = malloc(sizeof(struct btle_msg_s) +
            receiver->state.header.length);

          if (!receiver->state.msg)
            {
              dbg("Unable to allocate memory for BTLE message, size: %d\n",
                  receiver->state.header.length);

              /* Break out and reset BT LE receiver */

              break;
            }

          /* Copy header to BTLE message */

          memcpy(receiver->state.msg, &receiver->state.header, sizeof(struct btle_msg_s));

          if (receiver->state.header.length > 0)
            {
              /* Reset receiver offset for payload data */

              receiver->state.offset = 0;

              /* Prepare to receive message payload */

              receiver->state.state = BTLE_STATE_MSG_PAYLOAD;
            }
          else
            {
              /* Prepare to receive checksum */

              receiver->state.state = BTLE_STATE_MSG_CHECKSUM1;
            }

          /* Retrieve next byte */

          return OK;
        }

      case BTLE_STATE_MSG_PAYLOAD:
        {
#ifdef BTLE_DEBUG
          dbg("PAYLOAD: 0x%02X, offset:%d/%d\n",
              data, receiver->state.offset, receiver->state.header.length);
#endif
          receiver->state.msg->payload[receiver->state.offset++] = data;

          /* Restart timer every x bytes received */

          if (receiver->state.offset % BTLE_RECV_TIMER_RESTART == 0)
            {
              (void)bluetooth_reset_timer_timeout(btle, receiver->receive_timer_id,
                BTLE_MSG_RECV_TIMEOUT);
            }

          if (receiver->state.offset >= receiver->state.msg->length)
            {
              /* Prepare to receive message checksum */

              receiver->state.state = BTLE_STATE_MSG_CHECKSUM1;
            }

          /* Retrieve next byte */

          return OK;
        }

      case BTLE_STATE_MSG_CHECKSUM1:
        {
          if (receiver->state.ck_a != data)
            {
              dbg("Invalid CK_A checksum (got 0x%02x, expected 0x%02x).\n",
                  data, receiver->state.ck_a);

              invalid = true;

              /* Break out and reset BTLE receiver */

              break;
            }

          /* Receive second checksum byte */

          receiver->state.state = BTLE_STATE_MSG_CHECKSUM2;

          /* Retrieve next byte */

          return OK;
        }

      case BTLE_STATE_MSG_CHECKSUM2:
        {
          if (receiver->state.ck_b != data)
            {
              dbg("Invalid CK_B checksum (got %02x, expected %02x).\n",
                  data, receiver->state.ck_b);

              invalid = true;

              /* Break out and reset BTLE receiver */

              break;
            }

#ifdef BTLE_DEBUG
          dbg("BTLE message (properly) received. " \
              "msg_id:0x%02x, seq_num:0x%02x, payload length:%d\n",
              receiver->state.header.msg_id,
              receiver->state.header.seq_num,
              receiver->state.header.length);
#endif
          (void)btle_data_received(receiver, btle, receiver->state.msg);

          /* Break out and reset BTLE receiver */

          break;
        }
    }

  /* Stop receive timeout timer */

  if (receiver->receive_timer_id >= 0)
    {
      bluetooth_remove_timer(btle, receiver->receive_timer_id);

      /* Reset timer ID */

      receiver->receive_timer_id = -1;
    }

  /* Free pending BTLE message */

  if (receiver->state.state >= BTLE_STATE_MSG_PAYLOAD)
    {
      /* Free allocated message buffer */

      free(receiver->state.msg);
    }

  /* Reset BTLE receiver state */

  receiver->state.state = BTLE_STATE_MSG_WAIT;
  receiver->state.ck_a = 0;
  receiver->state.ck_b = 0;

  if (invalid)
    goto try_again;

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: btle_receiver_initialize
 *
 * Description:
 *   Initialize BTLE receiver
 *
 * Input Parameters:
 *   btle        - Pointer to BTLE structure
 *   priv        - Pointer to callback private data
 *
 * Returned Values:
 *   0 (OK) means the function was executed successfully
 *  -1 (ERROR) means the function was executed unsuccessfully.
 *
 ****************************************************************************/

int btle_receiver_initialize(struct btle_s * const btle, void * const priv)
{
  struct btle_data_receiver_s * const data_receiver = &btle->data_receiver;
  struct btle_ind_receiver_s * const ind_receiver = &btle->ind_receiver;
  struct btle_data_receiver_s * const hci_receiver = &btle->hci_receiver;

  /* reset receive handler */

  btle->rx_handler = NULL;
  btle->payload_len = 0;
  btle->state = BTLE_STATE_INIT;

  /* Initialize data receiver */

  if (!data_receiver)
    return ERROR;

  data_receiver->state.state = BTLE_STATE_MSG_WAIT;
  data_receiver->receive_timer_id = -1;
  data_receiver->priv = priv;

  /* Clear pending BT LE message */

  memset(&data_receiver->state.header, 0, sizeof(data_receiver->state.header));

  /* initialize indication receiver */

  if (!ind_receiver)
    return ERROR;

  ind_receiver->state = BTLE_IND_STATE_WAIT;
  memset(&ind_receiver->msg, 0, sizeof(ind_receiver->msg));

  /* initialize HCI receiver */

  if (!hci_receiver)
    return ERROR;

  hci_receiver->state.state = 0;
  hci_receiver->receive_timer_id = -1;
  hci_receiver->priv = priv;

  return OK;
}

/****************************************************************************
 * Name: btle_setup_rx_handler
 *
 * Description:
 *
 *
 * Input Parameters:
 *   btle         - Pointer to BT LE structure
 *   buffer       -
 *   len          -
 *
 * Returned Values:
 *   0 (OK) means the function was executed successfully
 *  -1 (ERROR) means the function was executed unsuccessfully.
 *
 ****************************************************************************/

int btle_setup_rx_handler(struct btle_s * const btle,
                          uint8_t** buffer,
                          ssize_t* len)
{
  struct btle_frame_s * frame = (struct btle_frame_s*)*buffer;
  int ret = OK;
  btle->rx_handler = NULL;

#ifdef BTLE_DEBUG
  dbg("msg_type:0x%02X, len:%d\n", frame->msg_type, frame->payload_len);
#endif

  switch(frame->msg_type)
    {
      case BTLE_MSG_DATA:
        if (frame->payload_len != DATA_MSG_LENGTH)
          {
             dbg("BTLE_MSG_DATA - invalid length %d!!!\n",
                 frame->payload_len);

             return ERROR;
          }

        btle->rx_handler = btle_receive_data;
        btle->data_receiver.len = btle->payload_len;
        break;

      case BTLE_MSG_HCI_EVENT:
        btle->rx_handler = btle_receive_hci;
        btle->hci_receiver.len = btle->payload_len;
        break;

      case BTLE_MSG_INDICATION:
        if (frame->payload_len != IND_MSG_LENGTH)
          {
             dbg("BTLE_MSG_INDICATION - invalid length %d!!!\n",
                 frame->payload_len);

             return ERROR;
          }

        btle->rx_handler = btle_receive_ind;
        btle->ind_receiver.len = btle->payload_len;
        break;

      default:
        btle->rx_handler = NULL;
        ret = ERROR;
        break;
    }

  btle->payload_len = frame->payload_len;

  /* Adjust buffer pointer and input length accordingly */

  *buffer = (uint8_t*)&frame->payload;
  *len -= 2;

  return ret;
}

/****************************************************************************
 * Name: btle_receiver_reset
 *
 * Description:
 *   Reset BT LE receiver send / receive timeouts and free allocated memory
 *
 * Input Parameters:
 *   btle        - Pointer to BT LE structure
 *
 * Returned Values:
 *   0 (OK) means the function was executed successfully
 *  -1 (ERROR) means the function was executed unsuccessfully. Check value of
 *  errno for more details.
 *
 ****************************************************************************/

int btle_receiver_reset(struct btle_s * const btle)
{
  struct btle_data_receiver_s * const receiver = &btle->data_receiver;

  if (!receiver)
    return ERROR;

  /* Reset receive handler */

  btle->rx_handler = NULL;

  /* Reset receive timeout timer */

  if (receiver->receive_timer_id >= 0)
    {
      bluetooth_remove_timer(btle, receiver->receive_timer_id);

      /* Reset timer ID */

      receiver->receive_timer_id = -1;
    }

  /* Free pending BTLE message */

  if (receiver->state.state >= BTLE_STATE_MSG_PAYLOAD)
    {
      /* Free allocated message buffer */

      free(receiver->state.msg);
    }

  /* Reset BTLE receiver state */

  receiver->state.state = BTLE_STATE_MSG_WAIT;
  receiver->state.ck_a = 0;
  receiver->state.ck_b = 0;

  return OK;
}
