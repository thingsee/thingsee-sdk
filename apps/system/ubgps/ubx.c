/****************************************************************************
 * apps/system/ubgps/ubx.c
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
 *   Author: Sami Pelkonen <sami.pelkonen@haltian.com>
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include "ubgps_internal.h"
#include "ubx.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef UBX_DEBUG
 #define dbg_ubx(...) dbg(__VA_ARGS__)
#else
  #define dbg_ubx(...)
#endif

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
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ubx_timeout
 *
 * Description:
 *   UBX protocol timeout handling
 *
 ****************************************************************************/
static int ubx_timeout(const int timer_id, void * const priv)
{
  struct ubx_receiver_s * const receiver = (struct ubx_receiver_s *)priv;

  /* Was this a UBX message send timeout */

  if (timer_id == receiver->send_timer_id)
    {
      dbg_ubx("UBX message ack timeout. Class:0x%02x, msg:0x%02x\n",
        receiver->state.waiting_ack.class_id, receiver->state.waiting_ack.msg_id);

      /* Reset timer ID */

      receiver->send_timer_id = -1;

      /* Invoke UBX callback */

      if (receiver->callback)
        {
          (void)receiver->callback(receiver,
                                   receiver->priv,
                                   &receiver->state.waiting_ack,
                                   true);
        }
    }

  /* Was this a UBX message receive timeout */

  else if (timer_id == receiver->receive_timer_id)
    {
      dbg_ubx("GPS UBX receiver reset due to timeout, state:%u\n",
          receiver->state.state);

      /* Reset timer ID */

      receiver->receive_timer_id = -1;

      /* Free pending UBX message */

      if (receiver->state.state >= UBX_STATE_MSG_PAYLOAD)
        {
          /* Free allocated message buffer */

          free(receiver->state.msg);
        }

      /* Reset UBX receiver state */

      receiver->state.state = UBX_STATE_MSG_WAIT;
      receiver->state.ck_a = 0;
      receiver->state.ck_b = 0;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ubx_initialize
 *
 * Description:
 *   Initialize UBX receiver
 *
 * Input Parameters:
 *   gps         - Pointer to GPS structure
 *   callback    - Pointer to callback
 *   priv        - Pointer to callback private data
 *
 * Returned Values:
 *   0 (OK) means the function was executed successfully
 *  -1 (ERROR) means the function was executed unsuccessfully.
 *
 ****************************************************************************/
int ubx_initialize(struct ubgps_s * const gps, ubx_callback_t callback, void * const priv)
{
  struct ubx_receiver_s * const receiver = &gps->state.ubx_receiver;

  if (!receiver)
    return ERROR;

  /* Initialize UBX receiver */

  receiver->state.state = UBX_STATE_MSG_WAIT;
  receiver->send_timer_id = -1;
  receiver->receive_timer_id = -1;
  receiver->callback = callback;
  receiver->priv = priv;

  /* Clear pending UBX message */

  memset(&receiver->state.header, 0, sizeof(receiver->state.header));

  return OK;
}

/****************************************************************************
 * Name: ubx_deinitialize
 *
 * Description:
 *   Deinitialize UBX receiver
 *
 * Input Parameters:
 *   gps         - Pointer to GPS structure
 *
 * Returned Values:
 *   0 (OK) means the function was executed successfully
 *  -1 (ERROR) means the function was executed unsuccessfully.
 *
 ****************************************************************************/
int ubx_deinitialize(struct ubgps_s * const gps)
{
  struct ubx_receiver_s * const receiver = &gps->state.ubx_receiver;

  if (!receiver)
    return ERROR;

  return ubx_reset(gps);
}

/****************************************************************************
 * Name: ubx_msg_allocate
 *
 * Description:
 *   Allocate buffer for UBX message
 *
 * Input Parameters:
 *   class_id       - UBX class ID
 *   msg_id         - UBX message ID
 *   payload_length - UBX message payload length
 *
 * Returned Values:
 *   Pointer to allocated message
 *   NULL means the function was executed unsuccessfully.
 *
 ****************************************************************************/
struct ubx_msg_s * ubx_msg_allocate(const uint8_t class_id, const uint8_t msg_id, const size_t payload_length)
{
  struct ubx_msg_s * msg;

  /* Allocate message */

  msg = malloc(sizeof(struct ubx_msg_s) + payload_length);
  if (!msg)
    return NULL;

  msg->class_id = class_id;
  msg->msg_id = msg_id;
  msg->length = payload_length;

  if (payload_length)
    memset(&msg->payload, 0, payload_length);

  return msg;
}

/****************************************************************************
 * Name: ubx_msg_send
 *
 * Description:
 *   Send UBX message
 *
 * Input Parameters:
 *   gps         - Pointer to GPS structure
 *   fd          - File descriptor to write message
 *   msg         - Pointer to UBX message
 *
 * Returned Values:
 *   0 (OK) means the function was executed successfully
 *  -1 (ERROR) means the function was executed unsuccessfully. Check value of
 *  errno for more details.
 *
 ****************************************************************************/
int ubx_msg_send(struct ubgps_s * const gps, const int fd, struct ubx_msg_s const * const msg)
{
  struct ubx_receiver_s * const receiver = &gps->state.ubx_receiver;
  uint8_t header[2] = { UBX_SYNC_CHAR_1, UBX_SYNC_CHAR_2 };
  uint8_t checksum[2];
  uint8_t ck_a = 0;
  uint8_t ck_b = 0;
  uint8_t * data;
  size_t msg_len;
  int ret;
#ifdef UBX_DEBUG
  uint16_t i;
#endif

  /* Input data validation */

  if (!receiver || fd < 0 || !msg)
    {
      set_errno(EINVAL);

      return ERROR;
    }

  /* Check if there is already a message waiting acknowledgment */

  if (receiver->send_timer_id >= 0)
    {
      set_errno(EBUSY);

      return ERROR;
    }

  dbg_ubx("Class:0x%02x, msg:0x%02x, length:%d\n",
    msg->class_id, msg->msg_id, msg->length);

  /* Send UBX sync characters */

  ret = write(fd, header, sizeof(header));
  if (ret != sizeof(header))
    return ERROR;

  msg_len = sizeof(struct ubx_msg_s) + msg->length;

#ifdef UBX_DEBUG
  printf("payload:");
  for(i=0; i < msg->length; i++)
    {
      if (i%10 == 0) printf("\n");
      printf("%02X", msg->payload[i]);
    }
  printf("\n");
#endif /* UBX_DEBUG */

  ret = write(fd, msg, msg_len);
  if (ret != msg_len)
    return ERROR;

  /* Calculate checksum (8-Bit Fletcher Algorithm) */

  data = (uint8_t *)msg;
  while (msg_len--)
    {
      ck_a += *data++;
      ck_b += ck_a;
    }

  /* Send checksum */

  checksum[0] = ck_a;
  checksum[1] = ck_b;

  ret = write(fd, checksum, sizeof(checksum));
  if (ret != sizeof(checksum))
    return ERROR;

  /*
   * Start acknowledgment timeout for CFG class messages. Acknowledgment
   * is also used by some LOG class messages, but this is not implemented.
   */

  if (msg->class_id == UBX_CLASS_CFG)
    {
      /* Setup acknowledgment waiting */

      memcpy(&receiver->state.waiting_ack, msg, sizeof(struct ubx_msg_s));

      /* Start UBX message acknowledgment timeout */

      receiver->send_timer_id = __ubgps_set_timer(gps,
                                                  UBX_MSG_ACK_TIMEOUT,
                                                  ubx_timeout,
                                                  receiver);
    }

  return OK;
}

/****************************************************************************
 * Name: ubx_msg_receive
 *
 * Description:
 *   Receive UBX messages
 *
 * Input Parameters:
 *   gps         - Pointer to GPS structure
 *   data        - Received data
 *
 * Returned Values:
 *   0 (OK) means the data was accepted by receiver
 *  -1 (ERROR) means the data was ignored by receiver and can be sent to next receiver
 *
 ****************************************************************************/
int ubx_msg_receive(struct ubgps_s * const gps, const uint8_t data)
{
  bool invalid;
  struct ubx_receiver_s * const receiver = &gps->state.ubx_receiver;
#ifdef UBX_DEBUG
  uint16_t i;
#endif

  if (!receiver)
    return ERROR;

try_again:

  invalid = false;

  if (receiver->state.state >= UBX_STATE_MSG_HEADER &&
      receiver->state.state <= UBX_STATE_MSG_PAYLOAD)
    {
      /* Update checksum (8-Bit Fletcher Algorithm) */

      receiver->state.ck_a += data;
      receiver->state.ck_b += receiver->state.ck_a;
    }

  switch (receiver->state.state)
    {
      case UBX_STATE_MSG_WAIT:
        {
          if (data == UBX_SYNC_CHAR_1)
            {

              dbg_ubx("UBX sync character received.\n");

              /* Prepare to receive second UBX sync character */

              receiver->state.state = UBX_STATE_MSG_SYNC;

              /* Start UBX message receive timeout */
              receiver->receive_timer_id = __ubgps_set_timer(gps,
                                                             UBX_MSG_RECV_TIMEOUT,
                                                             ubx_timeout,
                                                             receiver);

              /* Retrieve next byte */

              return OK;
            }

          /* Pass data to NMEA parser */

          return ERROR;
        }

      case UBX_STATE_MSG_SYNC:
        {
          if (data != UBX_SYNC_CHAR_2)
            {
              invalid = true;

              /* Break out and reset UBX receiver */

              break;
            }

          /* Reset receiver offset for header data */

          receiver->state.offset = 0;

          /* Prepare to receive UBX header (class id, message id and length) */

          receiver->state.state = UBX_STATE_MSG_HEADER;

          /* Retrieve next byte */

          return OK;
        }

      case UBX_STATE_MSG_HEADER:
        {
          uint8_t *rbuf = (uint8_t *)&receiver->state.header;

          rbuf[receiver->state.offset++] = data;
          if (receiver->state.offset < sizeof(struct ubx_msg_s))
            {
              /* Retrieve next byte */

              return OK;
            }

          dbg_ubx("Class:0x%02x, msg:0x%02x, length:%d\n",
              receiver->state.header.class_id,
              receiver->state.header.msg_id,
              receiver->state.header.length);

          /* Limit UBX message length */

          if (receiver->state.header.length > UBX_MSG_MAX_PAYLOAD_LENGTH)
            {
              dbg_ubx("Ignoring too long GPS UBX message.\n");

              invalid = true;

              /* Break out and reset UBX receiver */

              break;
            }

          /* Allocate memory for UBX message */

          receiver->state.msg = malloc(sizeof(struct ubx_msg_s) + receiver->state.header.length);
          if (!receiver->state.msg)
            {
              dbg_ubx("Unable to allocate memory for GPS UBX message. " \
                  "Class:0x%02x, msg:0x%02x. Payload:%d\n",
                  receiver->state.header.class_id,
                  receiver->state.header.msg_id,
                  receiver->state.header.length);

              /* Break out and reset UBX receiver */

              break;
            }

          /* Copy header to UBX message */

          memcpy(receiver->state.msg, &receiver->state.header, sizeof(struct ubx_msg_s));

          if (receiver->state.header.length > 0)
            {
              /* Reset receiver offset for payload data */

              receiver->state.offset = 0;

              /* Prepare to receive message payload */

              receiver->state.state = UBX_STATE_MSG_PAYLOAD;
            }
          else
            {
              /* Prepare to receive checksum */

              receiver->state.state = UBX_STATE_MSG_CHECKSUM1;
            }

          /* Retrieve next byte */

          return OK;
        }

      case UBX_STATE_MSG_PAYLOAD:
        {
          receiver->state.msg->payload[receiver->state.offset++] = data;

          if (receiver->state.offset >= receiver->state.msg->length)
            {
              /* Prepare to receive message checksum */

              receiver->state.state = UBX_STATE_MSG_CHECKSUM1;
            }

          /* Retrieve next byte */

          return OK;
        }

      case UBX_STATE_MSG_CHECKSUM1:
        {
          if (receiver->state.ck_a != data)
            {
              dbg_ubx("Invalid CK_A checksum (got %02x, expected %02x).\n",
                  data, receiver->state.ck_a);

              invalid = true;

              /* Break out and reset UBX receiver */

              break;
            }

          /* Receive second checksum byte */

          receiver->state.state = UBX_STATE_MSG_CHECKSUM2;

          /* Retrieve next byte */

          return OK;
        }

      case UBX_STATE_MSG_CHECKSUM2:
        {
          if (receiver->state.ck_b != data)
            {
              dbg_ubx("Invalid CK_B checksum (got %02x, expected %02x).\n",
                  data, receiver->state.ck_b);

              invalid = true;

              /* Break out and reset UBX receiver */

              break;
            }

          /* Check if we are waiting for UBX message acknowledgment */

          if (receiver->send_timer_id >= 0)
            {
              /* Check that message is ACK-ACK or ACK-NAK */

              if (receiver->state.msg->length == UBX_ACK_ACK_LEN &&
                  receiver->state.msg->class_id == UBX_CLASS_ACK &&
                  (receiver->state.msg->msg_id == UBX_ACK_ACK ||
                   receiver->state.msg->msg_id == UBX_ACK_NAK))
                {
                  uint8_t class_id = UBX_GET_U1(receiver->state.msg, 0);
                  uint8_t msg_id = UBX_GET_U1(receiver->state.msg, 1);

                  /* Check if ACK-ACK or ACK-NAK is for correct message */

                  if (class_id == receiver->state.waiting_ack.class_id &&
                      msg_id == receiver->state.waiting_ack.msg_id)
                    {
                      /* Stop send timeout timer */
                      __ubgps_remove_timer(gps, receiver->send_timer_id);

                      /* Reset timer ID */

                      receiver->send_timer_id = -1;
                    }
                }
            }

#ifdef UBX_DEBUG
          printf("payload:");
          for(i=0; i<receiver->state.msg->length; i++)
            {
              if (i%10 == 0) printf("\n");
              printf("%02X", receiver->state.msg->payload[i]);
            }
          printf("\n");
#endif /* UBX_DEBUG */

          /* Deliver UBX message to callback */

          receiver->callback(receiver,
                             receiver->priv,
                             receiver->state.msg,
                             false);

          /* Break out and reset UBX receiver */

          break;
        }
    }

  /* Stop receive timeout timer */

  if (receiver->receive_timer_id >= 0)
    {
      __ubgps_remove_timer(gps, receiver->receive_timer_id);

      /* Reset timer ID */

      receiver->receive_timer_id = -1;
    }

  /* Free pending UBX message */

  if (receiver->state.state >= UBX_STATE_MSG_PAYLOAD)
    {
      /* Free allocated message buffer */

      free(receiver->state.msg);
    }

  /* Reset UBX receiver state */

  receiver->state.state = UBX_STATE_MSG_WAIT;
  receiver->state.ck_a = 0;
  receiver->state.ck_b = 0;

  if (invalid)
    goto try_again;

  return OK;
}

/****************************************************************************
 * Name: ubx_reset
 *
 * Description:
 *   Reset UBX receiver send / receive timeouts and free allocated memory
 *
 * Input Parameters:
 *   gps        - Pointer to GPS structure
 *
 * Returned Values:
 *   0 (OK) means the function was executed successfully
 *  -1 (ERROR) means the function was executed unsuccessfully. Check value of
 *  errno for more details.
 *
 ****************************************************************************/
int ubx_reset(struct ubgps_s * const gps)
{
  struct ubx_receiver_s * const receiver = &gps->state.ubx_receiver;

  if (!receiver)
    return ERROR;

  /* Reset send timeout timer */

  if (receiver->send_timer_id >= 0)
    {
      __ubgps_remove_timer(gps, receiver->send_timer_id);

      /* Reset timer ID */

      receiver->send_timer_id = -1;
    }

  /* Reset receive timeout timer */

  if (receiver->receive_timer_id >= 0)
    {
      __ubgps_remove_timer(gps, receiver->receive_timer_id);

      /* Reset timer ID */

      receiver->receive_timer_id = -1;
    }

  /* Free pending UBX message */

  if (receiver->state.state >= UBX_STATE_MSG_PAYLOAD)
    {
      /* Free allocated message buffer */

      free(receiver->state.msg);
    }

  /* Reset UBX receiver state */

  receiver->state.state = UBX_STATE_MSG_WAIT;
  receiver->state.ck_a = 0;
  receiver->state.ck_b = 0;

  return OK;
}

/****************************************************************************
 * Name: ubx_busy
 *
 * Description:
 *   Check whether UBX receiver is busy
 *
 * Input Parameters:
 *   receiver    - Pointer to UBX receiver structure
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
bool ubx_busy(struct ubx_receiver_s * const receiver)
{
  if (!receiver)
    return false;

  if (receiver->send_timer_id >= 0)
    return true;

  return false;
}
