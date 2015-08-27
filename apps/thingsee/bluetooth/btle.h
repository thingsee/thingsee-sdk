/****************************************************************************
 * thingsee/bluetooth/btle.h
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

#ifndef BTLE_H
#define BTLE_H

/* Data receive timeout [ms] */

#define BTLE_MSG_RECV_TIMEOUT       4000

/* Receiver byte count after receiver timer is restarted */

#define BTLE_RECV_TIMER_RESTART     20

/* Frame start indicator */

#define BTLE_SYNC_CHAR              0xB5

/* Payload length for data packets */

#define DATA_MSG_LENGTH             20

/* Payload length for indication packets */

#define IND_MSG_LENGTH              2

/* Maximum total size for BT LE data frame */

#define MAX_PAYLOAD_LENGTH          4096

/* BTLE data receiver state */

enum btle_data_state_e
{
  BTLE_STATE_MSG_WAIT = 0,  /* Waiting for BTLE data frame */
  BTLE_STATE_MSG_HEADER,    /* Header (msg_id, length) */
  BTLE_STATE_MSG_PAYLOAD,   /* BTLE message payload */
  BTLE_STATE_MSG_CHECKSUM1,
  BTLE_STATE_MSG_CHECKSUM2,
};
typedef enum btle_data_state_e btle_data_state_t;

enum btle_ind_state_e
{
  BTLE_IND_STATE_WAIT = 0,
  BTLE_IND_STATE_CONN_STATE,
  BTLE_IND_STATE_GAP_STATE,
  BTLE_IND_STATE_MAX
};
typedef enum btle_ind_state_e btle_ind_state_t;

enum btle_ind_event_e
{
  BTLE_IND_EVENT_CONNECTED = 0,
  BTLE_IND_EVENT_DISCONNECT,
  BTLE_IND_EVENT_MAX
};

enum btle_data_type_e
{
  BTLE_DATA_PROFILE = 1,
  BTLE_DATA_CLOUD,
  BTLE_DATA_TIME,
  BTLE_DATA_ACK = 0x80,
};

enum btle_frame_type_e
{
  BTLE_MSG_DATA = 1,
  BTLE_MSG_HCI_EVENT,
  BTLE_MSG_INDICATION,
};

enum btle_ack_e
{
  BTLE_ACK_TIMEOUT = -127,
  BTLE_ACK_NO_MEM,
  BTLE_ACK_CHECKSUM_FAIL,
  BTLE_ACK_OK = 0,
};
typedef enum btle_ack_e btle_ack_t;

struct btle_frame_s
{
  uint8_t msg_type;
  uint8_t payload_len;
  uint8_t payload;
} packed_struct;

struct btle_frame;

struct btle_msg_s
{
  uint8_t msg_id;
  uint8_t seq_num;
  uint16_t length;
  uint8_t payload[0];
} packed_struct;

struct btle_ack_msg_s
{
  uint8_t type;
  uint8_t packet_len;
  uint8_t msg_id;
  uint8_t seq_num;
  uint16_t length;
  uint8_t reason;
  uint8_t ck_a;
  uint8_t ck_b;
} packed_struct;

struct btle_ind_s
{
  uint8_t conn_state;
  uint8_t gap_state;
} packed_struct;

struct btle_data_receiver_s;

struct btle_s;

/* Current rx data handler function */

typedef int (* btle_handler_fn_t)(struct btle_s * const, const uint8_t data);

/* BTLE receiver state */

struct btle_receiver_state_s
{
  /* BTLE receiver state */

  btle_data_state_t state;

  /* Received message header */

  struct btle_msg_s header;

  /* BTLE message */

  struct btle_msg_s * msg;

  /* Current receiving offset */

  size_t offset;

  /* Checksum */

  uint8_t ck_a;
  uint8_t ck_b;
};

/* Indication receiver */

struct btle_ind_receiver_s
{
  btle_ind_state_t state;

  /* data length */

  uint16_t len;

  struct btle_ind_s msg;
};

/* BTLE state */

struct btle_data_receiver_s
{
  /* BTLE receiver state */

  struct btle_receiver_state_s state;

  /* data length */

  uint16_t len;

  /* BTLE receive timeout timer ID */

  int receive_timer_id;

  /* BTLE callback private data */

  void * priv;
};

/* BTLE module structure */

struct btle_s {
  /* BT serial device file descriptor */

  int fd;

  /* BT serial device is non-blocking */

  bool is_nonblocking;

  /* Uninit after pending I/O is handled */

  bool uninit_pending;

  /* BT connection state */

  bluetooth_state_t state;

  /* Timers */

  sq_queue_t timers;

  /* Timer count */

  uint16_t timer_id_cnt;

  /* Mask of subscribed events for callbacks */

  uint32_t callback_event_mask;

  /* Registered BT callbacks */

  sq_queue_t callbacks;

  /* BT rx handler function */

  btle_handler_fn_t rx_handler;

  /* Bytes to be handled with current rx handler */

  uint8_t payload_len;

  /* BT data receiver */

  struct btle_data_receiver_s data_receiver;

  /* BT indication receiver */

  struct btle_ind_receiver_s ind_receiver;

  /* BT HCI receiver */

  struct btle_data_receiver_s hci_receiver;
};

/* Timer callback function type. */

typedef int (* btle_timer_fn_t)(int timer_id, void * const arg);


/* Bluetooth callback */

typedef void (*bluetooth_callback_t)(void const * const e,
                                     void * const priv);

struct bluetooth_callback_entry_s {
  /* Queue entry */

  sq_entry_t entry;

  /* Subscribed events */

  uint32_t event_mask;

  /* BT callback function */

  bluetooth_callback_t callback;

  /* Private data */

  void * priv;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: btle_receiver
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
int bluetooth_receiver(const struct pollfd * const pfd, void * const priv);

/****************************************************************************
 * Name: bluetooth_set_timer
 *
 * Description:
 *   Setup timer with callback.
 *
 ****************************************************************************/

int bluetooth_set_timer(struct btle_s *btle,
                        unsigned int timeout_msec, btle_timer_fn_t timer_cb,
                        void *cb_priv);

/****************************************************************************
 * Name: bluetooth_reset_timer_timeout
 *
 * Description:
 *   Reset existing timer timeout.
 *
 ****************************************************************************/

int bluetooth_reset_timer_timeout(struct btle_s *btle,
                                  uint16_t id,
                                  unsigned int timeout_msec);

/****************************************************************************
 * Name: bluetooth_remove_timer
 *
 * Description:
 *   Remove timer.
 *
 ****************************************************************************/

void bluetooth_remove_timer(struct btle_s * const btle, uint16_t id);

/****************************************************************************
 * Name: bluetooth_initialize
 *
 * Description:
 *   Initialize BT module
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/

struct btle_s *bluetooth_initialize(void);

/****************************************************************************
 * Name: bluetooth_uninitialize
 *
 * Description:
 *   Deinitialize BT module
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/

void bluetooth_uninitialize(struct btle_s * const btle);

/****************************************************************************
 * Name: bluetooth_setup_poll
 *
 * Description:
 *   Setup pollfd structure and timeout value for bluetooth
 *
 * Input Parameters:
 *   None.
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/

int bluetooth_poll_setup(struct btle_s *btle, struct pollfd *pfd,
                         int *timeout);

/****************************************************************************
 * Name: bluetooth_poll_event
 *
 * Description:
 *   Indicate poll event for BTLE library. Library will handle current
 *   pending I/O and internal state changes.
 *
 * Input Parameters:
 *   pfd: Poll structure for btle, setup with btle_poll_setup
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/

int bluetooth_poll_event(struct btle_s *btle, struct pollfd *pfd);

/****************************************************************************
 * Name: bluetooth_poll_timedout
 *
 * Description:
 *   Indicate that poll timed out. This allows library to handle internal
 *   timed state changes.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/

int bluetooth_poll_timedout(struct btle_s *btle);

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
                                void * const priv);


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

int bluetooth_callback_unregister(bluetooth_callback_t callback);

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

bluetooth_state_t bluetooth_get_state(void);

/****************************************************************************
 * Name: bluetooth_get_conn_state_name
 *
 * Description:
 *
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/

char const * const bluetooth_get_conn_state_name(bluetooth_state_t state);

#endif /* BTLE_H */
