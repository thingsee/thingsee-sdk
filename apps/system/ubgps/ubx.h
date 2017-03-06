/****************************************************************************
 * apps/system/ubgps/ubx.h
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

#ifndef __THINGSEE_GPS_UBX_H
#define __THINGSEE_GPS_UBX_H

#include <stdbool.h>
#include <stdlib.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Public Defines
 ****************************************************************************/

/* UBX protocol synchronization characters */

#define UBX_SYNC_CHAR_1               0xB5
#define UBX_SYNC_CHAR_2               0x62

/* UBX protocol class id's */

#define UBX_CLASS_NAV                 0x01  /* Navigation Results */
#define UBX_CLASS_RXM                 0x02  /* Receiver Manager Messages */
#define UBX_CLASS_INF                 0x04  /* Information Messages */
#define UBX_CLASS_ACK                 0x05  /* Ack/Nack Messages */
#define UBX_CLASS_CFG                 0x06  /* Configuration Input Messages */
#define UBX_CLASS_MON                 0x0A  /* Monitoring Messages */
#define UBX_CLASS_AID                 0x0B  /* AssistNow Aiding Messages */
#define UBX_CLASS_TIM                 0x0D  /* Timing Messages */
#define UBX_CLASS_LOG                 0x21  /* Logging Messages */


/* UBX ACK class (0x05) message id's */

#define UBX_ACK_ACK                   0x01  /* Message Acknowledged */
#define UBX_ACK_ACK_LEN               2
#define UBX_ACK_NAK                   0x00  /* Message Not-Acknowledged */
#define UBX_ACK_NAK_LEN               2


/* UBX CFG class (0x06) message id's */

/* CFG-MSG: Poll / set message rates - 0x01 */

#define UBX_CFG_MSG                   0x01
#define UBX_CFG_MSG_LEN               3
#define UBX_CFG_MSG_POLL              UBX_CFG_MSG
#define UBX_CFG_MSG_POLL_LEN          0

/* CFG-PRT: Port Configuration - 0x00 */

#define UBX_CFG_PRT                   0x00
#define UBX_CFG_PRT_LEN               20
#define UBX_CFG_PRT_POLL              UBX_CFG_PRT
#define UBX_CFG_PRT_POLL_LEN          0

/* CFG-RST: Reset Receiver - 0x04 */

#define UBX_CFG_RST                   0x04
#define UBX_CFG_RST_LEN               4

/* CFG-RATE: Navigation rate - 0x08 */

#define UBX_CFG_RATE                  0x08
#define UBX_CFG_RATE_LEN              6

/* CFG-CFG: Clear, save and load configurations - 0x09 */

#define UBX_CFG_CFG                   0x09
#define UBX_CFG_CFG_LEN               12

#define CFG_ACT_CLEAR                 1
#define CFG_ACT_SAVE                  2
#define CFG_ACT_LOAD                  3

#define CFG_IO_CONF                   (1 << 0)
#define CFG_MSG_CONF                  (1 << 1)
#define CFG_INF_CONF                  (1 << 2)
#define CFG_NAV_CONF                  (1 << 3)
#define CFG_RXM_CONF                  (1 << 4)
#define CFG_RINV_CONF                 (1 << 9)
#define CFG_ANT_CONF                  (1 << 10)
#define CFG_LOG_CONF                  (1 << 11)
#define CFG_FTS_CONF                  (1 << 12)

/* CFG-RXM: RXM configuration - 0x11 */

#define UBX_CFG_RXM                   0x11
#define UBX_CFG_RXM_LEN               2

#define RXM_CONTINOUS                 0
#define RXM_POWER_SAVE                1

/* CFG-ANT: Antenna control settings - 0x13 */

#define UBX_CFG_ANT                  0x13
#define UBX_CFG_ANT_LEN              4

#define ANT_FLAG_SVCS                (1 << 0)
#define ANT_FLAG_SCD                 (1 << 1)
#define ANT_FLAG_OCD                 (1 << 2)
#define ANT_FLAG_PDWN_ON_SCD         (1 << 3)
#define ANT_FLAG_RECOVERY            (1 << 4)

#define ANT_PIN_LNA_CTRL_SHIFT       0
#define ANT_PIN_SHORT_DET_SHIFT      5
#define ANT_PIN_OPEN_DET_SHIFT       10
#define ANT_PIN_RECONFIG             (1 << 15)

/* CFG-SBAS: SBAS subsystem configuration - 0x16 */

#define UBX_CFG_SBAS                  0x16
#define UBX_CFG_SBAS_LEN              8

#define SBAS_DISABLED                 0
#define SBAS_ENABLED                  1

#define SBAS_MODE_ENABLED             (1 << 0)
#define SBAS_MODE_TEST                (1 << 1)

#define SBAS_USAGE_RANGE              (1 << 0)
#define SBAS_USAGE_DIFFCORR           (1 << 1)
#define SBAS_USAGE_INTEGRITY          (1 << 2)

#define SBAS_SCAN_MODE_AUTO           0

/* CFG-NAV5: Navigation engine settings - 0x24 */

#define UBX_CFG_NAV5                  0x24
#define UBX_CFG_NAV5_LEN              36

#define NAV5_MASK_MODEL               (1 << 0)
#define NAV5_MASK_ELEV                (1 << 1)
#define NAV5_MASK_FIX_MODE            (1 << 2)
#define NAV5_MASK_POS_MASK            (1 << 4)
#define NAV5_MASK_TIME_MASK           (1 << 5)
#define NAV5_MASK_STATIC_HOLD         (1 << 6)
#define NAV5_MASK_DGPS                (1 << 7)
#define NAV5_MASK_CNO                 (1 << 8)
#define NAV5_MASK_UTC                 (1 << 10)

#define NAV5_MODEL_PORTABLE           0
#define NAV5_MODEL_STATIONARY         2
#define NAV5_MODEL_PEDESTRIAN         3
#define NAV5_MODEL_AUTOMOTIVE         4
#define NAV5_MODEL_SEA                5
#define NAV5_MODEL_AIRBORNE_1G        6
#define NAV5_MODEL_AIRBORNE_2G        7
#define NAV5_MODEL_AIRBORNE_4G        8

#define NAV5_FIXMODE_2D_ONLY          1
#define NAV5_FIXMODE_3D_ONLY          2
#define NAV5_FIXMODE_AUTO             3

/* CFG-PM2: Power management configuration - 0x3B */

#define UBX_CFG_PM2                   0x3B
#define UBX_CFG_PM2_LEN               44

#define PM2_EXT_INT_SEL1              (1 << 4)
#define PM2_EXT_INT_WAKE              (1 << 5)
#define PM2_EXT_INT_BACKUP            (1 << 6)
#define PM2_LIMIT_CURRENT             (1 << 8)
#define PM2_WAIT_TIME_FIX             (1 << 10)
#define PM2_UPDATE_RTC                (1 << 11)
#define PM2_UPDATE_EPH                (1 << 12)
#define PM2_NO_FIX_NO_OFF             (1 << 16)
#define PM2_MODE_CYCLIC               (1 << 17)

#define PM_CYCLIC_MIN_RATE            1000
#define PM_CYCLIC_MAX_RATE            10000

/* UBX NAV class (0x01) message id's */

#define UBX_NAV_AOPSTATUS             0x60
#define UBX_NAV_CLOCK                 0x22
#define UBX_NAV_DGPS                  0x31
#define UBX_NAV_DOP                   0x04
#define UBX_NAV_POSECEF               0x01
#define UBX_NAV_POSLLH                0x02
#define UBX_NAV_SBAS                  0x32
#define UBX_NAV_SOL                   0x06
#define UBX_NAV_STATUS                0x03
#define UBX_NAV_SVINFO                0x30
#define UBX_NAV_TIMEGPS               0x20
#define UBX_NAV_TIMEUTC               0x21
#define UBX_NAV_VELECEF               0x11
#define UBX_NAV_VELDED                0x12

#define UBX_NAV_PVT                   0x07
#define NAV_FLAG_FIX_OK               (1 << 0)
#define NAV_FLAG_DIFF                 (1 << 1)
#define NAV_FLAG_PSM_EN               (1 << 2)
#define NAV_FLAG_PSM_ACQ              (2 << 2)
#define NAV_FLAG_PSM_TRK              (3 << 2)
#define NAV_FLAG_PSM_POT              (4 << 2)
#define NAV_FLAG_PSM_INA              (5 << 2)
#define NAV_FLAG_PSM_MASK             (7 << 2)
#define NAV_FLAG_HEADING_VALID        (1 << 5)

/* UBX AID class (0x0B) message id's */

#define UBX_AID_ALM                   0x30
#define UBX_AID_ALPSRV                0x32
#define UBX_AID_ALPSRV_LEN            16

#define UBX_AID_ALP                   0x50
#define UBX_AID_ALP_POLL              UBX_AID_ALP
#define UBX_AID_ALP_POLL_LEN          0

#define UBX_AID_INI                   0x01
#define UBX_AID_INI_LEN               48
#define UBX_AID_INI_POLL              UBX_AID_INI
#define UBX_AID_INI_POLL_LEN          0

#define UBX_AID_AOP                   0x33
#define UBX_AID_DATA                  0x10
#define UBX_AID_EPH                   0x31
#define UBX_AID_HUI                   0x02

/* UBX protocol state */

enum ubx_state_e
{
  /* Waiting for UBX message */

  UBX_STATE_MSG_WAIT = 0,

  /* Synchronization characters */

  UBX_STATE_MSG_SYNC,

  /* Message header (class id, msg id and length) */

  UBX_STATE_MSG_HEADER,

  /* Receiving UBX message payload */

  UBX_STATE_MSG_PAYLOAD,

  /* Checksum */

  UBX_STATE_MSG_CHECKSUM1,
  UBX_STATE_MSG_CHECKSUM2,
};
typedef enum ubx_state_e ubx_state_t;

/* UBX message acknowledgment timeout in milliseconds */

#define UBX_MSG_ACK_TIMEOUT           200

/* UBX message receive timeout in milliseconds */

#define UBX_MSG_RECV_TIMEOUT          300

/* Maximum UBX message payload length */

#define UBX_MSG_MAX_PAYLOAD_LENGTH    1024

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* UBX message structure */

struct ubx_msg_s
{
  uint8_t class_id;                   /* Class ID */
  uint8_t msg_id;                     /* Message ID */
  uint16_t length;                    /* Payload length (little endian) */
  uint8_t payload[];                  /* Payload data */
} packed_struct;

struct ubx_receiver_s;

/****************************************************************************
 * Name: ubx_callback_t
 *
 * Description:
 *   UBX protocol callback function prototype
 *
 * Input Parameters:
 *   receiver    - Pointer to UBX receiver structure
 *   priv        - Pointer to callback private data
 *   msg         - Pointer to UBX message
 *   timeout     - Timeout occurred. UBX message class and message ID indicates the
 *                 message which timed out
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
typedef int (*ubx_callback_t)(struct ubx_receiver_s const * const receiver,
                              void * const priv,
                              struct ubx_msg_s const * const msg,
                              bool const timeout);

/* UBX receiver state */

struct ubx_state_s
{
  /* UBX receiver state */

  ubx_state_t state;

  /* UBX message waiting for acknowledgment (header only) */

  struct ubx_msg_s waiting_ack;

  /* Currently received UBX message header */

  struct ubx_msg_s header;

  /* UBX message */

  struct ubx_msg_s * msg;

  /* Current receiving offset */

  size_t offset;

  /* Checksum */

  uint8_t ck_a;
  uint8_t ck_b;
};

/* UBX receiver */

struct ubx_receiver_s
{
  /* UBX receiver state */

  struct ubx_state_s state;

  /* UBX send timeout timer ID */

  int send_timer_id;

  /* UBX receive timeout timer ID */

  int receive_timer_id;

  /* UBX callback */

  ubx_callback_t callback;

  /* UBX callback private data */

  void * priv;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Macro for UBX message allocations */

#define UBX_MSG_ALLOC(class, msgid)   ubx_msg_allocate(UBX_CLASS_ ## class, \
                                                       UBX_ ## class ## _ ## msgid, \
                                                       UBX_ ## class ## _ ## msgid ## _LEN)

#define UBX_MSG_FREE(msg)             free(msg);

/* Macro for retrieving data from UBX message with byte offset */

#define UBX_MSG_DATA_GET(m, t, o)             (*((t *)&m->payload[o]))

/* Macro for setting data to UBX message with byte offset */

#define UBX_MSG_DATA_SET(m, t, o, v)          (*((t *)&m->payload[o])) = v

/* Macros for retrieving / setting UBX message data */

#define UBX_GET_U1(msg, offset)               UBX_MSG_DATA_GET(msg, uint8_t, offset)
#define UBX_SET_U1(msg, offset, value)        UBX_MSG_DATA_SET(msg, uint8_t, offset, value)
#define UBX_GET_I1(msg, offset)               UBX_MSG_DATA_GET(msg, int8_t, offset)
#define UBX_SET_I1(msg, offset, value)        UBX_MSG_DATA_SET(msg, int8_t, offset, value)
#define UBX_GET_X1(msg, offset)               UBX_GET_U1(msg, offset)
#define UBX_SET_X1(msg, offset, value)        UBX_SET_U1(msg, offset, value)
#define UBX_GET_U2(msg, offset)               UBX_MSG_DATA_GET(msg, uint16_t, offset)
#define UBX_SET_U2(msg, offset, value)        UBX_MSG_DATA_SET(msg, uint16_t, offset, value)
#define UBX_GET_I2(msg, offset)               UBX_MSG_DATA_GET(msg, int16_t, offset)
#define UBX_SET_I2(msg, offset, value)        UBX_MSG_DATA_SET(msg, int16_t, offset, value)
#define UBX_GET_X2(msg, offset)               UBX_GET_U2(msg, offset)
#define UBX_SET_X2(msg, offset, value)        UBX_SET_U2(msg, offset, value)
#define UBX_GET_U4(msg, offset)               UBX_MSG_DATA_GET(msg, uint32_t, offset)
#define UBX_SET_U4(msg, offset, value)        UBX_MSG_DATA_SET(msg, uint32_t, offset, value)
#define UBX_GET_I4(msg, offset)               UBX_MSG_DATA_GET(msg, int32_t, offset)
#define UBX_SET_I4(msg, offset, value)        UBX_MSG_DATA_SET(msg, int32_t, offset, value)
#define UBX_GET_X4(msg, offset)               UBX_GET_U4(msg, offset)
#define UBX_SET_X4(msg, offset, value)        UBX_SET_U4(msg, offset, value)

/****************************************************************************
 * Name: ubx_initialize
 *
 * Description:
 *   Initialize UBX receiver
 *
 * Input Parameters:
 *   receiver    - Pointer to UBX receiver structure
 *   callback    - Pointer to callback
 *   priv        - Pointer to callback private data
 *
 * Returned Values:
 *   0 (OK) means the function was executed successfully
 *  -1 (ERROR) means the function was executed unsuccessfully.
 *
 ****************************************************************************/
int ubx_initialize(struct ubgps_s * const gps, ubx_callback_t callback, void * const priv);

/****************************************************************************
 * Name: ubx_deinitialize
 *
 * Description:
 *   Deinitialize UBX receiver
 *
 * Input Parameters:
 *   receiver    - Pointer to UBX receiver structure
 *
 * Returned Values:
 *   0 (OK) means the function was executed successfully
 *  -1 (ERROR) means the function was executed unsuccessfully.
 *
 ****************************************************************************/
int ubx_deinitialize(struct ubgps_s * const gps);

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
struct ubx_msg_s * ubx_msg_allocate(const uint8_t class_id, const uint8_t msg_id, const size_t payload_length);

/****************************************************************************
 * Name: ubx_msg_send
 *
 * Description:
 *   Send UBX message
 *
 * Input Parameters:
 *   receiver    - Pointer to UBX receiver structure
 *   fd          - File descriptor to write message
 *   msg         - Pointer to UBX message
 *
 * Returned Values:
 *   0 (OK) means the function was executed successfully
 *  -1 (ERROR) means the function was executed unsuccessfully. Check value of
 *  errno for more details.
 *
 ****************************************************************************/
int ubx_msg_send(struct ubgps_s * const gps, const int fd, struct ubx_msg_s const * const msg);

/****************************************************************************
 * Name: ubx_msg_receive
 *
 * Description:
 *   Receive UBX messages
 *
 * Input Parameters:
 *   receiver    - Pointer to UBX receiver structure
 *   data        - Received data
 *
 * Returned Values:
 *   0 (OK) means the data was accepted by receiver
 *  -1 (ERROR) means the data was ignored by receiver and can be sent to next receiver
 *
 ****************************************************************************/
int ubx_msg_receive(struct ubgps_s * const gps, const uint8_t data);

/****************************************************************************
 * Name: ubx_reset
 *
 * Description:
 *   Reset UBX receiver send / receive timeouts and free allocated memory
 *
 * Input Parameters:
 *   receiver    - Pointer to UBX receiver structure
 *
 * Returned Values:
 *   0 (OK) means the function was executed successfully
 *  -1 (ERROR) means the function was executed unsuccessfully. Check value of
 *  errno for more details.
 *
 ****************************************************************************/
int ubx_reset(struct ubgps_s * const gps);

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
bool ubx_busy(struct ubx_receiver_s * const receiver);

#endif /* __THINGSEE_GPS_UBX_H */
