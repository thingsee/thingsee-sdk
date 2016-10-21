/****************************************************************************
 * apps/ts_engine/connectors/conn_comm_execute_mqtt.c
 *
 * Copyright (C) 2015 Haltian Ltd. All rights reserved.
 * Authors: Timo Voutilainen <timo.voutilainen@haltian.com>
 *          Jussi Kivilinna <jussi.kivilinna@haltian.com>
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

#ifdef CONFIG_THINGSEE_MQTT_PROTOCOL

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <assert.h>
#include <debug.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <errno.h>
#include <netinet/in.h>

#include <apps/system/conman.h>

#include "connector.h"
#include "conn_comm.h"
#include "conn_comm_link.h"
#include "conn_comm_util.h"
#include "conn_comm_execute_mqtt.h"
#include "con_dbg.h"
#include "../engine/client.h"

#include "MQTTNuttx.h"
#include "MQTTClient.h"
#include "MQTTPacket.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CONNECTION_MQTT_CMD_TIMEOUT   (1 * 1000) /* msecs (too short for 2G?) */
#define CONNECTION_MQTT_BUFFER_SIZE   (0x100)
#define CONNECTION_MQTT_VERSION       3

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct mqtt_data_s
{
  Client mqtt_client;
  Network mqtt_network;
  MQTTPacket_connectData data;
  uint32_t connection_id;
  bool initialized;
  char *topic;
  unsigned char buf[CONNECTION_MQTT_BUFFER_SIZE];
  unsigned char read_buf[CONNECTION_MQTT_BUFFER_SIZE];
} mqtt_data_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mqtt_data_t g_mqtt_data;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void conn_mqtt_attempt_to_reconnect(char *host, int port, struct conn_network_task_s *task)
{
  /* Check if we are still online. If no, reconnect */

  int ret = OK;
  uint32_t connid;

  con_dbg("Host: %s, port: %d\n", host, port);

  con_dbg_save_pos();

  if (!g_mqtt_data.mqtt_client.isconnected)
    {
      if (g_mqtt_data.connection_id == -1)
        {
          ret = request_connection(&connid, true);
          if (ret < 0)
            {
              con_dbg("request_connection on failed, skipping task\n");
              goto destroy_task;
            }
          g_mqtt_data.connection_id = connid;
        }

      ret = ConnectNetwork(&g_mqtt_data.mqtt_network, host, port);
      con_dbg("MQTT network connection done with result: %d\n", ret);

      if (ret < 0)
        {
          con_dbg("Connection to network failed, closing connection\n");
          goto close_connection;
        }

      ret = MQTTConnect(&g_mqtt_data.mqtt_client, &g_mqtt_data.data);
      con_dbg("MQTT connection done with result: %d\n", ret);

      if (ret < 0)
        {
          con_dbg("MQTT connection failed, closing connection: %d\n", ret);
          goto close_socket;
        }
    }

  return;

close_socket:
  g_mqtt_data.mqtt_network.disconnect(&g_mqtt_data.mqtt_network);

close_connection:
  request_connection(&connid, false);

destroy_task:
  conn_destroy_task(task);
}

static int mqtt_write (Network *n, unsigned char *buffer, int len, int timeout_ms)
{
  /* We need those variables to only make it compatible with
   * conn_comm_write-function */

  const char *bufs[2] = { (char *)buffer, NULL };
  const size_t lens[2] = { len, 0 };
  const char **pbuf = bufs;
  const size_t *plen = lens;

  con_dbg("Write data: %s, Length: %d\n", (char *)buffer, len);

  return conn_comm_write(n->my_socket, pbuf, plen, timeout_ms);
}

static int mqtt_read (Network* n, unsigned char* buffer, int len, int timeout_ms)
{
  con_dbg("Read data\n");

  return conn_comm_read(n->my_socket, buffer, len, timeout_ms);
}

static void mqtt_disconnect (Network* n)
{
  con_dbg("Disconnect called\n");

  /* I have some doubts this one will be called in nuttx
   * Let's put it in error handlers */

  g_mqtt_data.mqtt_client.isconnected = 0;

  close (n->my_socket);
}

static inline void mqtt_init_connect_data(MQTTPacket_connectData *data)
{
  MQTTPacket_connectData con_data = MQTTPacket_connectData_initializer;

  memcpy(data, &con_data, sizeof(MQTTPacket_connectData));

  con_dbg("MQTT connection data initialized\n");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int conn_execute_mqtt_request(char *pdata, size_t datalen, struct conn_network_task_s *task, int *status_code)
{
  int ret = OK;
  MQTTMessage msg;

  con_dbg("Data: %s, Length: %d, Host: %s, Port: %d\n",
          pdata, datalen, con->host, con->port);

  con_dbg_save_pos();

  pthread_mutex_lock(&con->mutex);
  mqtt_attempt_to_reconnect(con->host, (int)con->port, task);

  /* Fill MQTTMessage structure with data */

  msg.qos = QOS0;
  msg.retained = false;
  msg.dup = false;
  msg.payload = (void *)pdata;
  msg.payloadlen = datalen;

  /* Send data to the server */

  ret = MQTTPublish(&g_mqtt_data.mqtt_client, g_mqtt_data.topic, &msg);
  con_dbg("MQTT message published with result: %d and topic: %s\n", ret, g_mqtt_data.topic);

  if (ret < 0)
    {
      /* We are here. Possibly connection is closed or pipe is broken.
       * Close MQTT connection */

      mqtt_uninit();
      *status_code = ERROR;
    }

  /* To make everything compatible for now the 200 is sent back */

  *status_code = 200;
  pthread_mutex_unlock(&con->mutex);

  return ret;
}

void conn_mqtt_init_con_objects(char *username, char *password, const char *topic)
{
  /* Well, for the very first version that will do.
   * However, the better way to do things is to separate
   * all mqtt related things to the separated file
   * that resides on the top of the MQTTNuttx.c */

  /* Initialize basic protocol data */

  conn_set_protocol_type(CON_PROTOCOL_MQTT);
  g_mqtt_data.connection_id = -1;

  /* Initialize network structure */

  NewNetwork(&g_mqtt_data.mqtt_network);
  g_mqtt_data.mqtt_network.mqttwrite = mqtt_write;
  g_mqtt_data.mqtt_network.mqttread = mqtt_read;
  g_mqtt_data.mqtt_network.disconnect = mqtt_disconnect;

  /* MQTT topic. Basically the name of the end point or
   * like a channel in irq */

  g_mqtt_data.topic = (char *)topic;

  /* Initialize Client data structure */

  MQTTClient(&g_mqtt_data.mqtt_client, &g_mqtt_data.mqtt_network,
             CONNECTION_MQTT_CMD_TIMEOUT, g_mqtt_data.buf, CONNECTION_MQTT_BUFFER_SIZE,
             g_mqtt_data.read_buf, CONNECTION_MQTT_BUFFER_SIZE);

  /* Initialize connection data. Needed to be initialized once */

  mqtt_init_connect_data(&g_mqtt_data.data);

  g_mqtt_data.data.willFlag = 0;
  g_mqtt_data.data.MQTTVersion = CONNECTION_MQTT_VERSION;
  g_mqtt_data.data.clientID.cstring = username;
  g_mqtt_data.data.username.cstring = username;
  g_mqtt_data.data.password.cstring = password;

  g_mqtt_data.data.keepAliveInterval = 10;
  g_mqtt_data.data.cleansession = 1;

  con_dbg("All data structures initialized\n");
}

void conn_mqtt_uninit(void)
{
  pthread_mutex_lock(&con->mutex);
  if (g_mqtt_data.mqtt_client.isconnected)
    {
      MQTTDisconnect(&g_mqtt_data.mqtt_client);
      g_mqtt_data.mqtt_network.disconnect(&g_mqtt_data.mqtt_network);
      request_connection(&g_mqtt_data.connection_id, false);
      g_mqtt_data.connection_id = -1;
    }

  con_dbg("Connection closed and callback called\n");
  pthread_mutex_unlock(&con->mutex);
}

#endif
