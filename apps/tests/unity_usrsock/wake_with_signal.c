/****************************************************************************
 * apps/examples/unity_usrsock/wake_with_signal.c
 * Wake blocked IO with signal or daemon abort
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
 *   Authors: Jussi Kivilinna <jussi.kivilinna@haltian.com>
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
#include <nuttx/testing/unity_fixture.h>
#include <sys/socket.h>
#include <assert.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <poll.h>

#include "defines.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

#define TEST_FLAG_PAUSE_USRSOCK_HANDLING (1 << 0)
#define TEST_FLAG_DAEMON_ABORT           (1 << 1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum e_test_type
{
  TEST_TYPE_SOCKET = 0,
  TEST_TYPE_CLOSE,
  TEST_TYPE_CONNECT,
  TEST_TYPE_SETSOCKOPT,
  TEST_TYPE_GETSOCKOPT,
  TEST_TYPE_SEND,
  TEST_TYPE_RECV,
  TEST_TYPE_POLL,
  __TEST_TYPE_MAX,
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/
static pthread_t tid;
static sem_t tid_startsem;
static int test_sd;
static enum e_test_type test_type;
static int test_flags;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static FAR void * usrsock_blocking_socket_thread(FAR void *param)
{
  bool test_abort = !!(test_flags & TEST_FLAG_DAEMON_ABORT);
  bool test_hang = !!(test_flags & TEST_FLAG_PAUSE_USRSOCK_HANDLING);

  TEST_ASSERT_TRUE(test_hang);
  TEST_ASSERT_TRUE(test_abort);

  /* Attempt hanging open socket. */

  TEST_ASSERT_EQUAL(0, usrsocktest_daemon_pause_usrsock_handling(true));

  sem_post(&tid_startsem);
  test_sd = socket(AF_INET, SOCK_STREAM, 0);
  TEST_ASSERT_EQUAL(-1, test_sd);
  TEST_ASSERT_EQUAL(ENETDOWN, errno);

  return NULL;
}

static FAR void * usrsock_blocking_close_thread(FAR void *param)
{
  struct sockaddr_in addr;
  int ret;
  bool test_abort = !!(test_flags & TEST_FLAG_DAEMON_ABORT);
  bool test_hang = !!(test_flags & TEST_FLAG_PAUSE_USRSOCK_HANDLING);

  /* Open socket. */

  test_sd = socket(AF_INET, SOCK_STREAM, 0);
  TEST_ASSERT_TRUE(test_sd >= 0);

  inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr.s_addr);
  addr.sin_family = AF_INET;
  addr.sin_port = htons(255);

  TEST_ASSERT_TRUE(test_hang);
  TEST_ASSERT_TRUE(test_abort);

  /* Attempt hanging close socket. */

  TEST_ASSERT_EQUAL(0, usrsocktest_daemon_pause_usrsock_handling(true));

  sem_post(&tid_startsem);
  ret = close(test_sd);
  TEST_ASSERT_EQUAL(0, ret);
  test_sd = -1;

  return NULL;
}

static FAR void * usrsock_blocking_connect_thread(FAR void *param)
{
  struct sockaddr_in addr;
  int ret;
  bool test_abort = !!(test_flags & TEST_FLAG_DAEMON_ABORT);
  bool test_hang = !!(test_flags & TEST_FLAG_PAUSE_USRSOCK_HANDLING);

  /* Open socket. */

  test_sd = socket(AF_INET, SOCK_STREAM, 0);
  TEST_ASSERT_TRUE(test_sd >= 0);

  inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr.s_addr);
  addr.sin_family = AF_INET;
  addr.sin_port = htons(255);

  if (test_hang)
    {
      TEST_ASSERT_EQUAL(0, usrsocktest_daemon_pause_usrsock_handling(true));
    }

  /* Attempt blocking connect. */

  sem_post(&tid_startsem);
  ret = connect(test_sd, (FAR const struct sockaddr *)&addr, sizeof(addr));
  TEST_ASSERT_EQUAL(-1, ret);
  TEST_ASSERT_EQUAL(test_abort ? ECONNABORTED : EINTR, errno);

  /* Close socket */

  TEST_ASSERT_TRUE(close(test_sd) >= 0);
  test_sd = -1;

  return NULL;
}

static FAR void * usrsock_blocking_setsockopt_thread(FAR void *param)
{
  struct sockaddr_in addr;
  int ret;
  int value;
  bool test_abort = !!(test_flags & TEST_FLAG_DAEMON_ABORT);
  bool test_hang = !!(test_flags & TEST_FLAG_PAUSE_USRSOCK_HANDLING);

  /* Open socket. */

  test_sd = socket(AF_INET, SOCK_STREAM, 0);
  TEST_ASSERT_TRUE(test_sd >= 0);

  inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr.s_addr);
  addr.sin_family = AF_INET;
  addr.sin_port = htons(255);

  TEST_ASSERT_TRUE(test_hang);
  TEST_ASSERT_TRUE(test_abort);

  /* Attempt hanging setsockopt. */

  TEST_ASSERT_EQUAL(0, usrsocktest_daemon_pause_usrsock_handling(true));

  sem_post(&tid_startsem);
  value = 1;
  ret = setsockopt(test_sd, SOL_SOCKET, SO_REUSEADDR, &value, sizeof(value));
  TEST_ASSERT_EQUAL(-1, ret);

  /* Close socket */

  TEST_ASSERT_TRUE(close(test_sd) >= 0);
  test_sd = -1;

  return NULL;
}

static FAR void * usrsock_blocking_getsockopt_thread(FAR void *param)
{
  struct sockaddr_in addr;
  int ret;
  int value;
  socklen_t valuelen;
  bool test_abort = !!(test_flags & TEST_FLAG_DAEMON_ABORT);
  bool test_hang = !!(test_flags & TEST_FLAG_PAUSE_USRSOCK_HANDLING);

  /* Open socket. */

  test_sd = socket(AF_INET, SOCK_STREAM, 0);
  TEST_ASSERT_TRUE(test_sd >= 0);

  inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr.s_addr);
  addr.sin_family = AF_INET;
  addr.sin_port = htons(255);

  TEST_ASSERT_TRUE(test_hang);
  TEST_ASSERT_TRUE(test_abort);

  /* Attempt hanging getsockopt. */

  TEST_ASSERT_EQUAL(0, usrsocktest_daemon_pause_usrsock_handling(true));

  sem_post(&tid_startsem);
  value = -1;
  valuelen = sizeof(value);
  ret = getsockopt(test_sd, SOL_SOCKET, SO_REUSEADDR, &value, &valuelen);
  TEST_ASSERT_EQUAL(-1, ret);

  /* Close socket */

  TEST_ASSERT_TRUE(close(test_sd) >= 0);
  test_sd = -1;

  return NULL;
}

static FAR void * usrsock_blocking_send_thread(FAR void *param)
{
  struct sockaddr_in addr;
  int ret;
  bool test_abort = !!(test_flags & TEST_FLAG_DAEMON_ABORT);
  bool test_hang = !!(test_flags & TEST_FLAG_PAUSE_USRSOCK_HANDLING);

  /* Open socket. */

  test_sd = socket(AF_INET, SOCK_STREAM, 0);
  TEST_ASSERT_TRUE(test_sd >= 0);

  inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr.s_addr);
  addr.sin_family = AF_INET;
  addr.sin_port = htons(255);

  /* Connect socket. */

  ret = connect(test_sd, (FAR const struct sockaddr *)&addr, sizeof(addr));
  TEST_ASSERT_EQUAL(0, ret);

  if (test_hang)
    {
      TEST_ASSERT_EQUAL(0, usrsocktest_daemon_pause_usrsock_handling(true));
    }

  /* Attempt blocking send. */

  sem_post(&tid_startsem);
  ret = send(test_sd, &addr, sizeof(addr), 0);
  TEST_ASSERT_EQUAL(-1, ret);
  TEST_ASSERT_EQUAL(test_abort ? EPIPE : EINTR, errno);

  /* Close socket */

  TEST_ASSERT_TRUE(close(test_sd) >= 0);
  test_sd = -1;

  return NULL;
}

static FAR void * usrsock_blocking_recv_thread(FAR void *param)
{
  struct sockaddr_in addr;
  int ret;
  bool test_abort = !!(test_flags & TEST_FLAG_DAEMON_ABORT);
  bool test_hang = !!(test_flags & TEST_FLAG_PAUSE_USRSOCK_HANDLING);

  /* Open socket. */

  test_sd = socket(AF_INET, SOCK_STREAM, 0);
  TEST_ASSERT_TRUE(test_sd >= 0);

  inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr.s_addr);
  addr.sin_family = AF_INET;
  addr.sin_port = htons(255);

  /* Connect socket. */

  ret = connect(test_sd, (FAR const struct sockaddr *)&addr, sizeof(addr));
  TEST_ASSERT_EQUAL(0, ret);

  if (test_hang)
    {
      TEST_ASSERT_EQUAL(0, usrsocktest_daemon_pause_usrsock_handling(true));
    }

  /* Attempt blocking recv. */

  sem_post(&tid_startsem);
  ret = recv(test_sd, &addr, sizeof(addr), 0);
  TEST_ASSERT_EQUAL(-1, ret);
  TEST_ASSERT_EQUAL(test_abort ? EPIPE : EINTR, errno);

  /* Close socket */

  TEST_ASSERT_TRUE(close(test_sd) >= 0);
  test_sd = -1;

  return NULL;
}

static FAR void * usrsock_blocking_poll_thread(FAR void *param)
{
  struct sockaddr_in addr;
  int ret;
  struct pollfd pfd = {};
  bool test_abort = !!(test_flags & TEST_FLAG_DAEMON_ABORT);
  bool test_hang = !!(test_flags & TEST_FLAG_PAUSE_USRSOCK_HANDLING);

  /* Open socket. */

  test_sd = socket(AF_INET, SOCK_STREAM, 0);
  TEST_ASSERT_TRUE(test_sd >= 0);

  inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr.s_addr);
  addr.sin_family = AF_INET;
  addr.sin_port = htons(255);

  /* Connect socket. */

  ret = connect(test_sd, (FAR const struct sockaddr *)&addr, sizeof(addr));
  TEST_ASSERT_EQUAL(0, ret);

  TEST_ASSERT_TRUE(test_abort);
  if (test_hang)
    {
      TEST_ASSERT_EQUAL(0, usrsocktest_daemon_pause_usrsock_handling(true));
    }

  /* Attempt poll. */

  pfd.fd = test_sd;
  pfd.events = POLLIN;

  sem_post(&tid_startsem);
  ret = poll(&pfd, 1, -1);
  TEST_ASSERT_EQUAL(1, ret);
  TEST_ASSERT_EQUAL(POLLERR | POLLHUP, pfd.revents);

  /* Attempt read from aborted socket */

  ret = recv(test_sd, &addr, sizeof(addr), 0);
  TEST_ASSERT_EQUAL(-1, ret);
  TEST_ASSERT_EQUAL(EPIPE, errno);

  /* Close socket */

  TEST_ASSERT_TRUE(close(test_sd) >= 0);
  test_sd = -1;

  return NULL;
}

static void do_wake_test(enum e_test_type type, int flags)
{
  static const pthread_startroutine_t thread_funcs[__TEST_TYPE_MAX] =
  {
    [TEST_TYPE_SOCKET]      = usrsock_blocking_socket_thread,
    [TEST_TYPE_CLOSE]       = usrsock_blocking_close_thread,
    [TEST_TYPE_CONNECT]     = usrsock_blocking_connect_thread,
    [TEST_TYPE_SETSOCKOPT]  = usrsock_blocking_setsockopt_thread,
    [TEST_TYPE_GETSOCKOPT]  = usrsock_blocking_getsockopt_thread,
    [TEST_TYPE_RECV]        = usrsock_blocking_recv_thread,
    [TEST_TYPE_SEND]        = usrsock_blocking_send_thread,
    [TEST_TYPE_POLL]        = usrsock_blocking_poll_thread,
  };
  int ret;

  /* Start test daemon. */

  TEST_ASSERT_EQUAL(OK, usrsocktest_daemon_start(&usrsocktest_daemon_config));
  TEST_ASSERT_EQUAL(0, usrsocktest_daemon_get_num_active_sockets());

  /* Launch worker thread. */

  test_type = type;
  test_flags = flags;
  ret = pthread_create(&tid, NULL, thread_funcs[type], NULL);
  TEST_ASSERT_EQUAL(OK, ret);

  /* Let worker to start. */

  sem_wait(&tid_startsem);
  usleep(100 * USEC_PER_MSEC); /* Let worker thread proceed to blocking
                                * function. */

  if (!(flags & TEST_FLAG_DAEMON_ABORT))
    {
      /* Wake waiting thread with signal. */

      /* Send signal to task to break out from blocking send. */

      pthread_kill(tid, 1);

      /* Wait threads to complete work. */

      ret = pthread_join(tid, NULL);
      TEST_ASSERT_EQUAL(OK, ret);
      tid = -1;
      TEST_ASSERT_FALSE(Unity.CurrentTestFailed);

      /* Stopping daemon should succeed. */

      TEST_ASSERT_EQUAL(OK, usrsocktest_daemon_stop());
      TEST_ASSERT_EQUAL(0, usrsocktest_endp_malloc_cnt);
      TEST_ASSERT_EQUAL(0, usrsocktest_dcmd_malloc_cnt);
    }
  else
    {
      /* Wake waiting thread with daemon abort. */

      /* Stopping daemon should succeed. */

      TEST_ASSERT_EQUAL(OK, usrsocktest_daemon_stop());
      TEST_ASSERT_EQUAL(0, usrsocktest_endp_malloc_cnt);
      TEST_ASSERT_EQUAL(0, usrsocktest_dcmd_malloc_cnt);

      /* Wait threads to complete work. */

      ret = pthread_join(tid, NULL);
      TEST_ASSERT_EQUAL(OK, ret);
      tid = -1;
      TEST_ASSERT_FALSE(Unity.CurrentTestFailed);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

TEST_GROUP(WakeWithSignal);

/****************************************************************************
 * Name: WakeWithSignal test group setup
 *
 * Description:
 *   Setup function executed before each testcase in this test group
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   None
 *
 ****************************************************************************/
TEST_SETUP(WakeWithSignal)
{
  tid = -1;
  test_sd = -1;
  sem_init(&tid_startsem, 0, 0);
}

/****************************************************************************
 * Name: WakeWithSignal test group teardown
 *
 * Description:
 *   Setup function executed after each testcase in this test group
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   None
 *
 ****************************************************************************/
TEST_TEAR_DOWN(WakeWithSignal)
{
  int ret;

  if (tid != -1)
    {
      ret = pthread_cancel(tid);
      assert(ret == OK);
      ret = pthread_join(tid, NULL);
      assert(ret == OK);
    }
  if (test_sd != -1)
    {
      close(test_sd);
    }
  sem_destroy(&tid_startsem);
}

/****************************************************************************
 * Name: WakeBlockingConnect
 *
 * Description:
 *   Wake blocking connect with signal
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   None
 *
 ****************************************************************************/
TEST(WakeWithSignal, WakeBlockingConnect)
{
  /* Configure test daemon. */

  usrsocktest_daemon_config = usrsocktest_daemon_defconf;
  usrsocktest_daemon_config.delay_all_responses = false;
  usrsocktest_daemon_config.endpoint_block_send = true;
  usrsocktest_daemon_config.endpoint_block_connect = true;
  usrsocktest_daemon_config.endpoint_addr = "127.0.0.1";
  usrsocktest_daemon_config.endpoint_port = 255;

  /* Run test. */

  do_wake_test(TEST_TYPE_CONNECT, 0);
}

/****************************************************************************
 * Name: WakeBlockingSend
 *
 * Description:
 *   Wake blocking send with signal
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   None
 *
 ****************************************************************************/
TEST(WakeWithSignal, WakeBlockingSend)
{
  /* Configure test daemon. */

  usrsocktest_daemon_config = usrsocktest_daemon_defconf;
  usrsocktest_daemon_config.delay_all_responses = false;
  usrsocktest_daemon_config.endpoint_block_send = true;
  usrsocktest_daemon_config.endpoint_block_connect = false;
  usrsocktest_daemon_config.endpoint_addr = "127.0.0.1";
  usrsocktest_daemon_config.endpoint_port = 255;

  /* Run test. */

  do_wake_test(TEST_TYPE_SEND, 0);
}

/****************************************************************************
 * Name: WakeBlockingRecv
 *
 * Description:
 *   Wake blocking recv with signal
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   None
 *
 ****************************************************************************/
TEST(WakeWithSignal, WakeBlockingRecv)
{
  /* Configure test daemon. */

  usrsocktest_daemon_config = usrsocktest_daemon_defconf;
  usrsocktest_daemon_config.delay_all_responses = false;
  usrsocktest_daemon_config.endpoint_block_send = false;
  usrsocktest_daemon_config.endpoint_block_connect = false;
  usrsocktest_daemon_config.endpoint_recv_avail = 0;
  usrsocktest_daemon_config.endpoint_addr = "127.0.0.1";
  usrsocktest_daemon_config.endpoint_port = 255;

  /* Run test. */

  do_wake_test(TEST_TYPE_RECV, 0);
}

/****************************************************************************
 * Name: AbortBlockingConnect
 *
 * Description:
 *   Wake blocking connect with daemon abort
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   None
 *
 ****************************************************************************/
TEST(WakeWithSignal, AbortBlockingConnect)
{
  /* Configure test daemon. */

  usrsocktest_daemon_config = usrsocktest_daemon_defconf;
  usrsocktest_daemon_config.delay_all_responses = false;
  usrsocktest_daemon_config.endpoint_block_send = true;
  usrsocktest_daemon_config.endpoint_block_connect = true;
  usrsocktest_daemon_config.endpoint_addr = "127.0.0.1";
  usrsocktest_daemon_config.endpoint_port = 255;

  /* Run test. */

  do_wake_test(TEST_TYPE_CONNECT, TEST_FLAG_DAEMON_ABORT);
}

/****************************************************************************
 * Name: AbortBlockingSend
 *
 * Description:
 *   Wake blocking send with daemon abort
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   None
 *
 ****************************************************************************/
TEST(WakeWithSignal, AbortBlockingSend)
{
  /* Configure test daemon. */

  usrsocktest_daemon_config = usrsocktest_daemon_defconf;
  usrsocktest_daemon_config.delay_all_responses = false;
  usrsocktest_daemon_config.endpoint_block_send = true;
  usrsocktest_daemon_config.endpoint_block_connect = false;
  usrsocktest_daemon_config.endpoint_addr = "127.0.0.1";
  usrsocktest_daemon_config.endpoint_port = 255;

  /* Run test. */

  do_wake_test(TEST_TYPE_SEND, TEST_FLAG_DAEMON_ABORT);
}

/****************************************************************************
 * Name: AbortBlockingRecv
 *
 * Description:
 *   Wake blocking recv with daemon abort
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   None
 *
 ****************************************************************************/
TEST(WakeWithSignal, AbortBlockingRecv)
{
  /* Configure test daemon. */

  usrsocktest_daemon_config = usrsocktest_daemon_defconf;
  usrsocktest_daemon_config.delay_all_responses = false;
  usrsocktest_daemon_config.endpoint_block_send = false;
  usrsocktest_daemon_config.endpoint_block_connect = false;
  usrsocktest_daemon_config.endpoint_recv_avail = 0;
  usrsocktest_daemon_config.endpoint_addr = "127.0.0.1";
  usrsocktest_daemon_config.endpoint_port = 255;

  /* Run test. */

  do_wake_test(TEST_TYPE_RECV, TEST_FLAG_DAEMON_ABORT);
}

/****************************************************************************
 * Name: PendingRequestBlockingConnect
 *
 * Description:
 *   Wake blocking connect with daemon abort (and daemon not handling pending
 *   request before abort)
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   None
 *
 ****************************************************************************/
TEST(WakeWithSignal, PendingRequestBlockingConnect)
{
  /* Configure test daemon. */

  usrsocktest_daemon_config = usrsocktest_daemon_defconf;
  usrsocktest_daemon_config.delay_all_responses = false;
  usrsocktest_daemon_config.endpoint_block_send = true;
  usrsocktest_daemon_config.endpoint_block_connect = true;
  usrsocktest_daemon_config.endpoint_addr = "127.0.0.1";
  usrsocktest_daemon_config.endpoint_port = 255;

  /* Run test. */

  do_wake_test(TEST_TYPE_CONNECT,
               TEST_FLAG_DAEMON_ABORT | TEST_FLAG_PAUSE_USRSOCK_HANDLING);
}

/****************************************************************************
 * Name: PendingRequestBlockingSend
 *
 * Description:
 *   Wake blocking send with daemon abort (and daemon not handling pending
 *   request before abort)
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   None
 *
 ****************************************************************************/
TEST(WakeWithSignal, PendingRequestBlockingSend)
{
  /* Configure test daemon. */

  usrsocktest_daemon_config = usrsocktest_daemon_defconf;
  usrsocktest_daemon_config.delay_all_responses = false;
  usrsocktest_daemon_config.endpoint_block_send = true;
  usrsocktest_daemon_config.endpoint_block_connect = false;
  usrsocktest_daemon_config.endpoint_addr = "127.0.0.1";
  usrsocktest_daemon_config.endpoint_port = 255;

  /* Run test. */

  do_wake_test(TEST_TYPE_SEND,
               TEST_FLAG_DAEMON_ABORT | TEST_FLAG_PAUSE_USRSOCK_HANDLING);
}

/****************************************************************************
 * Name: PendingRequestBlockingRecv
 *
 * Description:
 *   Wake blocking recv with daemon abort (and daemon not handling pending
 *   request before abort)
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   None
 *
 ****************************************************************************/
TEST(WakeWithSignal, PendingRequestBlockingRecv)
{
  /* Configure test daemon. */

  usrsocktest_daemon_config = usrsocktest_daemon_defconf;
  usrsocktest_daemon_config.delay_all_responses = false;
  usrsocktest_daemon_config.endpoint_block_send = false;
  usrsocktest_daemon_config.endpoint_block_connect = false;
  usrsocktest_daemon_config.endpoint_recv_avail = 0;
  usrsocktest_daemon_config.endpoint_addr = "127.0.0.1";
  usrsocktest_daemon_config.endpoint_port = 255;

  /* Run test. */

  do_wake_test(TEST_TYPE_RECV,
               TEST_FLAG_DAEMON_ABORT | TEST_FLAG_PAUSE_USRSOCK_HANDLING);
}

/****************************************************************************
 * Name: PendingRequestBlockingOpen
 *
 * Description:
 *   Wake blocking open with daemon abort (and daemon not handling pending
 *   request before abort)
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   None
 *
 ****************************************************************************/
TEST(WakeWithSignal, PendingRequestBlockingOpen)
{
  /* Configure test daemon. */

  usrsocktest_daemon_config = usrsocktest_daemon_defconf;
  usrsocktest_daemon_config.delay_all_responses = false;
  usrsocktest_daemon_config.endpoint_block_send = true;
  usrsocktest_daemon_config.endpoint_block_connect = true;
  usrsocktest_daemon_config.endpoint_recv_avail = 0;
  usrsocktest_daemon_config.endpoint_addr = "127.0.0.1";
  usrsocktest_daemon_config.endpoint_port = 255;

  /* Run test. */

  do_wake_test(TEST_TYPE_SOCKET,
               TEST_FLAG_DAEMON_ABORT | TEST_FLAG_PAUSE_USRSOCK_HANDLING);
}

/****************************************************************************
 * Name: PendingRequestBlockingClose
 *
 * Description:
 *   Wake blocking close with daemon abort (and daemon not handling pending
 *   request before abort)
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   None
 *
 ****************************************************************************/
TEST(WakeWithSignal, PendingRequestBlockingClose)
{
  /* Configure test daemon. */

  usrsocktest_daemon_config = usrsocktest_daemon_defconf;
  usrsocktest_daemon_config.delay_all_responses = false;
  usrsocktest_daemon_config.endpoint_block_send = true;
  usrsocktest_daemon_config.endpoint_block_connect = true;
  usrsocktest_daemon_config.endpoint_recv_avail = 0;
  usrsocktest_daemon_config.endpoint_addr = "127.0.0.1";
  usrsocktest_daemon_config.endpoint_port = 255;

  /* Run test. */

  do_wake_test(TEST_TYPE_CLOSE,
               TEST_FLAG_DAEMON_ABORT | TEST_FLAG_PAUSE_USRSOCK_HANDLING);
}

/****************************************************************************
 * Name: PendingRequestBlockingPoll
 *
 * Description:
 *   Wake blocking poll with daemon abort (and daemon not handling pending
 *   request before abort)
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   None
 *
 ****************************************************************************/
TEST(WakeWithSignal, PendingRequestBlockingPoll)
{
  /* Configure test daemon. */

  usrsocktest_daemon_config = usrsocktest_daemon_defconf;
  usrsocktest_daemon_config.delay_all_responses = false;
  usrsocktest_daemon_config.endpoint_block_send = true;
  usrsocktest_daemon_config.endpoint_block_connect = false;
  usrsocktest_daemon_config.endpoint_recv_avail = 0;
  usrsocktest_daemon_config.endpoint_addr = "127.0.0.1";
  usrsocktest_daemon_config.endpoint_port = 255;

  /* Run test. */

  do_wake_test(TEST_TYPE_POLL,
               TEST_FLAG_DAEMON_ABORT | TEST_FLAG_PAUSE_USRSOCK_HANDLING);
}

/****************************************************************************
 * Name: PendingRequestBlockingSetSockOpt
 *
 * Description:
 *   Wake blocking setsockopt with daemon abort (and daemon not handling pending
 *   request before abort)
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   None
 *
 ****************************************************************************/
TEST(WakeWithSignal, PendingRequestBlockingSetSockOpt)
{
  /* Configure test daemon. */

  usrsocktest_daemon_config = usrsocktest_daemon_defconf;
  usrsocktest_daemon_config.delay_all_responses = false;
  usrsocktest_daemon_config.endpoint_block_send = true;
  usrsocktest_daemon_config.endpoint_block_connect = true;
  usrsocktest_daemon_config.endpoint_recv_avail = 0;
  usrsocktest_daemon_config.endpoint_addr = "127.0.0.1";
  usrsocktest_daemon_config.endpoint_port = 255;

  /* Run test. */

  do_wake_test(TEST_TYPE_SETSOCKOPT,
               TEST_FLAG_DAEMON_ABORT | TEST_FLAG_PAUSE_USRSOCK_HANDLING);
}

/****************************************************************************
 * Name: PendingRequestBlockingGetSockOpt
 *
 * Description:
 *   Wake blocking getsockopt with daemon abort (and daemon not handling pending
 *   request before abort)
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   None
 *
 ****************************************************************************/
TEST(WakeWithSignal, PendingRequestBlockingGetSockOpt)
{
  /* Configure test daemon. */

  usrsocktest_daemon_config = usrsocktest_daemon_defconf;
  usrsocktest_daemon_config.delay_all_responses = false;
  usrsocktest_daemon_config.endpoint_block_send = true;
  usrsocktest_daemon_config.endpoint_block_connect = true;
  usrsocktest_daemon_config.endpoint_recv_avail = 0;
  usrsocktest_daemon_config.endpoint_addr = "127.0.0.1";
  usrsocktest_daemon_config.endpoint_port = 255;

  /* Run test. */

  do_wake_test(TEST_TYPE_GETSOCKOPT,
               TEST_FLAG_DAEMON_ABORT | TEST_FLAG_PAUSE_USRSOCK_HANDLING);
}
