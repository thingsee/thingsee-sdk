/****************************************************************************
 * apps/thingsee/lib/ts_mainloop_locker.c
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
 *   Author: Jussi Kivilinna <jussi.kivilinna@haltian.com>
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

#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>
#include <assert.h>
#include <debug.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <semaphore.h>

#include <apps/thingsee/ts_core.h>
#include <apps/thingsee/ts_mainloop_locker.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_THINGSEE_MAINLOOP_LOCKER

static struct {
	int writefd;
	int readfd;
	sem_t sem_unlock_event;
	sem_t sem_lock_event;
	volatile bool locked;
} lock;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int ts_mainloop_lock_read_handler(const struct pollfd * const pfd, void * const priv)
{
	uint8_t msg = 0;
	ssize_t ret;

	ret = read(lock.readfd, &msg, 1);
	if (ret <= 0)
		return OK;

	switch (msg) {
	case 'L': /* Lock */
		DEBUGASSERT(!lock.locked); /* Must be unlocked. */

		lock.locked = true;
		sem_post(&lock.sem_lock_event);

		/* Lock by starting to wait for semaphore. */

		do {
			ret = sem_wait(&lock.sem_unlock_event);
		} while (ret != OK);

		break;
	}

	return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int ts_mainloop_lock(void)
{
	ssize_t ret;
	uint8_t msg;

	if (lock.writefd < 0 || lock.readfd < 0)
		return -1;

	if (lock.locked)
		return -1; /* Already locked. */

	msg = 'L';
	ret = write(lock.writefd, &msg, 1);
	if (ret < 0)
		return -1;

	/* Wait mainloop to lock-up. */
	do {
		(void)sem_wait(&lock.sem_lock_event);
	} while (!lock.locked);

	return OK;
}

int ts_mainloop_unlock(void)
{
	if (lock.writefd < 0 || lock.readfd < 0)
		return -1;

	if (!lock.locked)
		return -1; /* Already unlocked. */

	/* Unlock mainloop. */
	lock.locked = false;
	sem_post(&lock.sem_unlock_event);

	return OK;
}

#endif /*CONFIG_THINGSEE_MAINLOOP_LOCKER*/

int ts_mainloop_lock_init(void)
{
#ifdef CONFIG_THINGSEE_MAINLOOP_LOCKER
	int fds[2];
	int err;
	int flags;

	/* Pipe for command queue */

	err = pipe(fds);
	DEBUGASSERT(err == OK);

	lock.readfd = fds[0];
	lock.writefd = fds[1];

	sem_init(&lock.sem_unlock_event, 0, 0);
	sem_init(&lock.sem_lock_event, 0, 0);

	/* Read side as non-blocking. */

	flags = fcntl(lock.readfd, F_GETFL, 0);
	flags |= O_NONBLOCK;
	err = fcntl(lock.readfd, F_SETFL, flags);
	DEBUGASSERT(err == OK);

	/* Register poll callback. */

	err = ts_core_fd_register(lock.readfd, POLLIN, &ts_mainloop_lock_read_handler, NULL);
	DEBUGASSERT(err == OK);
#endif

	return OK;
}
