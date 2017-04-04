/****************************************************************************
 * drivers/dev_random.c
 *
 *   Copyright (C) 2015-2017 Haltian Ltd. All rights reserved.
 *   Authors: Juha Niskanen <juha.niskanen@haltian.com>
 *            Jussi Kivilinna <jussi.kivilinna@haltian.com>
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

#include <stddef.h>
#include <string.h>
#include <debug.h>
#include <assert.h>
#include <poll.h>

#include <nuttx/arch.h>
#include <sys/random.h>
#include <nuttx/random.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define MIN(a, b)   (((a) < (b)) ? (a) : (b))

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t devurand_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen);
static ssize_t devurand_write(FAR struct file *filep, FAR const char *buffer,
                              size_t buflen);
#ifndef CONFIG_DISABLE_POLL
static int devurand_poll(FAR struct file *filep, FAR struct pollfd *fds,
                         bool setup);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_urand_fops =
{
  NULL,                         /* open */
  NULL,                         /* close */
  devurand_read,                /* read */
  devurand_write,               /* write */
  NULL,                         /* seek */
  NULL                          /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , devurand_poll               /* poll */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devurand_read
 ****************************************************************************/

static ssize_t devurand_read(FAR struct file *filep, FAR char *buffer,
                             size_t len)
{
#ifdef CONFIG_CRYPTO_RANDOM_POOL
  getrandom(buffer, len);
  return len;
#else
  return -ENOSYS;
#endif
}

/****************************************************************************
 * Name: devurand_write
 ****************************************************************************/

static ssize_t devurand_write(FAR struct file *filep, FAR const char *buffer,
                              size_t len)
{
  /* Write can be used to re-seed the PRNG state. */

#ifdef CONFIG_CRYPTO_RANDOM_POOL
  const unsigned int alignmask = sizeof(uint32_t) - 1;
  const size_t initlen = len;
  uint32_t tmp = 0;
  size_t currlen;

  if (!len)
    return 0;

  /* Seed entropy pool with data from user. */

  if ((uintptr_t)buffer & alignmask)
    {
      /* Make unaligned input aligned. */

      currlen = MIN(sizeof(uint32_t) - ((uintptr_t)buffer & alignmask), len);
      memcpy(&tmp, buffer, currlen);
      up_rngaddint(RND_SRC_SW, tmp);

      len -= currlen;
      buffer += currlen;
    }

  if (len >= sizeof(uint32_t))
    {
      /* Handle bulk aligned, word-sized data. */

      DEBUGASSERT(((uintptr_t)buffer & alignmask) == 0);
      currlen = len / sizeof(uint32_t);
      up_rngaddentropy((FAR uint32_t *)buffer, currlen);
      buffer += currlen * sizeof(uint32_t);
      len %= sizeof(uint32_t);
    }

  if (len > 0)
    {
      /* Handle trailing bytes. */

      DEBUGASSERT(len < sizeof(uint32_t));
      memcpy(&tmp, buffer, len);
      up_rngaddint(RND_SRC_SW, tmp);
    }

  /* Reseeding of random number generator from entropy pool. */

  up_rngreseed();

  return initlen;
#else
  return -ENOSYS;
#endif
}

/****************************************************************************
 * Name: devurand_poll
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static int devurand_poll(FAR struct file *filep, FAR struct pollfd *fds,
                         bool setup)
{
  if (setup)
    {
      fds->revents |= (fds->events & (POLLIN | POLLOUT));
      if (fds->revents != 0)
        {
          sem_post(fds->sem);
        }
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: up_rnginitialize
 ****************************************************************************/

void up_rnginitialize(void)
{
#ifdef CONFIG_CRYPTO_RANDOM_POOL
  up_randompool_initialize();
#endif

  register_driver("/dev/random", &g_urand_fops, 0666, NULL);
  register_driver("/dev/urandom", &g_urand_fops, 0666, NULL);
}
