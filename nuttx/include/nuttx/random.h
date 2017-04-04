/****************************************************************************
 * include/nuttx/random.h
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

#ifndef __INCLUDE_NUTTX_RANDOM_H
#define __INCLUDE_NUTTX_RANDOM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stddef.h>

#include <sys/random.h> /* getrandom() */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TODO: Make this Kconfig CONFIG_CRYPTO_RANDOM_POOL */

#define CONFIG_CRYPTO_RANDOM_POOL 1

/* in 32-bit integers, must be power of two */

#define ENTROPY_POOL_SIZE        128

#define add_sensor_randomness(x) up_rngaddint(RND_SRC_SENSOR, (x))
#define add_time_randomness(x)   up_rngaddint(RND_SRC_TIME, (x))
#define add_hw_randomness(x)     up_rngaddint(RND_SRC_HW, (x))
#define add_sw_randomness(x)     up_rngaddint(RND_SRC_SW, (x))
#define add_ui_randomness(x)     up_rngaddint(RND_SRC_UI, (x))

/* Allow above macros to always exist in source without ifdefs */

#ifndef CONFIG_CRYPTO_RANDOM_POOL
#  define up_rngaddint(k, x)            ((void)(k),(void)(x))
#  define up_rngaddentropy(buf, n)      ((void)(buf),(void)(x))
#endif

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Entropy pool structure */

struct entropy_pool_s
{
  uint32_t pool[ENTROPY_POOL_SIZE];
};

/* Randomness sources */

enum rnd_source_t
{
  RND_SRC_TIME,
  RND_SRC_SENSOR,
  RND_SRC_HW,      /* unique per HW UID or coming from factory line. */
  RND_SRC_SW,      /* unique per SW version. */
  RND_SRC_UI       /* buttons etc. user-visible interface elements. */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_BOARD_ENTROPY_POOL
/* Entropy pool structure can be provided by board source. Use for this is,
 * for example, allocate entropy pool from special area of RAM which content
 * is kept over system reset. */

extern struct entropy_pool_s board_entropy_pool;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_CRYPTO_RANDOM_POOL

/****************************************************************************
 * Function: getrandom
 *
 * Description:
 *   Fill a buffer of arbitrary length with randomness. This is the
 *   preferred interface for getting random numbers. The traditional
 *   /dev/random approach is susceptible for things like the attacker
 *   exhausting file descriptors on purpose.
 *
 *   Note that this function cannot fail, other than by asserting.
 *
 * Parameters:
 *   bytes  - Buffer for returned random bytes
 *   nbytes - Number of bytes requested.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void getrandom(FAR void *bytes, size_t nbytes);

/****************************************************************************
 * Function: up_rngaddint
 *
 * Description:
 *   Add one integer to entropy pool, contributing a specific kind
 *   of entropy to pool.
 *
 * Parameters:
 *   kindof  - Enumeration constant telling where val came from
 *   val     - Integer to be added
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_rngaddint(enum rnd_source_t kindof, int val);

/****************************************************************************
 * Function: up_rngaddentropy
 *
 * Description:
 *   Add buffer of integers to entropy pool.
 *
 * Parameters:
 *   buf  -   Buffer of integers to be added
 *   n    -   Number of elements in buf
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_rngaddentropy(FAR const uint32_t *buf, size_t n);

/****************************************************************************
 * Function: up_rngreseed
 *
 * Description:
 *   Force reseeding random number generator from entropy pool
 *
 ****************************************************************************/

void up_rngreseed(void);

/****************************************************************************
 * Function: up_randompool_initialize
 *
 * Description:
 *   Initialize entropy pool and random number generator
 *
 ****************************************************************************/

void up_randompool_initialize(void);

#endif /* CONFIG_CRYPTO_RANDOM_POOL */

#endif /* __INCLUDE_NUTTX_RANDOM_H */

