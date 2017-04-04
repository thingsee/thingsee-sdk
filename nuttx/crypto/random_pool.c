/****************************************************************************
 * crypto/random_pool.c
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <debug.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/random.h>
#include <nuttx/board.h>

#include <nuttx/crypto/blake2s.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

#define ROTL_32(x,n) ( ((x) << (n)) | ((x) >> (32-(n))) )
#define ROTR_32(x,n) ( ((x) >> (n)) | ((x) << (32-(n))) )

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rng_s
{
  sem_t rd_sem; /* Threads can only exclusively access the RNG */
  uint32_t rd_addptr;
  uint32_t rd_newentr;
  uint8_t rd_rotate;
};

enum
{
  POOL_SIZE = ENTROPY_POOL_SIZE,
  POOL_MASK = (POOL_SIZE - 1),

  MIN_SEED_NEW_ENTROPY_WORDS = 128,
  MAX_SEED_NEW_ENTROPY_WORDS = 1024
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct rng_s g_rng;

#ifndef CONFIG_BOARD_ENTROPY_POOL
  /* TODO: remove when moving to board level completed */
  #define CONFIG_BOARD_ENTROPY_POOL 1
  static struct entropy_pool_s board_entropy_pool
      __attribute__((section(".nuttx.randomdata")));
#endif

#ifdef CONFIG_BOARD_ENTROPY_POOL
/* Entropy pool structure can be provided by board source. Use for this is,
 * for example, allocate entropy pool from special area of RAM which content
 * is kept over system reset. */

#  define entropy_pool board_entropy_pool
#else
static struct entropy_pool_s entropy_pool;
#endif

/* Polynomial from paper "The Linux Pseudorandom Number Generator Revisited"
 * x^POOL_SIZE + x^104 + x^76 + x^51 + x^25 + x + 1 */

static const uint32_t pool_stir[] = { POOL_SIZE, 104, 76, 51, 25, 1 };

/* Derived from IEEE 802.3 CRC-32 */

static const uint32_t pool_twist[8] =
{
  0x00000000, 0x3b6e20c8, 0x76dc4190, 0x4db26158,
  0xedb88320, 0xd6d6a3e8, 0x9b64c2b0, 0xa00ae278
};

/* The BLAKE2Xs algorithm used as random number generator. */

static struct
{
  blake2s_param param;
  blake2s_state ctx;
  char out_root[BLAKE2S_OUTBYTES];
  uint32_t out_node_offset;
} g_blake2xs_rng;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: addentropy
 *
 * Description:
 *
 * This function adds a number of integers into the entropy pool.
 * The pool is stirred with a polynomial of degree POOL_SIZE over GF(2).
 *
 * Code is inspired by add_entropy_words() function of OpenBSD kernel.
 *
 * Parameters:
 *   buf  -   Buffer of integers to be added
 *   n    -   Number of elements in buf
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void addentropy(FAR const uint32_t *buf, size_t n)
{
  /* Compile time check for that POOL_SIZE is power of two. */

  static char pool_size_p2_check[1 - ((POOL_SIZE & (POOL_SIZE - 1)) * 2)];

  UNUSED(pool_size_p2_check);

  while (n-- > 0)
    {
      uint32_t rotate, w, i;

      rotate = g_rng.rd_rotate;
      w = ROTL_32(*buf, rotate);
      i = g_rng.rd_addptr = (g_rng.rd_addptr - 1) & POOL_MASK;

      /* Normal round, we add 7 bits of rotation to the pool.
       * At the beginning of the pool, we add extra 7 bits
       * rotation, in order for successive passes spread the
       * input bits across the pool evenly.
       */

      g_rng.rd_rotate = (rotate + (i ? 7 : 14)) & 31;

      /* XOR pool contents corresponding to polynomial terms */

      w ^= entropy_pool.pool[(i + pool_stir[1]) & POOL_MASK];
      w ^= entropy_pool.pool[(i + pool_stir[2]) & POOL_MASK];
      w ^= entropy_pool.pool[(i + pool_stir[3]) & POOL_MASK];
      w ^= entropy_pool.pool[(i + pool_stir[4]) & POOL_MASK];
      w ^= entropy_pool.pool[(i + pool_stir[5]) & POOL_MASK];
      w ^= entropy_pool.pool[i]; /* 2^POOL_SIZE */

      entropy_pool.pool[i] = (w >> 3) ^ pool_twist[w & 7];
      buf++;

      g_rng.rd_newentr += sizeof(uint32_t);
   }
}

/****************************************************************************
 * Function: getentropy
 *
 * Description:
 *   Hash entropy pool to BLAKE2s context. This is an internal interface for
 *   seeding out-facing BLAKE2Xs random bit generator from entropy pool.
 *
 *   Code is inspired by extract_entropy() function of OpenBSD kernel.
 *
 *   Note that this function cannot fail, other than by asserting.
 *
 *   Warning: In protected kernel builds, this interface MUST NOT be
 *   exported to userspace. This interface MUST NOT be used as a
 *   general-purpose random bit generator!
 *
 * Parameters:
 *   S  - BLAKE2s instance that will absorb entropy pool
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void getentropy(FAR blake2s_state *S)
{
  add_sw_randomness(g_rng.rd_newentr);

  /* Absorb the entropy pool */

  blake2s_update(S, entropy_pool.pool, sizeof(entropy_pool.pool));

  /* Add something back so repeated calls to this function
   * return different values.
   */

  add_sw_randomness(sizeof(entropy_pool.pool));

#ifdef CONFIG_ARCH_CHIP_STM32
  /* TODO: Add arch_extract_entropy? */
  extern uint32_t idle_wastecounter;

  addentropy(&idle_wastecounter, 1);
#else
  /* TODO: Why is there no up_waste() for other archs? */

  uint32_t bogus = ENXIO ^ nbytes;
  addentropy(&bogus, 1);
#endif

  g_rng.rd_newentr = 0;
}

/* The BLAKE2Xs based random number generator algorithm.
 *
 * BLAKE2X is a extensible-output function (XOF) variant of BLAKE2 hash
 * function. One application of XOFs is use as deterministic random bit
 * number generator (DRBG) as used here. BLAKE2 specification is available
 * at https://blake2.net/
 *
 * BLAKE2Xs here  implementation is based on public-domain/CC0 BLAKE2 reference
 * implementation by Samual Neves, at
 *  https://github.com/BLAKE2/BLAKE2/tree/master/ref
 * Copyright 2012, Samuel Neves <sneves@dei.uc.pt>
 */

static void rng_reseed(void)
{
  blake2s_param P = {};

  /* Reset output node counter. */

  g_blake2xs_rng.out_node_offset = 0;

  /* Initialize parameter block */

  P.digest_length = BLAKE2S_OUTBYTES;
  P.key_length    = 0;
  P.fanout        = 1;
  P.depth         = 1;
  blake2_store32(P.leaf_length, 0);
  blake2_store32(P.node_offset, 0);
  blake2_store16(P.xof_length, 0xffff);
  P.node_depth    = 0;
  P.inner_length  = 0;
  g_blake2xs_rng.param = P;

  blake2s_init_param(&g_blake2xs_rng.ctx, &g_blake2xs_rng.param);

  /* Initialize with randomness from entropy pool */

  getentropy(&g_blake2xs_rng.ctx);

  /* Absorb also the previous root */

  blake2s_update(&g_blake2xs_rng.ctx, g_blake2xs_rng.out_root,
                 sizeof(g_blake2xs_rng.out_root));

  /* Finalize the new root hash */

  blake2s_final(&g_blake2xs_rng.ctx, g_blake2xs_rng.out_root,
                BLAKE2S_OUTBYTES);

  explicit_bzero(&g_blake2xs_rng.ctx, sizeof(g_blake2xs_rng.ctx));

  /* Setup parameters for output phase. */

  g_blake2xs_rng.param.key_length = 0;
  g_blake2xs_rng.param.fanout = 0;
  blake2_store32(g_blake2xs_rng.param.leaf_length, BLAKE2S_OUTBYTES);
  g_blake2xs_rng.param.inner_length = BLAKE2S_OUTBYTES;
  g_blake2xs_rng.param.node_depth = 0;
}

static void rng_buf_internal(FAR void *bytes, size_t nbytes)
{
  static bool initialized = false;

  if (!initialized)
    {
      rng_reseed();
      initialized = true;
    }
  else if (g_rng.rd_newentr >= MAX_SEED_NEW_ENTROPY_WORDS)
    {
      /* Initial entropy is low. Reseed when we have accumulated more. */

      rng_reseed();
    }
  else if (g_blake2xs_rng.out_node_offset == UINT32_MAX)
    {
      /* Maximum BLAKE2Xs output reached (2^32-1 output blocks, maximum 128 GiB
       * bytes), reseed. */

      rng_reseed();
    }

  /* Output phase for BLAKE2Xs. */

  for (; nbytes > 0; ++g_blake2xs_rng.out_node_offset)
    {
      size_t block_size = MIN(nbytes, BLAKE2S_OUTBYTES);

      /* Initialize state */

      g_blake2xs_rng.param.digest_length = block_size;
      blake2_store32(g_blake2xs_rng.param.node_offset,
                     g_blake2xs_rng.out_node_offset);
      blake2s_init_param(&g_blake2xs_rng.ctx, &g_blake2xs_rng.param);

      /* Process state and output random bytes */

      blake2s_update(&g_blake2xs_rng.ctx, g_blake2xs_rng.out_root,
                     sizeof(g_blake2xs_rng.out_root));
      blake2s_final(&g_blake2xs_rng.ctx, bytes, block_size);

      bytes += block_size;
      nbytes -= block_size;
    }
}

static void rng_init(void)
{
  lldbg("Initializing RNG\n");

  memset(&g_rng, 0, sizeof(struct rng_s));
  sem_init(&g_rng.rd_sem, 0, 1);

  /* We do not initialize real key here because this is called
   * quite early in boot and there may not be enough entropy.
   *
   * Board level may define CONFIG_BOARD_INITRNGSEED if it implements
   * early random seeding.
   */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

void up_rngaddint(enum rnd_source_t kindof, int val)
{
  uint32_t buf[2];

  /* We don't actually track what kind of entropy we
   * receive, just add it all to pool.
   */

  buf[0] = (uint32_t)kindof ^ ROTL_32(val, 27);
  buf[1] = val;

  up_rngaddentropy(buf, 2);
}

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

void up_rngaddentropy(FAR const uint32_t *buf, size_t n)
{
  uint32_t tbuf[1];
  struct timespec ts;

  (void)clock_gettime(CLOCK_REALTIME, &ts);

  tbuf[0] = ROTL_32(ts.tv_nsec, 17) ^ ROTL_32(ts.tv_sec, 3);

  if (n == 0)
    {
      addentropy(tbuf, 1);
    }
  else
    {
      tbuf[0] ^= buf[0];

      addentropy(tbuf, 1);

      if (n > 1)
        {
          addentropy(&buf[1], n - 1);
        }
    }
}

/****************************************************************************
 * Function: up_rngreseed
 *
 * Description:
 *   Force reseeding random number generator from entropy pool
 *
 ****************************************************************************/

void up_rngreseed(void)
{
  while (sem_wait(&g_rng.rd_sem) != 0)
    {
      assert(errno == EINTR);
    }
  if (g_rng.rd_newentr >= MIN_SEED_NEW_ENTROPY_WORDS)
    rng_reseed();
  sem_post(&g_rng.rd_sem);
}

/****************************************************************************
 * Function: up_randompool_initialize
 *
 * Description:
 *   Initialize entropy pool and random number generator
 *
 ****************************************************************************/

void up_randompool_initialize(void)
{
  rng_init();

#ifdef CONFIG_BOARD_INITRNGSEED
  board_init_rndseed();
#endif
}

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

void getrandom(FAR void *bytes, size_t nbytes)
{
  while (sem_wait(&g_rng.rd_sem) != 0)
    {
      assert(errno == EINTR);
    }

  rng_buf_internal(bytes, nbytes);
  sem_post(&g_rng.rd_sem);
}

