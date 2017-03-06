/************************************************************************************
 * drivers/dev_random.c
 *
 *   Copyright (C) 2015-2017 Haltian Ltd. All rights reserved.
 *   Authors: Juha Niskanen <juha.niskanen@haltian.com>
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

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

#include <md5.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

#define ROTL_32(x,n) ( ((x) << (n)) | ((x) >> (32-(n))) )
#define ROTR_32(x,n) ( ((x) >> (n)) | ((x) << (32-(n))) )

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t rng_dev_read(struct file *filep, char *buffer, size_t n);

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rng_dev_s
{
  sem_t rd_devsem;    /* Threads can only exclusively access the RNG */
  uint32_t rd_addptr;
  uint32_t rd_newentr;
  uint8_t  rd_rotate;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct rng_dev_s g_rngdev;

enum {
  POOL_SIZE = 128, /* in 32-bit integers, must be power of two */
  POOL_MASK = (POOL_SIZE - 1)
};

static uint32_t entropy_pool[POOL_SIZE] __attribute__((section(".nuttx.randomdata")));

/* Polynomial from paper "The Linux Pseudorandom Number Generator Revisited" */
/* x^POOL_SIZE + x^104 + x^76 + x^51 + x^25 + x + 1 */
static const uint32_t pool_stir[] = { POOL_SIZE, 104, 76, 51, 25, 1 };

/* Derived from IEEE 802.3 CRC-32 */
static const uint32_t pool_twist[8] = {
    0x00000000, 0x3b6e20c8, 0x76dc4190, 0x4db26158,
    0xedb88320, 0xd6d6a3e8, 0x9b64c2b0, 0xa00ae278
};

static const struct file_operations g_rngops =
{
  0,               /* open */
  0,               /* close */
  rng_dev_read,    /* read */
  0,               /* write */
  0,               /* seek */
  0                /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  ,0               /* poll */
#endif
};

/************************************************************************************
 * Private Functions
 ************************************************************************************/

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

static void addentropy(const uint32_t *buf, size_t n)
{
  uint32_t rotate, w, i;

  while (n-- > 0)
    {
      rotate = g_rngdev.rd_rotate;
      w = ROTL_32(*buf, rotate);
      i = g_rngdev.rd_addptr = (g_rngdev.rd_addptr - 1) & POOL_MASK;

      /* Normal round, we add 7 bits of rotation to the pool.
       * At the beginning of the pool, we add extra 7 bits
       * rotation, in order for successive passes spread the
       * input bits across the pool evenly.
       */

      g_rngdev.rd_rotate = (rotate + (i ? 7 : 14)) & 31;

      /* XOR pool contents corresponding to polynomial terms */

      w ^= entropy_pool[(i + pool_stir[1]) & POOL_MASK];
      w ^= entropy_pool[(i + pool_stir[2]) & POOL_MASK];
      w ^= entropy_pool[(i + pool_stir[3]) & POOL_MASK];
      w ^= entropy_pool[(i + pool_stir[4]) & POOL_MASK];
      w ^= entropy_pool[(i + pool_stir[5]) & POOL_MASK];
      w ^= entropy_pool[i]; /* 2^POOL_SIZE */

      entropy_pool[i] = (w >> 3) ^ pool_twist[w & 7];
      buf++;

      g_rngdev.rd_newentr += sizeof(uint32_t);
   }
}

/****************************************************************************
 * Function: getentropy
 *
 * Description:
 *   Fill a small buffer with good-quality randomness. This is an
 *   internal interface for getting cryptographical keys or seeds
 *   from entropy pool.
 *
 *   Code is inspired by extract_entropy() function of OpenBSD kernel.
 *
 *   Note that this function cannot fail, other than by asserting.
 *
 *   Warning: In protected kernel builds, this interface MUST NOT be
 *   exported to userspace. This interface MUST NOT be used as a
 *   general-purpose random number generator!
 *
 * Parameters:
 *   bytes  - Buffer for returned random bytes
 *   nbytes - Number of bytes requested. nbytes must be less than or
 *            equal to 128.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void getentropy(void *bytes, size_t nbytes)
{
  enum { NMAXENTROPY = 128 };
  uint8_t buffer[16];
  unsigned int i;

  assert(nbytes <= NMAXENTROPY);

  add_sw_randomness(nbytes);

  while (nbytes > 0)
    {
      /* Hash the pool to get the output */

      md5_sum((void *)entropy_pool, sizeof(entropy_pool), buffer);

      /*
       * In case the hash function has some recognizable
       * output pattern, we fold it in half.
       */

      for (i = 0; i < sizeof(buffer) / 2; i++)
        {
          buffer[i] ^= buffer[sizeof(buffer) - 1 - i];
        }

      /* Copy data to destination buffer */

      i = nbytes > sizeof(buffer) / 2 ? sizeof(buffer) / 2 : nbytes;
      memcpy(bytes, buffer, i);
      nbytes -= i;
      bytes += i;

      /* Modify pool so next hash will produce different results. */

      add_sw_randomness(nbytes);
    }

  /* Add something back so repeated calls to this function
   * return different values.
   */

#ifdef CONFIG_ARCH_CHIP_STM32
  extern uint32_t idle_wastecounter;
  addentropy(&idle_wastecounter, 1);
#else
  /* TODO: Why is there no up_waste() for other archs? */
  uint32_t bogus = ENXIO ^ nbytes;
  addentropy(&bogus, 1);
#endif

  explicit_bzero(buffer, sizeof(buffer));
}

/* The spectrandom random number generation algoritm.
 *
 * Design uses Speck64/128 block cipher in CTR mode to generate a long
 * stream of randomness from a single 64-bit counter + secret key that
 * we get from entropy pool.
 *
 * Warning: Underlying cipher was designed by spooks [1], so beware! Also
 * this design has not been vetted by experts, so do not use if security is
 * paramount.
 *
 * This generator does not attempt to provide "prediction resistance"
 * against attackers who saw parts or whole of entropy pool in past, on the
 * theory that damage has already happened. Some people like RNGs such as
 * Yarrow or Fortuna because those are specifically designed to recover from
 * state compromise. There are heaps of academic papers that are convinced that
 * any RNG which doesn't worry about this theoretical attack is Obviously Broken
 * and Flawed. We leave it to academics to figure out, how to recover after
 * your long-lived crypto keys, that you generated from the compromised state,
 * are leaked out.
 *
 * References:
 *
 * [1] R. Beaulieu, D. Shors, J. Smith, S. Streatman-Clark, B. Weeks, and L. Wingers.
 *     The SIMON and SPECK families of lightweight block ciphers. Cryptology ePrint Archive,
 *     Report 2013/404, June 19, 2013. http://eprint.iacr.org/2013/404
 */

enum { NROUND = 27 };

typedef struct speck_ctx_s {
  uint32_t k[NROUND];
  uint32_t reseed_ctr;
  uint64_t ctr;
} speck_ctx_t;

static speck_ctx_t g_ctx;

static inline void schedule(speck_ctx_t *ctx, uint32_t *l)
{
  int i;
  uint32_t *k = ctx->k;

  for (i = 0; i < NROUND-1; i++)
    {
      l[i+3] = (k[i] + ROTR_32(l[i], 8)) ^ i;
      k[i+1] = ROTL_32(k[i], 3) ^ l[i+3];
    }
}

static void speck_setupkey(speck_ctx_t *ctx, uint32_t *key)
{
  uint32_t l[NROUND+2];

  l[2] = key[0];
  l[1] = key[1];
  l[0] = key[2];
  ctx->k[0] = key[3];
  schedule(ctx, l);
  explicit_bzero(l, sizeof(l));
}

static void speck_reseed(speck_ctx_t *ctx)
{
  uint32_t k[4];

  getentropy(k, sizeof(k));
  speck_setupkey(ctx, k);
  explicit_bzero(k, sizeof(k));
  ctx->reseed_ctr = 0;
}

static inline void speck_encrypt(speck_ctx_t *ctx, uint32_t *pt, uint32_t *ct)
{
  int i;
  ct[0] = pt[0];
  ct[1] = pt[1];

  for (i = 0; i < NROUND; i++)
    {
      ct[0] = (ROTR_32(ct[0], 8) + ct[1]) ^ ctx->k[i];
      ct[1] = ROTL_32(ct[1], 3) ^ ct[0];
    }
}

#ifdef CONFIG_CRYPTO_SPECK_DECRYPT
static inline void speck_decrypt(speck_ctx_t *ctx, uint32_t *pt, uint32_t *ct)
{
  int i;
  ct[0] = pt[0];
  ct[1] = pt[1];

  for (i = 0; i < NROUND; i++)
    {
      ct[1] = ROTR_32(ct[0] ^ ct[1], 3);
      ct[0] = ROTL_32((ct[0] ^ ctx->k[(NROUND-1)-i]) - ct[1], 8);
    }
}
#endif

static void speck_counter(speck_ctx_t *ctx, uint32_t *ct)
{
  /* Change key after 2**21 blocks. This allows doing q=2**10 encryptions
   * of L=2**11 counter values so error term (attacker's advantage) under
   * chosen-plaintext attack q*q*L/(2**64) = 2**(-32) is negligible.
   * Thus application that directly xors our output with plaintext is
   * CPA-secure, further assuming keystream is started from random location
   * where attacker does not know corresponding counter value (counter
   * has initial value 0 in boot).
   *
   * NIST SP 800-90A, Rev 1 allows 2**32 calls to 3-key TDES CTR_DRBG,
   * so ours is likely a conservative choise.
   *
   * REVISIT: Use speck128/128 instead and never worry about reseeding.
   */

  if (ctx->reseed_ctr > 2097152)
    {
      speck_reseed(ctx);
    }

  speck_encrypt(ctx, (uint32_t []) { ctx->ctr >> 32, (uint32_t)ctx->ctr }, ct);
  ctx->ctr++;
  ctx->reseed_ctr++;
}

static void speckrandom_buf_internal(void *bytes, size_t nbytes)
{
  static bool initialized = false;

  if (!initialized)
    {
      speck_reseed(&g_ctx);
      initialized = true;
    }

  if (g_rngdev.rd_newentr >= 1024)
    {
      /* Initial entropy is low. Reseed speck when we have accumulated more. */

      speck_reseed(&g_ctx);
      g_rngdev.rd_newentr = 0;
    }

  while (nbytes > 0)
    {
      uint32_t buf[2];
      size_t have;

      speck_counter(&g_ctx, buf);
      have = MIN(nbytes, sizeof(buf));
      memcpy(bytes, (uint8_t *)buf, have);
      bytes += have;
      nbytes -= have;
    }
}

#ifdef CONFIG_DEBUG
#define CONFIG_CRYPTO_SPECK_SELFTEST
#endif

#ifdef CONFIG_CRYPTO_SPECK_SELFTEST
static void speck_selftest(void)
{
    uint32_t pt[2] = { 0x3b726574, 0x7475432d };
    uint32_t ct[2] = { 0, 0 };
    speck_ctx_t ctx;

    speck_setupkey(&ctx, (uint32_t []) { 0x1b1a1918, 0x13121110, 0x0b0a0908, 0x03020100 });
    speck_encrypt(&ctx, pt, ct);
    DEBUGASSERT(ct[0] == 0x8c6fa548 && ct[1] == 0x454e028b);

#ifdef CONFIG_CRYPTO_SPECK_DECRYPT
    uint32_t pt2[2] = { 0 };
    speck_decrypt(&ctx, ct, pt2);
    DEBUGASSERT(pt2[0] == pt[0] && pt2[1] == pt[1]);
#endif
}
#endif /* CONFIG_CRYPTO_SPECK_SELFTEST */

static void speckrandom_init(void)
{
  lldbg("Initializing RNG\n");

  memset(&g_rngdev, 0, sizeof(struct rng_dev_s));
  sem_init(&g_rngdev.rd_devsem, 0, 1);

  /* We do not initialize real key here because this is called
   * quite early in boot and there may not be enough entropy.
   */

#ifdef CONFIG_CRYPTO_SPECK_SELFTEST
  speck_selftest();
#endif
}

/* Legacy device interface. */

static ssize_t rng_dev_read(struct file *filep, char *buffer, size_t n)
{
  /* We shall not return EINTR so that userspace has less
   * error handling to do, and therefore less chance to get
   * it wrong in some insecure fashion.
   */

  while (sem_wait(&g_rngdev.rd_devsem) != 0)
    {
      assert(errno == EINTR);
    }

  /* Luckily, the actual operation cannot fail. */

  speckrandom_buf_internal(buffer, n);

  sem_post(&g_rngdev.rd_devsem);
  return n;
}


/************************************************************************************
 * Public Functions
 ************************************************************************************/

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

void up_rngaddentropy(const uint32_t *buf, size_t n)
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

void getrandom(void *bytes, size_t nbytes)
{
  while (sem_wait(&g_rngdev.rd_devsem) != 0)
    {
      assert(errno == EINTR);
    }

  speckrandom_buf_internal(bytes, nbytes);
  sem_post(&g_rngdev.rd_devsem);
}

/****************************************************************************
 * Function: up_rnginitialize
 ****************************************************************************/

void up_rnginitialize(void)
{
  speckrandom_init();
#ifdef CONFIG_BOARD_INITRNGSEED
  board_init_rndseed();
#endif
  register_driver("/dev/random", &g_rngops, 0444, NULL);
}

