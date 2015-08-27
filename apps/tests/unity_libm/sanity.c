/****************************************************************************
 * apps/tests/unity_libm/sanity.c
 * Verify math library functions against sanity vectors
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
 *   Author: Roman Saveljev <roman.saveljev@haltian.com>
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
#include "mtest.h"

#include <nuttx/testing/unity_fixture.h>
#include <stdint.h>
#include <stdio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
 * Public Functions
 ****************************************************************************/

TEST_GROUP(Sanity);

/****************************************************************************
 * Name: Sanity test group setup
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
TEST_SETUP(Sanity)
{
}

/****************************************************************************
 * Name: Sanity test group tear down
 *
 * Description:
 *   Tear down function executed after each testcase in this test group
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
TEST_TEAR_DOWN(Sanity)
{
}

/****************************************************************************
 * Name: Acos
 *
 * Description:
 *   Verify acos() function
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

TEST(Sanity, Acos)
{
  static struct d_d t[] =
    {
#include "sanity/acos.h"
    };
  double y;
  float d;
  int i;
  struct d_d *p;

  for (i = 0; i < sizeof(t) / sizeof(*t); i++)
    {
      p = t + i;

      if (p->r < 0)
        {
          continue;
        }
      y = acos (p->x);
      d = ulperr (y, p->y, p->dy);
      if (!checkulp (d, p->r))
        {
          char buffer[128];
          snprintf (buffer, sizeof(buffer),
                    "%s:%d: %s acos(%.8g) want %.8g got %.8g ulperr %.3f = %.8g + %.8g",
                    p->file, p->line, rstr (p->r), p->x, p->y, y, d, d - p->dy,
                    p->dy);
          TEST_FAIL_MESSAGE(buffer);
        }
    }
}

/****************************************************************************
 * Name: Acosh
 *
 * Description:
 *   Verify acosh() function
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
TEST(Sanity, Acosh)
{
  static struct d_d t[] =
    {
#include "sanity/acosh.h"
    };
  double y;
  float d;
  int i;
  struct d_d *p;

  for (i = 0; i < sizeof(t) / sizeof(*t); i++)
    {
      p = t + i;

      if (p->r < 0)
        {
          continue;
        }
      y = acosh (p->x);
      d = ulperr (y, p->y, p->dy);
      if (!checkulp (d, p->r))
        {
          char buffer[128];
          snprintf (buffer, sizeof(buffer),
                    "%s:%d: %s acosh(%.8g) want %.8g got %.8g ulperr %.3f = %.8g + %.8g",
                    p->file, p->line, rstr (p->r), p->x, p->y, y, d, d - p->dy,
                    p->dy);
          TEST_FAIL_MESSAGE(buffer);
        }
    }
}

/****************************************************************************
 * Name: Asin
 *
 * Description:
 *   Verify asin() function
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
TEST(Sanity, Asin)
{
  static struct d_d t[] =
    {
#include "sanity/asin.h"
    };
  double y;
  float d;
  int i;
  struct d_d *p;

  for (i = 0; i < sizeof(t) / sizeof(*t); i++)
    {
      p = t + i;

      if (p->r < 0)
        {
          continue;
        }
      y = asin (p->x);
      d = ulperr (y, p->y, p->dy);
      if (!checkulp (d, p->r))
        {
          char buffer[128];
          snprintf (buffer, sizeof(buffer),
                    "%s:%d: %s asin(%.8g) want %.8g got %.8g ulperr %.3f = %.8g + %.8g",
                    p->file, p->line, rstr (p->r), p->x, p->y, y, d, d - p->dy,
                    p->dy);
          TEST_FAIL_MESSAGE(buffer);
        }
    }
}

/****************************************************************************
 * Name: Asinh
 *
 * Description:
 *   Verify asinh() function
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
TEST(Sanity, Asinh)
{
  static struct d_d t[] =
    {
#include "sanity/asinh.h"
    };
  double y;
  float d;
  int i;
  struct d_d *p;

  for (i = 0; i < sizeof(t) / sizeof(*t); i++)
    {
      p = t + i;

      if (p->r < 0)
        {
          continue;
        }
      y = asinh (p->x);
      d = ulperr (y, p->y, p->dy);
      if (!checkulp (d, p->r))
        {
          char buffer[128];
          snprintf (buffer, sizeof(buffer),
                    "%s:%d: %s asinh(%.8g) want %.8g got %.8g ulperr %.3f = %.8g + %.8g",
                    p->file, p->line, rstr (p->r), p->x, p->y, y, d, d - p->dy,
                    p->dy);
          TEST_FAIL_MESSAGE(buffer);
        }
    }
}

/****************************************************************************
 * Name: Atan
 *
 * Description:
 *   Verify atan() function
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
TEST(Sanity, Atan)
{
  static struct d_d t[] =
    {
#include "sanity/atan.h"
    };
  double y;
  float d;
  int i;
  struct d_d *p;

  for (i = 0; i < sizeof(t) / sizeof(*t); i++)
    {
      p = t + i;

      if (p->r < 0)
        {
          continue;
        }
      y = atan (p->x);
      d = ulperr (y, p->y, p->dy);
      if (!checkulp (d, p->r))
        {
          char buffer[128];
          snprintf (buffer, sizeof(buffer),
                    "%s:%d: %s atan(%.8g) want %.8g got %.8g ulperr %.3f = %.8g + %.8g",
                    p->file, p->line, rstr (p->r), p->x, p->y, y, d, d - p->dy,
                    p->dy);
          TEST_FAIL_MESSAGE(buffer);
        }
    }
}

/****************************************************************************
 * Name: Atan2
 *
 * Description:
 *   Verify atan2() function
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
TEST(Sanity, Atan2)
{
  static struct dd_d t[] =
    {
#include "sanity/atan2.h"
    };
  double y;
  float d;
  int i;
  struct dd_d *p;

  for (i = 0; i < sizeof(t) / sizeof(*t); i++)
    {
      p = t + i;

      if (p->r < 0)
        {
          continue;
        }
      y = atan2 (p->x, p->x2);
      d = ulperr (y, p->y, p->dy);
      if (!checkulp(d, p->r))
        {
          char buffer[128];
          snprintf (buffer, sizeof(buffer),
                    "%s:%d: %s atan2(%.8g,%.8g) want %.8g got %.8g ulperr %.3f = %.8g + %.8g",
                    p->file, p->line, rstr (p->r), p->x, p->x2, p->y, y, d, d - p->dy,
                    p->dy);
          TEST_FAIL_MESSAGE(buffer);
        }
    }
}

/****************************************************************************
 * Name: Atanh
 *
 * Description:
 *   Verify atanh() function
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
TEST(Sanity, Atanh)
{
  static struct d_d t[] =
    {
#include "sanity/atanh.h"
    };
  double y;
  float d;
  int i;
  struct d_d *p;

  for (i = 0; i < sizeof(t) / sizeof(*t); i++)
    {
      p = t + i;

      if (p->r < 0)
        {
          continue;
        }
      y = atanh (p->x);
      d = ulperr (y, p->y, p->dy);
      if (!checkulp (d, p->r))
        {
          char buffer[128];
          snprintf (buffer, sizeof(buffer),
                    "%s:%d: %s atanh(%.8g) want %.8g got %.8g ulperr %.3f = %.8g + %.8g",
                    p->file, p->line, rstr (p->r), p->x, p->y, y, d, d - p->dy,
                    p->dy);
          TEST_FAIL_MESSAGE(buffer);
        }
    }
}

/****************************************************************************
 * Name: Cbrt
 *
 * Description:
 *   Verify cbrt() function
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
TEST(Sanity, Cbrt)
{
  static struct d_d t[] =
    {
#include "sanity/cbrt.h"
    };
  double y;
  float d;
  int i;
  struct d_d *p;

  for (i = 0; i < sizeof(t) / sizeof(*t); i++)
    {
      p = t + i;

      if (p->r < 0)
        {
          continue;
        }
      y = cbrt (p->x);
      d = ulperr (y, p->y, p->dy);
      if (!checkulp (d, p->r))
        {
          char buffer[128];
          snprintf (buffer, sizeof(buffer),
                    "%s:%d: %s cbrt(%.8g) want %.8g got %.8g ulperr %.3f = %.8g + %.8g",
                    p->file, p->line, rstr (p->r), p->x, p->y, y, d, d - p->dy,
                    p->dy);
          TEST_FAIL_MESSAGE(buffer);
        }
    }
}

/****************************************************************************
 * Name: Ceil
 *
 * Description:
 *   Verify ceil() function
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
TEST(Sanity, Ceil)
{
  static struct d_d t[] =
    {
#include "sanity/ceil.h"
    };
  double y;
  float d;
  int i;
  struct d_d *p;

  for (i = 0; i < sizeof(t) / sizeof(*t); i++)
    {
      p = t + i;

      if (p->r < 0)
        {
          continue;
        }
      y = ceil (p->x);
      d = ulperr (y, p->y, p->dy);
      if (!checkcr(y, p->y, p->r))
        {
          char buffer[128];
          snprintf (buffer, sizeof(buffer),
                    "%s:%d: %s ceil(%.8g) want %.8g got %.8g ulperr %.3f = %.8g + %.8g",
                    p->file, p->line, rstr (p->r), p->x, p->y, y, d, d - p->dy,
                    p->dy);
          TEST_FAIL_MESSAGE(buffer);
        }
    }
}

/****************************************************************************
 * Name: Copysign
 *
 * Description:
 *   Verify copysign() function
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
TEST(Sanity, Copysign)
{
  static struct dd_d t[] =
    {
#include "sanity/copysign.h"
    };
  double y;
  float d;
  int i;
  struct dd_d *p;

  for (i = 0; i < sizeof(t) / sizeof(*t); i++)
    {
      p = t + i;

      if (p->r < 0)
        {
          continue;
        }
      y = copysign (p->x, p->x2);
      d = ulperr (y, p->y, p->dy);
      if (!checkcr (y, p->y, p->r))
        {
          char buffer[128];
          snprintf (buffer, sizeof(buffer),
                    "%s:%d: %s copysign(%.8g,%.8g) want %.8g got %.8g ulperr %.3f = %.8g + %.8g",
                    p->file, p->line, rstr (p->r), p->x, p->x2, p->y, y, d, d - p->dy,
                    p->dy);
          TEST_FAIL_MESSAGE(buffer);
        }
    }
}

/****************************************************************************
 * Name: Cos
 *
 * Description:
 *   Verify cos() function
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
TEST(Sanity, Cos)
{
  static struct d_d t[] =
    {
#include "sanity/cos.h"
    };
  double y;
  float d;
  int i;
  struct d_d *p;

  for (i = 0; i < sizeof(t) / sizeof(*t); i++)
    {
      p = t + i;

      if (p->r < 0)
        {
          continue;
        }
      y = cos (p->x);
      d = ulperr (y, p->y, p->dy);
      if (!checkulp (d, p->r))
        {
          char buffer[128];
          snprintf (buffer, sizeof(buffer),
                    "%s:%d: %s cos(%.8g) want %.8g got %.8g ulperr %.3f = %.8g + %.8g",
                    p->file, p->line, rstr (p->r), p->x, p->y, y, d, d - p->dy,
                    p->dy);
          TEST_FAIL_MESSAGE(buffer);
        }
    }
}

/****************************************************************************
 * Name: Cosh
 *
 * Description:
 *   Verify cosh() function
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
TEST(Sanity, Cosh)
{
  static struct d_d t[] =
    {
#include "sanity/cosh.h"
    };
  double y;
  float d;
  int i;
  struct d_d *p;

  for (i = 0; i < sizeof(t) / sizeof(*t); i++)
    {
      p = t + i;

      if (p->r < 0)
        {
          continue;
        }
      y = cosh (p->x);
      d = ulperr (y, p->y, p->dy);
      if (!checkulp (d, p->r))
        {
          char buffer[128];
          snprintf (buffer, sizeof(buffer),
                    "%s:%d: %s cosh(%.8g) want %.8g got %.8g ulperr %.3f = %.8g + %.8g",
                    p->file, p->line, rstr (p->r), p->x, p->y, y, d, d - p->dy,
                    p->dy);
          TEST_FAIL_MESSAGE(buffer);
        }
    }
}

/****************************************************************************
 * Name: Drem
 *
 * Description:
 *   Verify drem() function
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
TEST(Sanity, Drem)
{
  static struct dd_d t[] =
    {
#include "sanity/remainder.h"
    };
  double y;
  float d;
  int i;
  struct dd_d *p;

  for (i = 0; i < sizeof(t) / sizeof(*t); i++)
    {
      p = t + i;

      if (p->r < 0)
        {
          continue;
        }
      y = drem (p->x, p->x2);
      d = ulperr (y, p->y, p->dy);
      if (!checkcr(y, p->y, p->r))
        {
          char buffer[128];
          snprintf (buffer, sizeof(buffer),
                    "%s:%d: %s drem(%.8g,%.8g) want %.8g got %.8g ulperr %.3f = %.8g + %.8g",
                    p->file, p->line, rstr (p->r), p->x, p->x2, p->y, y, d, d - p->dy,
                    p->dy);
          TEST_FAIL_MESSAGE(buffer);
        }
    }
}

/****************************************************************************
 * Name: Erf
 *
 * Description:
 *   Verify erf() function
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
TEST(Sanity, Erf)
{
  static struct d_d t[] =
    {
#include "sanity/erf.h"
    };
  double y;
  float d;
  int i;
  struct d_d *p;

  for (i = 0; i < sizeof(t) / sizeof(*t); i++)
    {
      p = t + i;

      if (p->r < 0)
        {
          continue;
        }
      y = erf(p->x);
      d = ulperr (y, p->y, p->dy);
      if (!checkulp (d, p->r))
        {
          char buffer[128];
          snprintf (buffer, sizeof(buffer),
                    "%s:%d: %s erf(%.8g) want %.8g got %.8g ulperr %.3f = %.8g + %.8g",
                    p->file, p->line, rstr (p->r), p->x, p->y, y, d, d - p->dy,
                    p->dy);
          TEST_FAIL_MESSAGE(buffer);
        }
    }
}

/****************************************************************************
 * Name: Erfc
 *
 * Description:
 *   Verify erfc() function
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
TEST(Sanity, Erfc)
{
  static struct d_d t[] =
    {
#include "sanity/erfc.h"
    };
  double y;
  float d;
  int i;
  struct d_d *p;

  for (i = 0; i < sizeof(t) / sizeof(*t); i++)
    {
      p = t + i;

      if (p->r < 0)
        {
          continue;
        }
      y = erfc(p->x);
      d = ulperr (y, p->y, p->dy);
      if (!checkulp (d, p->r))
        {
          char buffer[128];
          snprintf (buffer, sizeof(buffer),
                    "%s:%d: %s erfc(%.8g) want %.8g got %.8g ulperr %.3f = %.8g + %.8g",
                    p->file, p->line, rstr (p->r), p->x, p->y, y, d, d - p->dy,
                    p->dy);
          TEST_FAIL_MESSAGE(buffer);
        }
    }
}

/****************************************************************************
 * Name: Exp
 *
 * Description:
 *   Verify exp() function
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
TEST(Sanity, Exp)
{
  static struct d_d t[] =
    {
#include "sanity/exp.h"
    };
  double y;
  float d;
  int i;
  struct d_d *p;

  for (i = 0; i < sizeof(t) / sizeof(*t); i++)
    {
      p = t + i;

      if (p->r < 0)
        {
          continue;
        }
      y = exp(p->x);
      d = ulperr (y, p->y, p->dy);
      if (!checkulp (d, p->r))
        {
          char buffer[128];
          snprintf (buffer, sizeof(buffer),
                    "%s:%d: %s exp(%.8g) want %.8g got %.8g ulperr %.3f = %.8g + %.8g",
                    p->file, p->line, rstr (p->r), p->x, p->y, y, d, d - p->dy,
                    p->dy);
          TEST_FAIL_MESSAGE(buffer);
        }
    }
}

/****************************************************************************
 * Name: Exp2
 *
 * Description:
 *   Verify exp2() function
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
TEST(Sanity, Exp2)
{
  static struct d_d t[] =
    {
#include "sanity/exp2.h"
    };
  double y;
  float d;
  int i;
  struct d_d *p;

  for (i = 0; i < sizeof(t) / sizeof(*t); i++)
    {
      p = t + i;

      if (p->r < 0)
        {
          continue;
        }
      y = exp2(p->x);
      d = ulperr (y, p->y, p->dy);
      if (!checkulp (d, p->r))
        {
          char buffer[128];
          snprintf (buffer, sizeof(buffer),
                    "%s:%d: %s exp2(%.8g) want %.8g got %.8g ulperr %.3f = %.8g + %.8g",
                    p->file, p->line, rstr (p->r), p->x, p->y, y, d, d - p->dy,
                    p->dy);
          TEST_FAIL_MESSAGE(buffer);
        }
    }
}

/****************************************************************************
 * Name: Expm1
 *
 * Description:
 *   Verify expm1() function
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
TEST(Sanity, Expm1)
{
  static struct d_d t[] =
    {
#include "sanity/expm1.h"
    };
  double y;
  float d;
  int i;
  struct d_d *p;

  for (i = 0; i < sizeof(t) / sizeof(*t); i++)
    {
      p = t + i;

      if (p->r < 0)
        {
          continue;
        }
      y = expm1(p->x);
      d = ulperr (y, p->y, p->dy);
      if (!checkulp (d, p->r))
        {
          char buffer[128];
          snprintf (buffer, sizeof(buffer),
                    "%s:%d: %s expm1(%.8g) want %.8g got %.8g ulperr %.3f = %.8g + %.8g",
                    p->file, p->line, rstr (p->r), p->x, p->y, y, d, d - p->dy,
                    p->dy);
          TEST_FAIL_MESSAGE(buffer);
        }
    }
}

/****************************************************************************
 * Name: Fabs
 *
 * Description:
 *   Verify fabs() function
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
TEST(Sanity, Fabs)
{
  static struct d_d t[] =
    {
#include "sanity/fabs.h"
    };
  double y;
  float d;
  int i;
  struct d_d *p;

  for (i = 0; i < sizeof(t) / sizeof(*t); i++)
    {
      p = t + i;

      if (p->r < 0)
        {
          continue;
        }
      y = fabs (p->x);
      d = ulperr (y, p->y, p->dy);
      if (!checkcr(y, p->y, p->r))
        {
          char buffer[128];
          snprintf (buffer, sizeof(buffer),
                    "%s:%d: %s ceil(%.8g) want %.8g got %.8g ulperr %.3f = %.8g + %.8g",
                    p->file, p->line, rstr (p->r), p->x, p->y, y, d, d - p->dy,
                    p->dy);
          TEST_FAIL_MESSAGE(buffer);
        }
    }
}

/****************************************************************************
 * Name: Fdim
 *
 * Description:
 *   Verify fdim() function
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
TEST(Sanity, Fdim)
{
  static struct dd_d t[] =
    {
#include "sanity/fdim.h"
    };
  double y;
  float d;
  int i;
  struct dd_d *p;

  for (i = 0; i < sizeof(t) / sizeof(*t); i++)
    {
      p = t + i;

      if (p->r < 0)
        {
          continue;
        }
      y = fdim (p->x, p->x2);
      d = ulperr (y, p->y, p->dy);
      if (!checkcr(y, p->y, p->r))
        {
          char buffer[128];
          snprintf (buffer, sizeof(buffer),
                    "%s:%d: %s fdim(%.8g,%.8g) want %.8g got %.8g ulperr %.3f = %.8g + %.8g",
                    p->file, p->line, rstr (p->r), p->x, p->x2, p->y, y, d, d - p->dy,
                    p->dy);
          TEST_FAIL_MESSAGE(buffer);
        }
    }
}

/****************************************************************************
 * Name: Floor
 *
 * Description:
 *   Verify floor() function
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
TEST(Sanity, Floor)
{
  static struct d_d t[] =
    {
#include "sanity/floor.h"
    };
  double y;
  float d;
  int i;
  struct d_d *p;

  for (i = 0; i < sizeof(t) / sizeof(*t); i++)
    {
      p = t + i;

      if (p->r < 0)
        {
          continue;
        }
      y = floor (p->x);
      d = ulperr (y, p->y, p->dy);
      if (!checkulp (d, p->r))
        {
          char buffer[128];
          snprintf (buffer, sizeof(buffer),
                    "%s:%d: %s floor(%.8g) want %.8g got %.8g ulperr %.3f = %.8g + %.8g",
                    p->file, p->line, rstr (p->r), p->x, p->y, y, d, d - p->dy,
                    p->dy);
          TEST_FAIL_MESSAGE(buffer);
        }
    }
}

/****************************************************************************
 * Name: Fmax
 *
 * Description:
 *   Verify fmax() function
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
TEST(Sanity, Fmax)
{
  static struct dd_d t[] =
    {
#include "sanity/fmax.h"
    };
  double y;
  float d;
  int i;
  struct dd_d *p;

  for (i = 0; i < sizeof(t) / sizeof(*t); i++)
    {
      p = t + i;

      if (p->r < 0)
        {
          continue;
        }
      y = fmax (p->x, p->x2);
      d = ulperr (y, p->y, p->dy);
      if (!checkcr(y, p->y, p->r))
        {
          char buffer[128];
          snprintf (buffer, sizeof(buffer),
                    "%s:%d: %s fmax(%.8g,%.8g) want %.8g got %.8g ulperr %.3f = %.8g + %.8g",
                    p->file, p->line, rstr (p->r), p->x, p->x2, p->y, y, d, d - p->dy,
                    p->dy);
          TEST_FAIL_MESSAGE(buffer);
        }
    }
}

/****************************************************************************
 * Name: Fmin
 *
 * Description:
 *   Verify fmin() function
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
TEST(Sanity, Fmin)
{
  static struct dd_d t[] =
    {
#include "sanity/fmin.h"
    };
  double y;
  float d;
  int i;
  struct dd_d *p;

  for (i = 0; i < sizeof(t) / sizeof(*t); i++)
    {
      p = t + i;

      if (p->r < 0)
        {
          continue;
        }
      y = fmin (p->x, p->x2);
      d = ulperr (y, p->y, p->dy);
      if (!checkcr(y, p->y, p->r))
        {
          char buffer[128];
          snprintf (buffer, sizeof(buffer),
                    "%s:%d: %s fmin(%.8g,%.8g) want %.8g got %.8g ulperr %.3f = %.8g + %.8g",
                    p->file, p->line, rstr (p->r), p->x, p->x2, p->y, y, d, d - p->dy,
                    p->dy);
          TEST_FAIL_MESSAGE(buffer);
        }
    }
}

/****************************************************************************
 * Name: Fmod
 *
 * Description:
 *   Verify fmod() function
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
TEST(Sanity, Fmod)
{
  static struct dd_d t[] =
    {
#include "sanity/fmod.h"
    };
  double y;
  float d;
  int i;
  struct dd_d *p;

  for (i = 0; i < sizeof(t) / sizeof(*t); i++)
    {
      p = t + i;

      if (p->r < 0)
        {
          continue;
        }
      y = fmod (p->x, p->x2);
      d = ulperr (y, p->y, p->dy);
      if (!checkcr(y, p->y, p->r))
        {
          char buffer[128];
          snprintf (buffer, sizeof(buffer),
                    "%s:%d: %s fmod(%.8g,%.8g) want %.8g got %.8g ulperr %.3f = %.8g + %.8g",
                    p->file, p->line, rstr (p->r), p->x, p->x2, p->y, y, d, d - p->dy,
                    p->dy);
          TEST_FAIL_MESSAGE(buffer);
        }
    }
}

/****************************************************************************
 * Name: Frexp
 *
 * Description:
 *   Verify frexp() function
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
TEST(Sanity, Frexp)
{
  static struct d_di t[] =
    {
#include "sanity/frexp.h"
    };
  int yi;
  double y;
  float d;
  int i;
  struct d_di *p;

  for (i = 0; i < sizeof(t) / sizeof(*t); i++)
    {
      p = t + i;

      if (p->r < 0)
        {
          continue;
        }
      y = frexp (p->x, &yi);
      d = ulperr (y, p->y, p->dy);
      if (!checkcr(y, p->y, p->r) || (isfinite(p->x) && yi != p->i))
        {
          char buffer[128];
          snprintf (buffer, sizeof(buffer),
                    "%s:%d: %s frexp(%.8g) want %.8g,%lld got %.8g,%lld ulperr %.3f = %.8g + %.8g",
                    p->file, p->line, rstr (p->r), p->x, p->y, p->i, y, yi, d, d - p->dy,
                    p->dy);
          TEST_FAIL_MESSAGE(buffer);
        }
    }
}

/****************************************************************************
 * Name: Hypot
 *
 * Description:
 *   Verify hypot() function
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
TEST(Sanity, Hypot)
{
  static struct dd_d t[] =
    {
#include "sanity/hypot.h"
    };
  double y;
  float d;
  int i;
  struct dd_d *p;

  for (i = 0; i < sizeof(t) / sizeof(*t); i++)
    {
      p = t + i;

      if (p->r < 0)
        {
          continue;
        }
      y = hypot (p->x, p->x2);
      d = ulperr (y, p->y, p->dy);
      if (!checkulp(d, p->r) || (p->r == RN && fabs(d) >= 1.0))
        {
          char buffer[128];
          snprintf (buffer, sizeof(buffer),
                    "%s:%d: %s hypot(%.8g,%.8g) want %.8g got %.8g ulperr %.3f = %.8g + %.8g",
                    p->file, p->line, rstr (p->r), p->x, p->x2, p->y, y, d, d - p->dy,
                    p->dy);
          TEST_FAIL_MESSAGE(buffer);
        }
    }
}

/****************************************************************************
 * Name: Ilogb
 *
 * Description:
 *   Verify ilogb() function
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
TEST(Sanity, Ilogb)
{
  static struct d_i t[] =
    {
#include "sanity/ilogb.h"
    };
  long long yi;
  int i;
  struct d_i *p;

  for (i = 0; i < sizeof(t) / sizeof(*t); i++)
    {
      p = t + i;

      if (p->r < 0)
        {
          continue;
        }
      yi = ilogb (p->x);
      if (yi != p->i)
        {
          char buffer[128];
          printf ("%s:%d: %s ilogb(%.8g) want %lld got %lld", p->file, p->line,
                  rstr (p->r), p->x, p->i, yi);
          TEST_FAIL_MESSAGE(buffer);
        }
    }
}

/****************************************************************************
 * Name: J0
 *
 * Description:
 *   Verify j0() function
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
TEST(Sanity, J0)
{
  static struct d_d t[] =
    {
#include "sanity/j0.h"
    };
  double y;
  float d;
  int i;
  struct d_d *p;

  for (i = 0; i < sizeof(t) / sizeof(*t); i++)
    {
      p = t + i;

      if (p->r < 0)
        {
          continue;
        }
      y = j0 (p->x);
      d = ulperr (y, p->y, p->dy);
      if (!checkulp (d, p->r))
        {
          char buffer[128];
          snprintf (buffer, sizeof(buffer),
                    "%s:%d: %s j0(%.8g) want %.8g got %.8g ulperr %.3f = %.8g + %.8g",
                    p->file, p->line, rstr (p->r), p->x, p->y, y, d, d - p->dy,
                    p->dy);
          TEST_FAIL_MESSAGE(buffer);
        }
    }
}

/****************************************************************************
 * Name: J1
 *
 * Description:
 *   Verify j1() function
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
TEST(Sanity, J1)
{
  static struct d_d t[] =
    {
#include "sanity/j1.h"
    };
  double y;
  float d;
  int i;
  struct d_d *p;

  for (i = 0; i < sizeof(t) / sizeof(*t); i++)
    {
      p = t + i;

      if (p->r < 0)
        {
          continue;
        }
      y = j1 (p->x);
      d = ulperr (y, p->y, p->dy);
      if (!checkulp (d, p->r))
        {
          char buffer[128];
          snprintf (buffer, sizeof(buffer),
                    "%s:%d: %s j1(%.8g) want %.8g got %.8g ulperr %.3f = %.8g + %.8g",
                    p->file, p->line, rstr (p->r), p->x, p->y, y, d, d - p->dy,
                    p->dy);
          TEST_FAIL_MESSAGE(buffer);
        }
    }
}

/****************************************************************************
 * Name: Sin
 *
 * Description:
 *   Verify sin() function
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
TEST(Sanity, Sin)
{
  static struct d_d t[] =
    {
#include "sanity/sin.h"
    };
  double y;
  float d;
  int i;
  struct d_d *p;

  for (i = 0; i < sizeof(t) / sizeof(*t); i++)
    {
      p = t + i;

      if (p->r < 0)
        {
          continue;
        }
      y = sin (p->x);
      d = ulperr (y, p->y, p->dy);
      if (!checkulp (d, p->r))
        {
          char buffer[128];
          snprintf (buffer, sizeof(buffer),
                    "%s:%d: %s sin(%.8g) want %.8g got %.8g ulperr %.3f = %.8g + %.8g",
                    p->file, p->line, rstr (p->r), p->x, p->y, y, d, d - p->dy,
                    p->dy);
          TEST_FAIL_MESSAGE(buffer);
        }
    }
}
