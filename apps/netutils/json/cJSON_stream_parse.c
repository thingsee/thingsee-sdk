/****************************************************************************
 * apps/netutils/json/cJSON_stream_parse.c
 *
 * This file is a part of NuttX:
 *
 *   Copyright © 2011 Gregory Nutt. All rights reserved.
 *   Copyright © 2015 Jussi Kivilinna <jussi.kivilinna@iki.fi>
 *
 * And derives from the cJSON Project which has an MIT license:
 *
 *   Copyright (c) 2009 Dave Gamble
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <float.h>
#include <limits.h>
#include <ctype.h>
#include <unistd.h>
#include <errno.h>

#include <apps/netutils/cJSON.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FD_PARSER_CACHE_SIZE 256

#define cJSON_malloc malloc
#define cJSON_free free
#define cJSON_realloc realloc
#define cJSON_strdup strdup
#define cJSON_New_Item() ((cJSON *)calloc(1, sizeof(cJSON)))

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct cJSON_instream_s
{
  char (*getc_fn)(void *priv);
  void *fn_priv;
  int curr;
} cJSON_instream;

struct parse_fd_priv_s
{
  int fd;
  ssize_t max_readlen;
  size_t nread;
  char *buf;
  size_t buflen;
  size_t bufpos;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const unsigned char firstByteMark[7] =
  { 0x00, 0x00, 0xc0, 0xe0, 0xf0, 0xf8, 0xfc };

/****************************************************************************
 * Private Prototypes
 ****************************************************************************/

static cJSON_instream *stream_parse_value(cJSON *item, cJSON_instream *in);
static cJSON_instream *stream_parse_array(cJSON *item, cJSON_instream *in);
static cJSON_instream *stream_parse_object(cJSON *item, cJSON_instream *in);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline char stream_get(cJSON_instream *in)
{
  if (in->curr >= 0)
    {
      char curr = in->curr;
      in->curr = -1;
      return curr;
    }

  return in->getc_fn(in->fn_priv);
}

static inline char stream_peek(cJSON_instream *in)
{
  if (in->curr >= 0)
    {
      return in->curr;
    }

  in->curr = (unsigned int)in->getc_fn(in->fn_priv) & 0xFF;
  return in->curr;
}

/* Parse the input text to generate a number, and populate the result into item. */

static cJSON_instream *stream_parse_number(cJSON *item, cJSON_instream *in)
{
  double n = 0, sign = 1, scale = 0;
  int subscale = 0, signsubscale = 1;

  /* Has sign? */

  if (stream_peek(in) == '-')
    {
      sign = -1;
      (void)stream_get(in);
    }

  /* is zero */

  if (stream_peek(in) == '0')
    {
      (void)stream_get(in);

      if (stream_peek(in) != '.' && stream_peek(in) != 'e' &&
          stream_peek(in) != 'E')
        {
          n = sign * 0.0;
          item->valuedouble = n;
          item->valueint = (int)n;
          item->type = cJSON_Number;
          return in;
        }
    }

  /* Number? */

  if (stream_peek(in) >= '1' && stream_peek(in) <= '9')
    {
      do
        {
          n = (n * 10.0) + (stream_get(in) - '0');
        }
      while (stream_peek(in) >= '0' && stream_peek(in) <= '9');
    }

  /* Fractional part? */

  if (stream_peek(in) == '.')
    {
      (void)stream_get(in);

      if (stream_peek(in) >= '0' && stream_peek(in) <= '9')
        {
          do
            {
              n = (n * 10.0) + (stream_get(in) - '0'), scale--;
            }
          while (stream_peek(in)>= '0' && stream_peek(in)<= '9');
        }
    }

  /* Exponent? */

  if (stream_peek(in) == 'e' || stream_peek(in) == 'E')
    {
      (void)stream_get(in);

      if (stream_peek(in) == '+')
        {
          (void)stream_get(in);
        }

      /* With sign? */

      else if (stream_peek(in) == '-')
        {
          signsubscale = -1;
          (void)stream_get(in);
        }

      /* Number? */

      while (stream_peek(in) >= '0' && stream_peek(in) <= '9')
        {
          subscale = (subscale * 10) + (stream_get(in) - '0');
        }
    }

  /* number = +/- number.fraction * 10^+/-exponent */

  n = sign * n * pow(10.0, (scale + subscale * signsubscale));
  item->valuedouble = n;
  item->valueint = (int)n;
  item->type = cJSON_Number;
  return in;
}

static bool parse_realloc(char **out, int *outlen, char **out2, int inclen)
{
  char *newp;

  *outlen += inclen;
  newp = cJSON_realloc(*out, *outlen);
  if (!newp)
    {
      free(*out);
      *out = NULL;
      *out2 = NULL;
      *outlen = 0;
      return false;
    }

  *out2 += newp - *out;
  *out = newp;
  return true;
}

static int stream_parse_hex(cJSON_instream *in)
{
  char ascii = stream_peek(in);
  unsigned h;

  if (ascii >= '0' && ascii <= '9')
    {
      h = ascii - '0';
    }
  else if (ascii >= 'A' && ascii <= 'F')
    {
      h = 0xA + ascii - 'A';
    }
  else if (ascii >= 'a' && ascii <= 'f')
    {
      h = 0xa + ascii - 'a';
    }
  else
    {
      return -1;
    }

  (void)stream_get(in);
  return h;
}

static unsigned stream_parse_hex4(cJSON_instream *in)
{
  int val;
  unsigned h = 0;

  val = stream_parse_hex(in);
  if (val < 0)
    return 0;

  h += val;
  h <<= 4;

  val = stream_parse_hex(in);
  if (val < 0)
    return 0;

  h += val;
  h <<= 4;

  val = stream_parse_hex(in);
  if (val < 0)
    return 0;

  h += val;
  h <<= 4;

  val = stream_parse_hex(in);
  if (val < 0)
    return 0;

  h += val;
  return h;
}

/* Parse the input text into an unescaped cstring, and populate item. */

static cJSON_instream *stream_parse_string(cJSON *item, cJSON_instream *in)
{
  char *ptr2;
  char *out;
  int len;
  int outlen;
  unsigned uc;
  unsigned uc2;

  if (stream_peek(in) != '\"')
    {
      /* not a string! */

      return NULL;
    }

  outlen = 1;
  out = (char *)cJSON_malloc(outlen + 7);
  if (!out)
    {
      return NULL;
    }

  (void)stream_get(in);
  ptr2 = out;
  while (stream_peek(in) != '\"' && stream_peek(in))
    {
      if (stream_peek(in) != '\\')
        {
          if (!parse_realloc(&out, &outlen, &ptr2, 1))
            {
              /* not enough memory */

              return NULL;
            }

          *ptr2++ = stream_get(in);
        }
      else
        {
          char val;

          (void)stream_get(in);

next_escape:
          val = stream_peek(in);
          switch (val)
            {
            case 'b':
              val = '\b';
              break;

            case 'f':
              val = '\f';
              break;

            case 'n':
              val = '\n';
              break;

            case 'r':
              val = '\r';
              break;

            case 't':
              val = '\t';
              break;
            }

          (void)stream_get(in);

          switch (val)
            {
            default:
              if (!parse_realloc(&out, &outlen, &ptr2, 1))
                {
                  /* not enough memory */

                  return NULL;
                }

              *ptr2++ = val;
              break;

            case 'u':
              /* Transcode utf16 to utf8. */
              /* Get the unicode char. */

              uc = stream_parse_hex4(in);

              /* Check for invalid. */

              if ((uc >= 0xdc00 && uc <= 0xdfff) || uc == 0)
                {
                  break;
                }

              if (uc >= 0xd800 && uc <= 0xdbff) /* UTF16 surrogate pairs. */
                {
                  /* missing second-half of surrogate. */

                  if (stream_peek(in) != '\\')
                    {
                      break;
                    }

                  (void)stream_get(in);

                  if (stream_peek(in) != 'u')
                    {
                      goto next_escape;
                    }

                  (void)stream_get(in);

                  uc2 = stream_parse_hex4(in);
                  if (uc2 < 0xdc00 || uc2 > 0xdfff)
                    {
                      /* Invalid second-half of surrogate. */

                      break;
                    }

                  uc = 0x10000 | ((uc & 0x3ff) << 10) | (uc2 & 0x3ff);
                }

              len = 4;
              if (uc < 0x80)
                {
                  len = 1;
                }
              else if (uc < 0x800)
                {
                  len = 2;
                }
              else if (uc < 0x10000)
                {
                  len = 3;
                }

              if (!parse_realloc(&out, &outlen, &ptr2, len))
                {
                  /* not enough memory */

                  return NULL;
                }

              ptr2 += len;

              switch (len)
                {
                case 4:
                  *--ptr2 = ((uc | 0x80) & 0xbf);
                  uc >>= 6;
                  /* no break */
                case 3:
                  *--ptr2 = ((uc | 0x80) & 0xbf);
                  uc >>= 6;
                  /* no break */
                case 2:
                  *--ptr2 = ((uc | 0x80) & 0xbf);
                  uc >>= 6;
                  /* no break */
                case 1:
                  *--ptr2 = (uc | firstByteMark[len]);
                  break;
                }

              ptr2 += len;
              break;
            }
        }
    }

  *ptr2 = 0;

  if (stream_peek(in) == '\"')
    {
      (void)stream_get(in);
    }

  item->valuestring = out;
  item->type = cJSON_String;
  return in;
}

/* Utility to jump whitespace and cr/lf */

static cJSON_instream *skip(cJSON_instream *in)
{
  while (in && stream_peek(in) && (unsigned char)stream_peek(in) <= 32)
    {
      (void)stream_get(in);
    }

  return in;
}

/* Parser core - when encountering text, process appropriately. */

static cJSON_instream *stream_parse_value(cJSON *item, cJSON_instream *in)
{
  static const struct
  {
    const char *name;
    char type;
    char value;
  } match_strings[] =
    {
      { "null", cJSON_NULL, 0 },
      { "false", cJSON_False, 0 },
      { "true", cJSON_True, 1 },
      { }
    };
  int midx;

  if (!in)
    {
      /* Fail on null. */

      return NULL;
    }

  for (midx = 0; match_strings[midx].name; midx++)
    {
      const char *match = match_strings[midx].name;

      if (stream_peek(in) == *match)
        {
          while (*match && stream_get(in) == *match)
            {
              match++;
            }

          if (*match != '\0')
            {
              /* Failure. */

              return NULL;
            }

          item->type = match_strings[midx].type;
          item->valueint = match_strings[midx].value;
          return in;
        }
    }

  if (stream_peek(in) == '\"')
    {
      return stream_parse_string(item, in);
    }

  if (stream_peek(in) == '-' ||
      (stream_peek(in) >= '0' && stream_peek(in) <= '9'))
    {
      return stream_parse_number(item, in);
    }

  if (stream_peek(in) == '[')
    {
      return stream_parse_array(item, in);
    }

  if (stream_peek(in) == '{')
    {
      return stream_parse_object(item, in);
    }

  /* Failure. */

  return NULL;
}

/* Build an array from input text. */

static cJSON_instream *stream_parse_array(cJSON *item, cJSON_instream *in)
{
  cJSON *child;

  if (stream_peek(in) != '[')
    {
      /* not an array! */

      return NULL;
    }

  item->type = cJSON_Array;
  (void)stream_get(in);
  in = skip(in);
  if (stream_peek(in) == ']')
    {
      /* Empty array. */

      (void)stream_get(in);
      return in;
    }

  item->child = child = cJSON_New_Item();
  if (!item->child)
    {
      /* Memory fail */

      return NULL;
    }

  /* Skip any spacing, get the value. */

  in = skip(stream_parse_value(child, skip(in)));
  if (!in)
    {
      return NULL;
    }

  while (stream_peek(in) == ',')
    {
      cJSON *new_item;
      if (!(new_item = cJSON_New_Item()))
        {
          /* memory fail */

          return NULL;
        }

      child->next = new_item;
      new_item->prev = child;
      child = new_item;
      (void)stream_get(in);
      in = skip(stream_parse_value(child, skip(in)));
      if (!in)
        {
          /* Memory fail */

          return NULL;
        }
    }

  if (stream_peek(in) == ']')
    {
      /* End of array */

      (void)stream_get(in);
      return in;
    }

  /* Malformed */

  return NULL;
}

/* Build an object from the text. */

static cJSON_instream *stream_parse_object(cJSON *item, cJSON_instream *in)
{
  cJSON *child;
  if (stream_peek(in) != '{')
    {
      /* Not an object! */

      return NULL;
    }

  item->type = cJSON_Object;
  (void)stream_get(in);
  in = skip(in);
  if (stream_peek(in) == '}')
    {
      /* Empty array. */

      (void)stream_get(in);
      return in;
    }

  item->child = child = cJSON_New_Item();
  if (!item->child)
    {
      return NULL;
    }

  in = skip(stream_parse_string(child, skip(in)));
  if (!in)
    {
      return NULL;
    }

  child->string = child->valuestring;
  child->valuestring = 0;
  if (stream_peek(in) != ':')
    {
      return NULL;
    }

   /* Skip any spacing, get the value. */

  (void)stream_get(in);
  in = skip(stream_parse_value(child, skip(in)));
  if (!in)
    {
      return NULL;
    }

  while (stream_peek(in) == ',')
    {
      cJSON *new_item;
      if (!(new_item = cJSON_New_Item()))
        {
          /* Memory fail */

          return NULL;
        }

      child->next = new_item;
      new_item->prev = child;
      child = new_item;
      (void)stream_get(in);
      in = skip(stream_parse_string(child, skip(in)));
      if (!in)
        {
          return NULL;
        }

      child->string = child->valuestring;
      child->valuestring = 0;
      if (stream_peek(in) != ':')
        {
          return NULL;
        }

     /* Skip any spacing, get the value. */

      (void)stream_get(in);
      in = skip(stream_parse_value(child, skip(in)));
      if (!in)
        {
          return NULL;
        }
    }

  if (stream_peek(in) == '}')
    {
      /* End of array */

      (void)stream_get(in);
      return in;
    }

  /* Malformed */

  return NULL;
}

static char getc_string(void *priv)
{
  char **pvalue = priv;
  char *value = *pvalue;
  char val = *value;

  if (val)
    {
      value++;
      *pvalue = value;
    }

  return val;
}

static char getc_fd(void *priv)
{
  struct parse_fd_priv_s *parse = priv;
  ssize_t ret;
  char val;

  if (parse->max_readlen > 0 && parse->nread == parse->max_readlen)
    {
      /* End of fd-buffer reached, give NULL char. */

      return '\0';
    }

  if (parse->buf && parse->bufpos < parse->buflen)
    {
      /* Load next from buffer. */

      return parse->buf[parse->bufpos++];
    }

  if (parse->max_readlen > 0 && !parse->buf)
    {
      /* Allocate read buffer. */

      parse->buf = malloc(FD_PARSER_CACHE_SIZE);
    }

  if (parse->buf)
    {
      size_t maxread = parse->max_readlen - parse->nread;

      if (maxread > FD_PARSER_CACHE_SIZE)
        {
          maxread = FD_PARSER_CACHE_SIZE;
        }

      /* Read buffer full or remaining bytes. */

      while ((ret = read(parse->fd, parse->buf, maxread)) < 0)
        {
          if (errno == EINTR || errno == EAGAIN)
            {
              continue;
            }

          break;
        }

      if (ret <= 0)
        {
          return '\0';
        }

      parse->nread += ret;
      parse->bufpos = 0;
      parse->buflen = ret;

      return parse->buf[parse->bufpos++];
    }

  /* Buffer not used, either allocation failed or do not know input buffer
   * length. */

  while ((ret = read(parse->fd, &val, 1)) < 0)
    {
      if (errno == EINTR || errno == EAGAIN)
        {
          continue;
        }

      break;
    }

  if (ret == 1)
    {
      parse->nread++;
      return val;
    }

  return '\0';
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Parse an object from stream - create a new root, and populate. */

cJSON *cJSON_Parse_Stream(char (*getc_fn)(void *priv), void *priv)
{
  cJSON_instream stream = {};
  cJSON *c;

  c = cJSON_New_Item();
  if (!c)
    {
      /* Memory fail */

      return NULL;
    }

  stream.getc_fn = getc_fn;
  stream.fn_priv = priv;
  stream.curr = -1;

  if (!stream_parse_value(c, skip(&stream)))
    {
      cJSON_Delete(c);
      return NULL;
    }

  skip(&stream);
  if (stream_get(&stream))
    {
      /* Malformed at end. */

      cJSON_Delete(c);
      return NULL;
    }

  return c;
}

/* Parse an object from input string - create a new root, and populate. */

cJSON *cJSON_Parse(const char *value)
{
  char **pvalue = (void *)&value;
  return cJSON_Parse_Stream(getc_string, (void *)pvalue);
}

/* Parse an object from file-descriptor - create a new root, and populate. */

cJSON *cJSON_Parse_fd(int fd, ssize_t max_readlen, size_t *nread)
{
  struct parse_fd_priv_s parse =
    {
      .fd = fd,
      .max_readlen = max_readlen,
      .buf = NULL
    };
  cJSON *obj;

  obj = cJSON_Parse_Stream(getc_fd, &parse);
  if (nread)
    {
      *nread = parse.nread;
    }

  free(parse.buf);

  return obj;
}
