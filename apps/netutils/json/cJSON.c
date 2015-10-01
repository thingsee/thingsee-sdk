/****************************************************************************
 * apps/netutils/json/cJSON.c
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Ported by: Darcy Gong
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

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <float.h>
#include <limits.h>
#include <ctype.h>
#include <unistd.h>

#include <apps/netutils/cJSON.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define cJSON_malloc malloc
#define cJSON_free free
#define cJSON_realloc realloc
#define cJSON_strdup strdup
#define cJSON_New_Item() ((cJSON *)calloc(1, sizeof(cJSON)))

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char *ep;

static const unsigned char firstByteMark[7] =
  { 0x00, 0x00, 0xc0, 0xe0, 0xf0, 0xf8, 0xfc };

/****************************************************************************
 * Private Prototypes
 ****************************************************************************/

static const char *parse_value(cJSON *item, const char *value);
static char *print_value(cJSON *item, int depth, int fmt);
static const char *parse_array(cJSON *item, const char *value);
static char *print_array(cJSON *item, int depth, int fmt);
static const char *parse_object(cJSON *item, const char *value);
static char *print_object(cJSON *item, int depth, int fmt);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int cJSON_strcasecmp(const char *s1, const char *s2)
{
  if (!s1)
    {
      return (s1 == s2) ? 0 : 1;
    }

  if (!s2)
    {
      return 1;
    }

  for (; tolower(*s1) == tolower(*s2); ++s1, ++s2)
    {
      if (*s1 == 0)
        {
          return 0;
        }
    }

  return tolower(*(const unsigned char *)s1) - tolower(*(const unsigned char *)s2);
}

/* Parse the input text to generate a number, and populate the result into item. */

static const char *parse_number(cJSON *item, const char *num)
{
  double n = 0, sign = 1, scale = 0;
  int subscale = 0, signsubscale = 1;

  /* Could use sscanf for this? */
  /* Has sign? */

  if (*num == '-')
    {
      sign = -1, num++;
    }

  /* is zero */

  if (*num == '0')
    {
      num++;
    }

  /* Number? */

  if (*num >= '1' && *num <= '9')
    {
      do
        {
          n = (n * 10.0) + (*num++ - '0');
        }
      while (*num >= '0' && *num <= '9');
    }

  /* Fractional part? */

  if (*num == '.' && num[1] >= '0' && num[1] <= '9')
    {
      num++;
      do
        {
          n = (n * 10.0) + (*num++ - '0'), scale--;
        }
      while (*num >= '0' && *num <= '9');
    }

  /* Exponent? */

  if (*num == 'e' || *num == 'E')
    {
      num++;
      if (*num == '+')
        {
          num++;
        }

      /* With sign? */

      else if (*num == '-')
        {
          signsubscale = -1;
          num++;
        }

      /* Number? */

      while (*num >= '0' && *num <= '9')
        {
          subscale = (subscale * 10) + (*num++ - '0');
        }
    }

  /* number = +/- number.fraction * 10^+/-exponent */

  n = sign * n * pow(10.0, (scale + subscale * signsubscale));
  item->valuedouble = n;
  item->valueint = (int)n;
  item->type = cJSON_Number;
  return num;
}

/* Render the number nicely from the given item into a string. */

static char *print_number(cJSON *item)
{
  char *str;
  double d = item->valuedouble;
  double id = item->valueint;
  size_t slen;
  int type;

  /* Get type and needed length output string. */

  if ((d <= INT_MAX && d >= INT_MIN) &&
      (fabs(id - d) <= DBL_EPSILON || fabs(floor(d) - d) <= DBL_EPSILON))
    {
      slen = snprintf(NULL, 0, "%d", item->valueint);
      type = 1;
    }
  else if (fabs(d) < 1.0e-6 || fabs(d) > 1.0e9)
    {
      slen = snprintf(NULL, 0, "%e", d);
      type = -1;
    }
  else
    {
      slen = snprintf(NULL, 0, "%.10f", d);
      type = 0;
    }

  /* Allocate and fill output. */

  slen += 1;
  str = (char *)cJSON_malloc(slen);

  if (!str)
    return NULL;

  if (type > 0)
    {
      snprintf(str, slen, "%d", item->valueint);
    }
  else if (type < 0)
    {
      snprintf(str, slen, "%e", d);
    }
  else
    {
      snprintf(str, slen, "%.10f", d);

      if (strchr(str, '.'))
        {
          /* Remove trailing zeros. */

          slen -= 1;

          while (slen > 0)
            {
              if (str[--slen] != '0')
                break;

              str[slen] = '\0';
            }

          /* Remove trailing dot. */

          if (slen > 0 && str[slen] == '.')
            str[slen] = '\0';
        }
    }

  return str;
}

/* Parse the input text into an unescaped cstring, and populate item. */

static const char *parse_string(cJSON *item, const char *str)
{
  const char *ptr = str + 1;
  char *ptr2;
  char *out;
  int len = 0;
  unsigned uc;
  unsigned uc2;

  if (*str != '\"')
    {
      /* not a string! */

      ep = str;
      return 0;
    }

  while (*ptr != '\"' && *ptr && ++len)
    {
      /* Skip escaped quotes. */

      if (*ptr++ == '\\')
        {
          ptr++;
        }
    }

  /* This is how long we need for the string, roughly. */

  out = (char *)cJSON_malloc(len + 1);
  if (!out)
    {
      return 0;
    }

  ptr = str + 1;
  ptr2 = out;
  while (*ptr != '\"' && *ptr)
    {
      if (*ptr != '\\')
        {
          *ptr2++ = *ptr++;
        }
      else
        {
          ptr++;
          switch (*ptr)
            {
            case 'b':
              *ptr2++ = '\b';
              break;

            case 'f':
              *ptr2++ = '\f';
              break;

            case 'n':
              *ptr2++ = '\n';
              break;

            case 'r':
              *ptr2++ = '\r';
              break;

            case 't':
              *ptr2++ = '\t';
              break;

            case 'u':
              /* Transcode utf16 to utf8. */
              /* Get the unicode char. */

              sscanf(ptr + 1, "%4x", &uc);
              ptr += 4;

              /* Check for invalid. */

              if ((uc >= 0xdc00 && uc <= 0xdfff) || uc == 0)
                {
                  break;
                }

              if (uc >= 0xd800 && uc <= 0xdbff) /* UTF16 surrogate pairs. */
                {
                  /* missing second-half of surrogate. */

                  if (ptr[1] != '\\' || ptr[2] != 'u')
                    {
                      break;
                    }

                  sscanf(ptr + 3, "%4x", &uc2);
                  ptr += 6;
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

            default:
              *ptr2++ = *ptr;
              break;
            }
          ptr++;
        }
    }

  *ptr2 = 0;
  if (*ptr == '\"')
    {
      ptr++;
    }

  item->valuestring = out;
  item->type = cJSON_String;
  return ptr;
}

/* Render the cstring provided to an escaped version that can be printed. */

static char *print_string_ptr(const char *str)
{
  const char *ptr;
  char *ptr2, *out;
  int len = 0;
  unsigned char token;

  if (!str)
    {
      return cJSON_strdup("");
    }

  ptr = str;
  while ((token = *ptr) && ++len)
    {
      if (strchr("\"\\\b\f\n\r\t", token))
        {
          len++;
        }
      else if (token < 32)
        {
          len += 5;
        }

      ptr++;
    }

  out = (char *)cJSON_malloc(len + 3);
  if (!out)
    {
      return 0;
    }

  ptr2 = out;
  ptr = str;
  *ptr2++ = '\"';
  while (*ptr)
    {
      if ((unsigned char)*ptr > 31 && *ptr != '\"' && *ptr != '\\')
        {
          *ptr2++ = *ptr++;
        }
      else
        {
          *ptr2++ = '\\';
          switch (token = *ptr++)
            {
            case '\\':
              *ptr2++ = '\\';
              break;

            case '\"':
              *ptr2++ = '\"';
              break;

            case '\b':
              *ptr2++ = 'b';
              break;

            case '\f':
              *ptr2++ = 'f';
              break;

            case '\n':
              *ptr2++ = 'n';
              break;

            case '\r':
              *ptr2++ = 'r';
              break;

            case '\t':
              *ptr2++ = 't';
              break;

            default:
              /* Escape and print */

              sprintf(ptr2, "u%04x", token);
              ptr2 += 5;
              break;
            }
        }
    }

  *ptr2++ = '\"';
  *ptr2++ = 0;
  return out;
}

/* Invote print_string_ptr (which is useful) on an item. */

static char *print_string(cJSON *item)
{
  return print_string_ptr(item->valuestring);
}

/* Utility to jump whitespace and cr/lf */

static const char *skip(const char *in)
{
  while (in && *in && (unsigned char)*in <= 32)
    {
      in++;
    }

  return in;
}

/* Parser core - when encountering text, process appropriately. */

static const char *parse_value(cJSON *item, const char *value)
{
  if (!value)
    {
      /* Fail on null. */

      return 0;
    }

  if (!strncmp(value, "null", 4))
    {
      item->type = cJSON_NULL;
      return value + 4;
    }

  if (!strncmp(value, "false", 5))
    {
      item->type = cJSON_False;
      return value + 5;
    }

  if (!strncmp(value, "true", 4))
    {
      item->type = cJSON_True;
      item->valueint = 1;
      return value + 4;
    }

  if (*value == '\"')
    {
      return parse_string(item, value);
    }

  if (*value == '-' || (*value >= '0' && *value <= '9'))
    {
      return parse_number(item, value);
    }

  if (*value == '[')
    {
      return parse_array(item, value);
    }

  if (*value == '{')
    {
      return parse_object(item, value);
    }

  /* Failure. */

  ep = value;
  return 0;
}

/* Render a value to text. */

static char *print_value(cJSON *item, int depth, int fmt)
{
  char *out = 0;
  if (!item)
    {
      return 0;
    }

  switch ((item->type) & 255)
    {
    case cJSON_NULL:
      out = cJSON_strdup("null");
      break;

    case cJSON_False:
      out = cJSON_strdup("false");
      break;

    case cJSON_True:
      out = cJSON_strdup("true");
      break;

    case cJSON_Number:
      out = print_number(item);
      break;

    case cJSON_String:
      out = print_string(item);
      break;

    case cJSON_Array:
      out = print_array(item, depth, fmt);
      break;

    case cJSON_Object:
      out = print_object(item, depth, fmt);
      break;
    }

  return out;
}

/* Build an array from input text. */

static const char *parse_array(cJSON *item, const char *value)
{
  cJSON *child;

  if (*value != '[')
    {
      /* not an array! */

      ep = value;
      return 0;
    }

  item->type = cJSON_Array;
  value = skip(value + 1);
  if (*value == ']')
    {
      /* Empty array. */

      return value + 1;
    }

  item->child = child = cJSON_New_Item();
  if (!item->child)
    {
      /* Memory fail */

      return 0;
    }

  /* Skip any spacing, get the value. */

  value = skip(parse_value(child, skip(value)));
  if (!value)
    {
      return 0;
    }

  while (*value == ',')
    {
      cJSON *new_item;
      if (!(new_item = cJSON_New_Item()))
        {
          /* <emory fail */

          return 0;
        }

      child->next = new_item;
      new_item->prev = child;
      child = new_item;
      value = skip(parse_value(child, skip(value + 1)));
      if (!value)
        {
          /* Memory fail */

          return 0;
        }
    }

  if (*value == ']')
    {
      /* End of array */

      return value + 1;
    }

  /* Malformed */

  ep = value;
  return 0;
}

/* Render an array to text */

static char *print_array(cJSON *item, int depth, int fmt)
{
  char **entries = NULL;
  char *out = 0;
  char *ptr;
  char *ret;
  int len = 5;
  cJSON *child = item->child;
  int numentries = 0;
  int i = 0;
  int fail = 0;

  /* How many entries in the array? */

  while (child)
    {
      numentries++, child = child->next;
    }

  if (numentries > 0)
    {
      /* Allocate an array to hold the values for each */

      entries = (char **)cJSON_malloc(numentries * sizeof(char *));
      if (!entries)
        {
          return 0;
        }

      memset(entries, 0, numentries * sizeof(char *));

      /* Retrieve all the results: */

      child = item->child;
      while (child && !fail)
        {
          ret = print_value(child, depth + 1, fmt);
          entries[i++] = ret;
          if (ret)
            {
              len += strlen(ret) + 2 + (fmt ? 1 : 0);
            }
          else
            {
              fail = 1;
            }

          child = child->next;
        }
    }

  /* If we didn't fail, try to malloc the output string */

  if (!fail)
    {
      out = (char *)cJSON_malloc(len);
    }

  /* If that fails, we fail. */

  if (!out)
    {
      fail = 1;
    }

  /* Handle failure. */

  if (fail)
    {
      for (i = 0; i < numentries; i++)
        {
          if (entries[i])
            {
              cJSON_free(entries[i]);
            }
        }

      cJSON_free(entries);
      return 0;
    }

  /* Compose the output array. */

  *out = '[';
  ptr = out + 1;
  *ptr = 0;
  for (i = 0; i < numentries; i++)
    {
      strcpy(ptr, entries[i]);
      ptr += strlen(entries[i]);
      if (i != numentries - 1)
        {
          *ptr++ = ',';
          if (fmt)
            {
              *ptr++ = ' ';
            }

          *ptr = 0;
        }
      cJSON_free(entries[i]);
    }

  cJSON_free(entries);
  *ptr++ = ']';
  *ptr++ = 0;
  return out;
}

/* Build an object from the text. */

static const char *parse_object(cJSON *item, const char *value)
{
  cJSON *child;
  if (*value != '{')
    {
      /* Not an object! */

      ep = value;
      return 0;
    }

  item->type = cJSON_Object;
  value = skip(value + 1);
  if (*value == '}')
    {
      /* Empty array. */

      return value + 1;
    }

  item->child = child = cJSON_New_Item();
  if (!item->child)
    {
      return 0;
    }

  value = skip(parse_string(child, skip(value)));
  if (!value)
    {
      return 0;
    }

  child->string = child->valuestring;
  child->valuestring = 0;
  if (*value != ':')
    {
      ep = value;
      return 0;
    }

   /* Skip any spacing, get the value. */

  value = skip(parse_value(child, skip(value + 1)));
  if (!value)
    {
      return 0;
    }

  while (*value == ',')
    {
      cJSON *new_item;
      if (!(new_item = cJSON_New_Item()))
        {
          /* Memory fail */

          return 0;
        }

      child->next = new_item;
      new_item->prev = child;
      child = new_item;
      value = skip(parse_string(child, skip(value + 1)));
      if (!value)
        {
          return 0;
        }

      child->string = child->valuestring;
      child->valuestring = 0;
      if (*value != ':')
        {
          ep = value;
          return 0;
        }

     /* Skip any spacing, get the value. */

      value = skip(parse_value(child, skip(value + 1)));
      if (!value)
        {
          return 0;
        }
    }

  if (*value == '}')
    {
      /* End of array */

      return value + 1;
    }

  /* Malformed */

  ep = value;
  return 0;
}

/* Render an object to text. */

static char *print_object(cJSON *item, int depth, int fmt)
{
  char **entries = 0;
  char **names = 0;
  char *out = 0;
  char *ptr;
  char *ret;
  char *str;
  int len = 7;
  int i = 0;
  int j;
  cJSON *child = item->child;
  int numentries = 0;
  int fail = 0;

  /* Count the number of entries. */

  while (child)
    {
      numentries++, child = child->next;
    }

  if (numentries > 0)
    {
      /* Allocate space for the names and the objects */

      entries = (char **)cJSON_malloc(numentries * sizeof(char *));
      if (!entries)
        {
          return 0;
        }

      names = (char **)cJSON_malloc(numentries * sizeof(char *));
      if (!names)
        {
          cJSON_free(entries);
          return 0;
        }

      memset(entries, 0, sizeof(char *) * numentries);
      memset(names, 0, sizeof(char *) * numentries);

      /* Collect all the results into our arrays: */

      child = item->child;
      depth++;
      if (fmt)
        {
          len += depth;
        }

      while (child)
        {
          names[i] = str = print_string_ptr(child->string);
          entries[i++] = ret = print_value(child, depth, fmt);
          if (str && ret)
            {
              len += strlen(ret) + strlen(str) + 2 + (fmt ? 2 + depth : 0);
            }
          else
            {
              fail = 1;
            }

          child = child->next;
        }
    }

  /* Try to allocate the output string */

  if (!fail)
    {
      out = (char *)cJSON_malloc(len);
    }

  if (!out)
    {
      fail = 1;
    }

  /* Handle failure */

  if (fail)
    {
      for (i = 0; i < numentries; i++)
        {
          if (names[i])
            {
              cJSON_free(names[i]);
            }

          if (entries[i])
            {
              cJSON_free(entries[i]);
            }
        }

      cJSON_free(names);
      cJSON_free(entries);
      return 0;
    }

  /* Compose the output: */

  *out = '{';
  ptr = out + 1;
  if (fmt)
    {
      *ptr++ = '\n';
    }
  *ptr = 0;

  for (i = 0; i < numentries; i++)
    {
      if (fmt)
        {
          for (j = 0; j < depth; j++)
            {
            *ptr++ = '\t';
            }
        }

      strcpy(ptr, names[i]);
      ptr += strlen(names[i]);
      *ptr++ = ':';
      if (fmt)
        {
          *ptr++ = '\t';
        }

      strcpy(ptr, entries[i]);
      ptr += strlen(entries[i]);
      if (i != numentries - 1)
        {
          *ptr++ = ',';
        }

      if (fmt)
        {
          *ptr++ = '\n';
        }

      *ptr = 0;
      cJSON_free(names[i]);
      cJSON_free(entries[i]);
    }

  cJSON_free(names);
  cJSON_free(entries);
  if (fmt)
    {
      for (i = 0; i < depth - 1; i++)
        {
          *ptr++ = '\t';
        }
    }

  *ptr++ = '}';
  *ptr++ = 0;
  return out;
}

/* Utility for array list handling. */

static void suffix_object(cJSON *prev, cJSON *item)
{
  prev->next = item;
  item->prev = prev;
}

/* Utility for handling references. */

static cJSON *create_reference(cJSON *item)
{
  cJSON *ref = cJSON_New_Item();
  if (!ref)
    {
      return 0;
    }

  memcpy(ref, item, sizeof(cJSON));
  ref->string = 0;
  ref->type |= cJSON_IsReference;
  ref->next = ref->prev = 0;
  return ref;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

const char *cJSON_GetErrorPtr(void)
{
  return ep;
}

/* Delete a cJSON structure. */

void cJSON_Delete(cJSON *c)
{
  cJSON *next;
  while (c)
    {
      next = c->next;
      if (!(c->type & cJSON_IsReference) && c->child)
        {
          cJSON_Delete(c->child);
        }

      if (!(c->type & cJSON_IsReference) && c->valuestring)
        {
          cJSON_free(c->valuestring);
        }

      if (c->string)
        {
          cJSON_free(c->string);
        }

      cJSON_free(c);
      c = next;
    }
}

/* Parse an object - create a new root, and populate. */

cJSON *cJSON_Parse_Old(const char *value)
{
  cJSON *c = cJSON_New_Item();
  ep = 0;
  if (!c)
    {
      /* Memory fail */

      return 0;
    }

  if (!parse_value(c, skip(value)))
    {
      cJSON_Delete(c);
      return 0;
    }

  return c;
}

/* Render a cJSON item/entity/structure to text. */

char *cJSON_Print_Old(cJSON *item)
{
  return print_value(item, 0, 1);
}

char *cJSON_PrintUnformatted_Old(cJSON *item)
{
  return print_value(item, 0, 0);
}

/* Get Array size/item / object item. */

int cJSON_GetArraySize(cJSON *array)
{
  cJSON *c = array->child;
  int i = 0;

  while (c)
    {
      i++;
      c = c->next;
    }

  return i;
}

cJSON *cJSON_GetArrayItem(cJSON *array, int item)
{
  cJSON *c = array->child;

  while (c && item > 0)
    {
      item--;
      c = c->next;
    }

  return c;
}

cJSON *cJSON_GetObjectItem(cJSON *object, const char *string)
{
  cJSON *c = object->child;

  while (c && cJSON_strcasecmp(c->string, string))
    {
      c = c->next;
    }

  return c;
}

/* Add item to array/object. */
void cJSON_AddItemToArray(cJSON *array, cJSON *item)
{
  cJSON *c = array->child;

  if (!item)
    {
      return;
    }

  if (!c)
    {
      array->child = item;
    }
  else
    {
      while (c && c->next)
        {
          c = c->next;
        }

      suffix_object(c, item);
    }
}

void cJSON_AddItemToObject(cJSON *object, const char *string, cJSON *item)
{
  if (!item)
    {
      return;
    }

  if (item->string)
    {
      cJSON_free(item->string);
    }

  item->string = cJSON_strdup(string);
  cJSON_AddItemToArray(object, item);
}

void cJSON_AddItemReferenceToArray(cJSON *array, cJSON *item)
{
  cJSON_AddItemToArray(array, create_reference(item));
}

void cJSON_AddItemReferenceToObject(cJSON *object, const char *string, cJSON *item)
{
  cJSON_AddItemToObject(object, string, create_reference(item));
}

cJSON *cJSON_DetachItemFromArray(cJSON *array, int which)
{
  cJSON *c = array->child;

  while (c && which > 0)
    {
      c = c->next, which--;
    }

  if (!c)
    {
      return 0;
    }

  if (c->prev)
    {
      c->prev->next = c->next;
    }

  if (c->next)
    {
      c->next->prev = c->prev;
    }

  if (c == array->child)
    {
      array->child = c->next;
    }

  c->prev = c->next = 0;
  return c;
}

void cJSON_DeleteItemFromArray(cJSON *array, int which)
{
  cJSON_Delete(cJSON_DetachItemFromArray(array, which));
}

cJSON *cJSON_DetachItemFromObject(cJSON *object, const char *string)
{
  int i = 0;
  cJSON *c = object->child;

  while (c && cJSON_strcasecmp(c->string, string))
    {
      i++;
      c = c->next;
    }

  if (c)
    {
      return cJSON_DetachItemFromArray(object, i);
    }

  return 0;
}

void cJSON_DeleteItemFromObject(cJSON *object, const char *string)
{
  cJSON_Delete(cJSON_DetachItemFromObject(object, string));
}

/* Replace array/object items with new ones. */

void cJSON_ReplaceItemInArray(cJSON *array, int which, cJSON *newitem)
{
  cJSON *c = array->child;

  while (c && which > 0)
    {
      c = c->next, which--;
    }

  if (!c)
    {
      return;
    }

  newitem->next = c->next;
  newitem->prev = c->prev;
  if (newitem->next)
    {
      newitem->next->prev = newitem;
    }

  if (c == array->child)
    {
      array->child = newitem;
    }
  else
    {
      newitem->prev->next = newitem;
    }

  c->next = c->prev = 0;
  cJSON_Delete(c);
}

void cJSON_ReplaceItemInObject(cJSON *object, const char *string, cJSON *newitem)
{
  int i = 0;
  cJSON *c = object->child;

  while (c && cJSON_strcasecmp(c->string, string))
    {
      i++;
      c = c->next;
    }

  if (c)
    {
      newitem->string = cJSON_strdup(string);
      cJSON_ReplaceItemInArray(object, i, newitem);
    }
}

/* Create basic types: */

cJSON *cJSON_CreateNull(void)
{
  cJSON *item = cJSON_New_Item();
  if (item)
    {
      item->type = cJSON_NULL;
    }

  return item;
}

cJSON *cJSON_CreateTrue(void)
{
  cJSON *item = cJSON_New_Item();
  if (item)
    {
      item->type = cJSON_True;
    }

  return item;
}

cJSON *cJSON_CreateFalse(void)
{
  cJSON *item = cJSON_New_Item();
  if (item)
    {
      item->type = cJSON_False;
    }

  return item;
}

cJSON *cJSON_CreateBool(int b)
{
  cJSON *item = cJSON_New_Item();
  if (item)
    {
      item->type = b ? cJSON_True : cJSON_False;
    }

  return item;
}

cJSON *cJSON_CreateNumber(double num)
{
  cJSON *item = cJSON_New_Item();
  if (item)
    {
      item->type = cJSON_Number;
      item->valuedouble = num;
      item->valueint = (int)num;
    }

  return item;
}

cJSON *cJSON_CreateString(const char *string)
{
  cJSON *item = cJSON_New_Item();
  if (item)
    {
      item->type = cJSON_String;
      item->valuestring = cJSON_strdup(string);
    }

  return item;
}

cJSON *cJSON_CreateArray(void)
{
  cJSON *item = cJSON_New_Item();
  if (item)
    {
      item->type = cJSON_Array;
    }

  return item;
}

cJSON *cJSON_CreateObject(void)
{
  cJSON *item = cJSON_New_Item();
  if (item)
    {
      item->type = cJSON_Object;
    }

  return item;
}

/* Create Arrays: */

cJSON *cJSON_CreateIntArray(const int *numbers, int count)
{
  cJSON *n = 0;
  cJSON *p = 0;
  cJSON *a = cJSON_CreateArray();
  int i;

  for (i = 0; a && i < count; i++)
    {
      n = cJSON_CreateNumber(numbers[i]);
      if (!i)
        {
          a->child = n;
        }
      else
        {
          suffix_object(p, n);
        }

      p = n;
    }

  return a;
}

cJSON *cJSON_CreateFloatArray(const float *numbers, int count)
{
  cJSON *n = 0;
  cJSON *p = 0;
  cJSON *a = cJSON_CreateArray();
  int i;

  for (i = 0; a && i < count; i++)
    {
      n = cJSON_CreateNumber(numbers[i]);
      if (!i)
        {
          a->child = n;
        }
      else
        {
          suffix_object(p, n);
        }

      p = n;
    }

  return a;
}

cJSON *cJSON_CreateDoubleArray(const double *numbers, int count)
{
  cJSON *n = 0;
  cJSON *p = 0;
  cJSON *a = cJSON_CreateArray();
  int i;

  for (i = 0; a && i < count; i++)
    {
      n = cJSON_CreateNumber(numbers[i]);
      if (!i)
        {
          a->child = n;
        }
      else
        {
          suffix_object(p, n);
        }

      p = n;
    }

  return a;
}

cJSON *cJSON_CreateStringArray(const char **strings, int count)
{
  cJSON *n = 0;
  cJSON *p = 0;
  cJSON *a = cJSON_CreateArray();
  int i;

  for (i = 0; a && i < count; i++)
    {
      n = cJSON_CreateString(strings[i]);
      if (!i)
        {
          a->child = n;
        }
      else
        {
          suffix_object(p, n);
        }

      p = n;
    }

  return a;
}
