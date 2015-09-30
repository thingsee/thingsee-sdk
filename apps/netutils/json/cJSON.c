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

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Prototypes
 ****************************************************************************/

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

cJSON *cJSON_New_Item(void)
{
  return calloc(1, sizeof(cJSON));
}

const char *cJSON_GetErrorPtr(void)
{
  return "";
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
