/****************************************************************************
 * apps/netutils/json/cJSON.c
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Ported by: Darcy Gong
 *
 *   Copyright (c) 2015 Haltian Ltd.
 *     - Jussi Kivilinna <jussi.kivilinna@haltian.com>
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

#include <nuttx/mm/mm.h>

#include <apps/netutils/cJSON.h>

#include "cJSON_internal.h"

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

/* Get next pointer of item. */

static cJSON *cJSON_GetNextPointer(cJSON *item)
{
  uint8_t next_type = item->info & JSON_NEXT_MASK;
  size_t item_len;

  if (next_type == JSON_NEXT_NULL)
    {
      return NULL;
    }

  if (next_type == JSON_NEXT_PTR)
    {
      struct cJSON_DataNext *field = (void *)item->data;

      return field->next;
    }

  DEBUGASSERT(next_type == JSON_NEXT_MEM);

  item_len = cJSON_GetFieldPointers(item, NULL, NULL, NULL, NULL, NULL, NULL);
  return (cJSON *)((uint8_t *)item + item_len);
}

/* Get name-type needed for given name-length. */

static uint8_t cJSON_NameFieldTypeByLength(size_t name_string_len)
{
  if (name_string_len == 0)
    return JSON_NAME_EMPTY;

  if (name_string_len + 1 <= UINT8_MAX)
    return JSON_NAME_SHORT;

  /* DEBUGASSERT(name_string_len + 1 > UINT16_MAX); */

  return JSON_NAME_LONG;
}

/* Utility for getting name string pointer of object. */

static const char *cJSON_GetNameString(cJSON *item)
{
  void *name_field = NULL;
  uint8_t name_type = JSON_NAME_EMPTY;

  (void)cJSON_GetFieldPointers(item, NULL, NULL, NULL, NULL, &name_field,
                               &name_type);

  if (name_type == JSON_NAME_EMPTY)
    {
      return "";
    }

  DEBUGASSERT(name_field);

  if (name_type == JSON_NAME_SHORT)
    {
      struct cJSON_DataStringShort *ss = name_field;

      return ss->valuestring;
    }

  if (name_type == JSON_NAME_LONG)
    {
      struct cJSON_DataStringLong *sl = name_field;

      return sl->valuestring;
    }

  DEBUGASSERT(false);
  return NULL;
}

/* Utility for getting pointer to array/object data structure. */

static struct cJSON_DataArray *cJSON_ArrayField(cJSON *item)
{
  uint8_t data_type = item->info & JSON_DATA_MASK;

  if (data_type == JSON_DATA_ARRAY ||
      data_type == JSON_DATA_OBJECT)
    {
      struct cJSON_DataArray *data = NULL;

      (void)cJSON_GetFieldPointers(item, NULL, NULL, (void **)&data, NULL,
                                   NULL, NULL);

      return data;
    }

  return NULL;
}

/* Utility for getting pointer to next field structure. */

static struct cJSON_DataNext *cJSON_NextField(cJSON *item)
{
  uint8_t next_type = item->info & JSON_NEXT_MASK;

  if (next_type == JSON_NEXT_PTR)
    {
      struct cJSON_DataNext *next = NULL;

      (void)cJSON_GetFieldPointers(item, &next, NULL, NULL, NULL, NULL, NULL);

      return next;
    }

  return NULL;
}

/* Utility for changing name string of object. */

static cJSON *cJSON_ChangeName(cJSON *item, const char *new_name)
{
  size_t new_name_len = strlen(new_name);
  void *name_field = NULL;
  uint8_t name_type = 0;
  uint8_t next_type = 0;
  uint8_t new_name_type;
  size_t name_field_len = 0;
  size_t new_name_field_len = 0;
  cJSON *newitem;
  size_t objsize;
  size_t newsize;
  size_t endsize;

  objsize = cJSON_GetFieldPointers(item, NULL, &next_type, NULL, NULL,
                                   &name_field, &name_type);

  /* Caller needs to make sure that prev is not in packed format. */

  DEBUGASSERT(next_type == JSON_NEXT_PTR);

  if (name_type != JSON_NAME_EMPTY)
    {
      name_field_len = ((uint8_t *)item + objsize) - (uint8_t *)name_field;
    }

  new_name_type = cJSON_NameFieldTypeByLength(new_name_len);
  if (new_name_type != JSON_NAME_EMPTY)
    {
      new_name_field_len = (new_name_type == JSON_NAME_SHORT) ?
                            sizeof(struct cJSON_DataStringShort) :
                            sizeof(struct cJSON_DataStringLong);
      new_name_field_len += new_name_len + 1;
    }

  if (name_type == new_name_type && name_field_len == new_name_field_len)
    {
      /* Already correct size. */

      newsize = objsize;
    }
  else
    {
      if (new_name_field_len > name_field_len)
        {
          /* Grow cJSON storage. */

          newsize = objsize + (new_name_field_len - name_field_len);
        }
      else
        {
          /* Shrink cJSON storage. */

          newsize = objsize - (name_field_len - new_name_field_len);
        }

      newitem = cJSON_realloc(item, newsize);
      if (!newitem)
        {
          cJSON_Delete(item);
          return NULL;
        }

      if (newsize > objsize)
        {
          memset((uint8_t *)newitem + objsize, 0, newsize - objsize);
        }
      item = newitem;

      item->info &= ~JSON_NAME_MASK;
      item->info |= new_name_type;

      (void)cJSON_GetFieldPointers(item, NULL, NULL, NULL, NULL, &name_field,
                                   NULL);
    }

  /* Copy new name. */

  if (new_name_type == JSON_NAME_SHORT)
    {
      struct cJSON_DataStringShort *ss = name_field;

      memmove(ss->valuestring, new_name, new_name_len + 1);
      ss->string_size = new_name_len + 1;
    }
  else if (new_name_type == JSON_NAME_LONG)
    {
      struct cJSON_DataStringLong *sl = name_field;

      memmove(sl->valuestring, new_name, new_name_len + 1);
      sl->string_size = new_name_len + 1;
    }

  /* Verify object size against calculated. */

  endsize = cJSON_GetFieldPointers(item, NULL, NULL, NULL, NULL, NULL, NULL);

  DEBUGASSERT(endsize == newsize);

  return item;
}

/* Utility for creating new cJSON item. */

static cJSON *cJSON_New_Item(uint8_t info, size_t fields_size)
{
  cJSON *item;

  item = calloc(1, sizeof(cJSON) + fields_size);
  if (!item)
    {
      return NULL;
    }

  item->info = info;

  return item;
}

/* Utility for calculating next field size by type. */

static size_t cJSON_CalculateNextFieldSize(uint8_t next_type)
{
  if (next_type == JSON_NEXT_PTR)
    return sizeof(struct cJSON_DataNext);
  return 0;
}

/* Utility for calculating name field size. */

static size_t cJSON_CalculateNameFieldSize(const char *name, uint8_t *nametype)
{
  size_t namelen = name ? strlen(name) : 0;

  *nametype = cJSON_NameFieldTypeByLength(namelen);

  if (*nametype == JSON_NAME_EMPTY)
    {
      return 0;
    }
  else if (*nametype == JSON_NAME_SHORT)
    {
      return sizeof(struct cJSON_DataStringShort) + namelen + 1;
    }
  else /* if (*nametype == JSON_NAME_LONG) */
    {
      return sizeof(struct cJSON_DataStringShort) + namelen + 1;
    }
}

/* Check if two floating-point numbers are equal. */

static bool dbl_equal(double a, double b)
{
  double fabs_a = fabs(a);
  double fabs_b = fabs(b);
  return fabs(a - b) <= ((fabs_a > fabs_b ? fabs_b : fabs_a) * DBL_EPSILON);
}

/* Utility to calculate smallest storage field size for given number. */

static size_t cJSON_CalculateDataNumberFieldSize(double num,
                                                 uint8_t *data_type)
{
  int32_t s32;
  float f;

  /* Find smallest storage type that can accurately represent value of
   * 'num'. */

  if (num == 0.0)
    {
      if (!signbit(num))
        {
          /* Positive zero. Use 8-bit integer storage. */

          *data_type = JSON_DATA_NUM_INT8;
          return sizeof(struct cJSON_DataInt8);
        }
      else
        {
          /* Negative zero, needs floating point type. */

          *data_type = JSON_DATA_NUM_FLOAT;
          return sizeof(struct cJSON_DataFloat);
        }
    }

  s32 = num;
  if (s32 == num || dbl_equal(num, s32))
    {
      if (s32 >= INT8_MIN && s32 <= INT8_MAX)
        {
          /* Number does not have decimals and fits 8-bit integer. */

          *data_type = JSON_DATA_NUM_INT8;
          return sizeof(struct cJSON_DataInt8);
        }

      if (s32 >= INT16_MIN && s32 <= INT16_MAX)
        {
          /* Number does not have decimals and fits 16-bit integer. */

          *data_type = JSON_DATA_NUM_INT16;
          return sizeof(struct cJSON_DataInt16);
        }

      /* Number does not have decimals and fits 32-bit integer. */

      *data_type = JSON_DATA_NUM_INT32;
      return sizeof(struct cJSON_DataInt32);
    }

  f = num;
  if (f == num || dbl_equal(num, f))
    {
      /* Number can be accurately presented using 32-bit floating-point
       * number. */

      *data_type = JSON_DATA_NUM_FLOAT;
      return sizeof(struct cJSON_DataFloat);
    }

  /* Need to stick with double type. */

  *data_type = JSON_DATA_NUM_DOUBLE;
  return sizeof(struct cJSON_DataDouble);
}

/* Utility for calculating smallest storage field size for given string
 * length. */

static size_t cJSON_CalculateDataStringFieldSize(size_t string_len,
                                                 uint8_t *data_type)
{
  if (string_len + 1 <= UINT8_MAX)
    {
      *data_type = JSON_DATA_STR_SHORT;
      return sizeof(struct cJSON_DataStringShort) + string_len + 1;
    }

  /* DEBUGASSERT(string_len + 1 > UINT16_MAX); */

  *data_type = JSON_DATA_STR_LONG;
  return sizeof(struct cJSON_DataStringLong) + string_len + 1;
}

/* Utility for calculating storage size for given buffer length. */

static size_t cJSON_CalculateDataBufferFieldSize(size_t buffer_len)
{
  return sizeof(struct cJSON_DataStringLong) + buffer_len;
}

/* Utility for storing given number to configured data field. */

static void cJSON_SetValueNumber(cJSON *item, double num)
{
  uint8_t data_type = 0;
  void *data = NULL;

  (void)cJSON_GetFieldPointers(item, NULL, NULL, &data, &data_type, NULL, NULL);

  switch (data_type)
    {
    case JSON_DATA_NUM_INT8:
      ((struct cJSON_DataInt8 *)data)->valueint8 = num;
      break;

    case JSON_DATA_NUM_INT16:
      ((struct cJSON_DataInt16 *)data)->valueint16 = num;
      break;

    case JSON_DATA_NUM_INT32:
      ((struct cJSON_DataInt32 *)data)->valueint32 = num;
      break;

    case JSON_DATA_NUM_FLOAT:
      ((struct cJSON_DataFloat *)data)->valuefloat = num;
      break;

    case JSON_DATA_NUM_DOUBLE:
      ((struct cJSON_DataDouble *)data)->valuedouble = num;
      break;

    default:
      DEBUGASSERT(false);
      break;
    }
}

/* Utility for reading number value as integer from number data field. */

static int cJSON_GetValueInt(cJSON *item)
{
  uint8_t data_type = 0;
  void *data = NULL;

  (void)cJSON_GetFieldPointers(item, NULL, NULL, &data, &data_type, NULL, NULL);

  switch (data_type)
    {
    case JSON_DATA_NUM_INT8:
      return ((struct cJSON_DataInt8 *)data)->valueint8;
      break;

    case JSON_DATA_NUM_INT16:
      return ((struct cJSON_DataInt16 *)data)->valueint16;
      break;

    case JSON_DATA_NUM_INT32:
      return ((struct cJSON_DataInt32 *)data)->valueint32;
      break;

    case JSON_DATA_NUM_FLOAT:
      return ((struct cJSON_DataFloat *)data)->valuefloat;
      break;

    case JSON_DATA_NUM_DOUBLE:
      return ((struct cJSON_DataDouble *)data)->valuedouble;
      break;

    default:
      DEBUGASSERT(false);
      return 0;
    }
}

/* Utility for reading number value as floating-point from number data
 * field. */

static double cJSON_GetValueDouble(cJSON *item)
{
  uint8_t data_type = 0;
  void *data = NULL;

  (void)cJSON_GetFieldPointers(item, NULL, NULL, &data, &data_type, NULL, NULL);

  switch (data_type)
    {
    case JSON_DATA_NUM_INT8:
      return ((struct cJSON_DataInt8 *)data)->valueint8;
      break;

    case JSON_DATA_NUM_INT16:
      return ((struct cJSON_DataInt16 *)data)->valueint16;
      break;

    case JSON_DATA_NUM_INT32:
      return ((struct cJSON_DataInt32 *)data)->valueint32;
      break;

    case JSON_DATA_NUM_FLOAT:
      return ((struct cJSON_DataFloat *)data)->valuefloat;
      break;

    case JSON_DATA_NUM_DOUBLE:
      return ((struct cJSON_DataDouble *)data)->valuedouble;
      break;

    default:
      DEBUGASSERT(false);
      return 0;
    }
}

/* Utility for storing given buffer to configured string/buffer data field. */

static void cJSON_SetValueBuffer(cJSON *item, const void *buf, size_t buflen)
{
  uint8_t data_type = 0;
  void *data = NULL;
  char *dst = NULL;

  (void)cJSON_GetFieldPointers(item, NULL, NULL, &data, &data_type, NULL, NULL);

  switch (data_type)
    {
    case JSON_DATA_STR_SHORT:
      {
        struct cJSON_DataStringShort *ss = data;

        ss->string_size = buflen;
        dst = ss->valuestring;

        DEBUGASSERT(ss->string_size == buflen);
      }
      break;

    case JSON_DATA_BUFFER:
    case JSON_DATA_STR_LONG:
      {
        struct cJSON_DataStringLong *sl = data;

        sl->string_size = buflen;
        dst = sl->valuestring;

        DEBUGASSERT(sl->string_size == buflen);
      }
      break;

    default:
      DEBUGASSERT(false);
      break;
    }

  if (buflen)
    {
      memmove(dst, buf, buflen);
    }
}

/* Utility for reading string/buffer from data field. */

static struct cJSON_buffer_s cJSON_GetValueBuffer(cJSON *item)
{
  struct cJSON_buffer_s buf;
  uint8_t data_type = 0;
  void *data = NULL;

  (void)cJSON_GetFieldPointers(item, NULL, NULL, &data, &data_type, NULL,
                               NULL);

  switch (data_type)
    {
    case JSON_DATA_STR_SHORT:
      {
        struct cJSON_DataStringShort *ss = data;

        buf.ptr = ss->valuestring;
        buf.len = ss->string_size;
      }
      break;

    case JSON_DATA_BUFFER:
    case JSON_DATA_STR_LONG:
      {
        struct cJSON_DataStringLong *sl = data;

        buf.ptr = sl->valuestring;
        buf.len = sl->string_size;
      }
      break;

    default:
      DEBUGASSERT(false);
      break;
    }

  return buf;
}

/* Utility for storing child pointer to configured array/object data field. */

static void cJSON_SetArrayData(cJSON *item, cJSON *child)
{
  struct cJSON_DataArray *data = cJSON_ArrayField(item);

  DEBUGASSERT(data);

  data->child = child;
}

/* Utility to make unpacked copy of item. */

static cJSON *cJSON_MakeUnpackedCopy(cJSON *src)
{
  cJSON *newitem;
  void *new_data_field;
  void *new_name_field;
  uint8_t data_type;
  uint8_t name_type;
  void *data_field;
  void *name_field;
  size_t data_len;
  size_t name_len;
  size_t fields_size;

  (void)cJSON_GetFieldPointers(src, NULL, NULL, &data_field, &data_type,
                               &name_field, &name_type);

  fields_size = cJSON_CalculateNextFieldSize(JSON_NEXT_PTR);
  fields_size += data_len = cJSON_DataFieldLength(data_type, data_field);
  fields_size += name_len = cJSON_NameFieldLength(name_type, name_field);

  newitem = cJSON_New_Item(data_type | JSON_NEXT_PTR | name_type, fields_size);
  if (!newitem)
    {
      return NULL;
    }

  /* Copy data field. */

  if (data_len)
    {
      (void)cJSON_GetFieldPointers(newitem, NULL, NULL, &new_data_field, NULL,
                                   NULL, NULL);
      memmove(new_data_field, data_field, data_len);
    }

  /* Data field complete, now can get name field pointer and copy name. */

  if (name_len)
    {
      (void)cJSON_GetFieldPointers(newitem, NULL, NULL, NULL, NULL,
                                   &new_name_field, NULL);
      memmove(new_name_field, name_field, name_len);
    }

  return newitem;
}

/* Utility to calculate size of packed array. */

static size_t cJSON_PackedArraySize(cJSON *item)
{
  size_t total = 0;

  if (!item || !cJSON_IsPacked(item))
    {
      return 0;
    }

  do
    {
      total += cJSON_GetFieldPointers(item, NULL, NULL, NULL, NULL, NULL, NULL);

      item = cJSON_next(item);
    }
  while (item);

  return total;
}

/* Utility for copying data from cJSON object to another. */

static void cJSON_CopyFields(cJSON *dst, cJSON *src)
{
  struct cJSON_DataNext *nextdata;
  struct cJSON_DataNext *newnextdata;
  size_t copysize;
  size_t objsize;
  void *data;
  void *newdata;
  void *name;
  void *newname;

  objsize = cJSON_GetFieldPointers(src, &nextdata, NULL, &data, NULL,
                                   &name, NULL);
  (void)cJSON_GetFieldPointers(dst, &newnextdata, NULL, &newdata, NULL,
                               &newname, NULL);

  if (newnextdata)
    {
      newnextdata->next = nextdata ? nextdata->next : NULL;
    }

  if (!data)
    {
      DEBUGASSERT(!newdata);

      if (!name)
        {
          DEBUGASSERT(!newname);
        }
      else
        {
          DEBUGASSERT(newname);

          copysize = ((uint8_t *)src + objsize) - (uint8_t *)name;
          memmove(newname, name, copysize);
        }
    }
  else
    {
      DEBUGASSERT(newdata);

      copysize = ((uint8_t *)src + objsize) - (uint8_t *)data;
      memmove(newdata, data, copysize);
    }
}

/* Utility for adding item at the end of array list. */

static bool suffix_object(cJSON *array, cJSON *prev, cJSON *item)
{
  if (cJSON_IsPacked(prev))
    {
      struct cJSON_DataArray *adata = cJSON_ArrayField(array);
      struct cJSON *pack;
      size_t packsize;
      size_t itemsize;
      uint8_t next_type;

      DEBUGASSERT(adata && adata->child);

      /* 'prev' needs to be last item of array. */

      DEBUGASSERT((prev->info & JSON_NEXT_MASK) == JSON_NEXT_NULL);

      /* Current pack-size. */

      pack = adata->child;
      packsize = cJSON_PackedArraySize(pack);

      /* Item size in packed form. */

      itemsize = cJSON_GetFieldPointers(item, NULL, &next_type, NULL, NULL,
                                        NULL, NULL);
      if (next_type == JSON_NEXT_PTR)
        {
          itemsize -= sizeof(struct cJSON_DataNext);
        }

      /* Grow packed array. */

      adata->child = realloc(pack, packsize + itemsize);
      if (!adata->child)
        {
          adata->child = pack;
          return false;
        }

      memset((uint8_t *)adata->child + packsize, 0, itemsize);

      /* Adjust 'prev' pointer to new allocation area. */

      prev = (cJSON *)((uint8_t *)prev + ((uint8_t *)adata->child - (uint8_t *)pack));

      /* Setup new last item. */

      pack = (cJSON *)((uint8_t *)adata->child + packsize);

      pack->info = item->info & ~JSON_NEXT_MASK;
      pack->info |= JSON_NEXT_NULL;

      cJSON_CopyFields(pack, item);

      /* Change previous item next type. */

      prev->info &= ~JSON_NEXT_MASK;
      prev->info |= JSON_NEXT_MEM;

      cJSON_free(item);
    }
  else
    {
      struct cJSON_DataNext *prev_next = NULL;
      uint8_t next_type = 0;

      (void)cJSON_GetFieldPointers(prev, &prev_next, &next_type, NULL, NULL,
                                   NULL, NULL);

      /* Caller needs to make sure that prev is not in packed format. */

      DEBUGASSERT(next_type == JSON_NEXT_PTR);
      DEBUGASSERT(prev_next);

      prev_next->next = item;
    }

  return true;
}

/* Get allocation size of cJSON object. */

static size_t GetAllocMemSize(cJSON *item, bool check_next)
{
  struct mm_allocnode_s *node;
  size_t total = 0;
  cJSON *c;

  if (!item)
    {
      return total;
    }

  if (cJSON_IsPacked(item))
    {
      size_t allocsize;

      if (check_next)
        {
          /* Get allocated memory for item. */

          node = (void *)((char *)item - SIZEOF_MM_ALLOCNODE);
          total += node->size;
        }
      else
        {
          /* Cannot know if this is first item in packed array. Do not try to
           * check node structure, instead add allocation overhead. */

          allocsize = cJSON_GetFieldPointers(item, NULL, NULL, NULL, NULL, NULL,
                                             NULL);
          total += MM_ALIGN_UP(allocsize + SIZEOF_MM_ALLOCNODE);
        }

      do
        {
          /* Check if item has child object. If has, get its size. */

          c = cJSON_child(item);
          if (c)
            {
              total += GetAllocMemSize(c, true);
            }

          if (!check_next)
            {
              break;
            }

          /* Check next item. */

          item = cJSON_next(item);
        }
      while (item);
    }
  else
    {
      do
        {
          /* Get allocated memory for item. */

          node = (void *)((char *)item - SIZEOF_MM_ALLOCNODE);
          total += node->size;

          /* Check if item has child object. If has, get its size. */

          c = cJSON_child(item);
          if (c)
            {
              total += GetAllocMemSize(c, true);
            }

          if (!check_next)
            {
              break;
            }

          /* Check next item. */

          item = cJSON_next(item);
        }
      while (item);
    }

  return total;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

const char *cJSON_GetErrorPtr(void)
{
  return "";
}

/* Set the name of object. */

cJSON *cJSON_SetItemName(cJSON *item, const char *name)
{
  return cJSON_ChangeName(item, name);
}

/* Check if item is in packed format. */

bool cJSON_IsPacked(cJSON *item)
{
  uint8_t next_type = item->info & JSON_NEXT_MASK;

  return next_type != JSON_NEXT_PTR;
}

/* Convert array of items to packed storage form. */

cJSON *cJSON_PackArray(cJSON *item)
{
  size_t packsize;
  size_t objsize;
  cJSON *pack;
  cJSON *curr;
  cJSON *dst;

  if (!item || cJSON_IsPacked(item))
    {
      return item;
    }

  /* Calculate allocation size. */

  packsize = 0;
  curr = item;
  do
    {
      uint8_t next_type;

      objsize = cJSON_GetFieldPointers(curr, NULL, &next_type, NULL, NULL,
                                       NULL, NULL);
      DEBUGASSERT(next_type == JSON_NEXT_PTR);

      /* Remove size of next_field, not used in packed format. */

      objsize -= sizeof(struct cJSON_DataNext);

      packsize += objsize;
      curr = cJSON_next(curr);
    }
  while (curr);

  /* Allocate pack. */

  pack = calloc(1, packsize);
  if (!pack)
    {
      return NULL;
    }

  /* Copy unpacked array items to packed array. */

  curr = item;
  dst = pack;
  do
    {
      cJSON *next;

      /* Setup info field. */

      dst->info = curr->info & ~JSON_NEXT_MASK;
      dst->info |= JSON_NEXT_MEM;

      /* Copy data & name fields. */

      cJSON_CopyFields(dst, curr);

      /* Get next, free current. */

      next = cJSON_next(curr);
      cJSON_free(curr);
      curr = next;

      /* Adjust info field if this is last item of array. */

      if (!curr)
        {
          dst->info &= ~JSON_NEXT_MASK;
          dst->info |= JSON_NEXT_NULL;
        }
      else
        {
          dst = cJSON_next(dst);
        }
    }
  while (curr);

  return pack;
}

/* Convert child object of array/object to packed storage form. */

void cJSON_PackChild(cJSON *array)
{
  struct cJSON_DataArray *data;
  cJSON *c;

  if (!array)
    {
      return;
    }

  data = cJSON_ArrayField(array);
  if (!data || !data->child)
    {
      return;
    }

  c = cJSON_PackArray(data->child);
  if (c)
    {
      data->child = c;
    }
}

/* Convert packed array of items to unpacked storage form. */

cJSON *cJSON_UnpackArray(cJSON *item)
{
  cJSON *newitem = NULL;
  cJSON *prev = NULL;
  cJSON *newcurr;
  cJSON *curr;

  if (!item || !cJSON_IsPacked(item))
    {
      return item;
    }

  curr = item;

  do
    {
      newcurr = cJSON_MakeUnpackedCopy(curr);
      if (!newcurr)
        {
          cJSON_Delete(newitem);
          return NULL;
        }

      if (!newitem)
        {
          newitem = newcurr;
        }

      if (prev)
        {
          cJSON_NextField(prev)->next = newcurr;
        }

      prev = newcurr;
      newcurr = cJSON_next(prev);

      curr = cJSON_next(curr);
    }
  while (curr);

  cJSON_Delete(item);
  return newitem;
}

/* Delete a cJSON structure. */

void cJSON_Delete(cJSON *c)
{
  cJSON *next;

  if (!c)
    {
      return;
    }

  if (cJSON_IsPacked(c))
    {
      cJSON *first = c;

      while (c)
        {
          cJSON_Delete(cJSON_child(c)); /* No-op if child is NULL. */
          c = cJSON_next(c);
        }

      cJSON_free(first);
    }

  while (c)
    {
      next = cJSON_next(c);

      cJSON_Delete(cJSON_child(c)); /* No-op if child is NULL. */
      cJSON_free(c);

      c = next;
    }
}

/* Get Array size/item / object item. */

int cJSON_GetArraySize(cJSON *array)
{
  cJSON *c = cJSON_child(array);
  int i = 0;

  while (c)
    {
      i++;
      c = cJSON_next(c);
    }

  return i;
}

cJSON *cJSON_GetArrayItem(cJSON *array, int item)
{
  cJSON *c = cJSON_child(array);

  while (c && item > 0)
    {
      item--;
      c = cJSON_next(c);
    }

  return c;
}

cJSON *cJSON_GetObjectItem(cJSON *object, const char *string)
{
  cJSON *c = cJSON_child(object);

  while (c && cJSON_strcasecmp(cJSON_GetNameString(c), string))
    {
      c = cJSON_next(c);
    }

  return c;
}

/* Add item to array/object. */
bool cJSON_AddItemToArray(cJSON *array, cJSON *item)
{
  cJSON *c = cJSON_child(array);

  if (!item)
    {
      return true;
    }

  if (!c)
    {
      if (cJSON_ArrayField(array))
        {
          cJSON_ArrayField(array)->child = item;
        }
    }
  else
    {
      while (c && cJSON_next(c))
        {
          c = cJSON_next(c);
        }

      return suffix_object(array, c, item);
    }

  return true;
}

bool cJSON_AddItemToObject(cJSON *object, const char *string, cJSON *item)
{
  if (!item)
    {
      return true;
    }

  item = cJSON_ChangeName(item, string);
  if (!item)
    {
      return false;
    }

  return cJSON_AddItemToArray(object, item);
}

cJSON *cJSON_DetachItemFromArray(cJSON *array, int which)
{
  cJSON *c = cJSON_child(array);
  cJSON *prev = NULL;

  while (c && which > 0)
    {
      prev = c;
      c = cJSON_next(c), which--;
    }

  if (!c)
    {
      return 0;
    }

  if (cJSON_IsPacked(c))
    {
      cJSON *copy;
      size_t tailsize;
      uint8_t next_type;

      copy = cJSON_MakeUnpackedCopy(c);

      /* Item is in packed format. Detach procedure makes non-packed copy
       * of item, and moves tail of memory area over the detached object. */

      next_type = c->info & JSON_NEXT_MASK;
      if (next_type == JSON_NEXT_NULL)
        {
          if (prev)
            {
              /* This item is the last of array. Mark previous as last
               * instead. */

              prev->info &= ~JSON_NEXT_MASK;
              prev->info |= JSON_NEXT_NULL;
            }

          if (c == cJSON_child(array))
            {
              DEBUGASSERT(!prev);

              /* First and last item. Free memory. */

              cJSON_free(c);
              cJSON_ArrayField(array)->child = NULL;
            }
        }
      else if (next_type == JSON_NEXT_MEM)
        {
          cJSON *next;

          /* Get length of packed array tail. */

          next = cJSON_next(c);
          DEBUGASSERT(next);

          tailsize = cJSON_PackedArraySize(next);

          memmove(c, next, tailsize);
        }
      else
        {
          DEBUGASSERT(false);
        }

      /* Trim memory. */

      c = cJSON_ArrayField(array)->child;
      if (c)
        {
          tailsize = cJSON_PackedArraySize(c);
          DEBUGASSERT(tailsize > 0);

          cJSON_ArrayField(array)->child = realloc(c, tailsize);
          if (!cJSON_ArrayField(array)->child)
            {
              /* realloc failed, restore old. */

              cJSON_ArrayField(array)->child = c;
            }
        }

      return copy;
    }
  else
    {
      if (prev)
        {
          cJSON_NextField(prev)->next = cJSON_next(c);
        }

      if (c == cJSON_child(array))
        {
          cJSON_ArrayField(array)->child = cJSON_next(c);
        }

      cJSON_NextField(c)->next = NULL;
      return c;
    }
}

void cJSON_DeleteItemFromArray(cJSON *array, int which)
{
  cJSON_Delete(cJSON_DetachItemFromArray(array, which));
}

cJSON *cJSON_DetachItemFromObject(cJSON *object, const char *string)
{
  int i = 0;
  cJSON *c = cJSON_child(object);

  while (c && cJSON_strcasecmp(cJSON_GetNameString(c), string))
    {
      i++;
      c = cJSON_next(c);
    }

  if (c)
    {
      return cJSON_DetachItemFromArray(object, i);
    }

  return NULL;
}

void cJSON_DeleteItemFromObject(cJSON *object, const char *string)
{
  cJSON_Delete(cJSON_DetachItemFromObject(object, string));
}

/* Replace array/object items with new ones. */

bool cJSON_ReplaceItemInArray(cJSON *array, int which, cJSON *newitem)
{
  cJSON *c = cJSON_child(array);
  cJSON *prev = NULL;

  if (c && cJSON_IsPacked(c))
    {
      /* TODO: Replacing item in packed array. */

      cJSON *newc;

      /* Child array is in packed format, unpack before modifying. */

      newc = cJSON_UnpackArray(c);
      if (!newc)
        {
          return false;
        }

      c = newc;
      cJSON_ArrayField(array)->child = c;
    }

  while (c && which > 0)
    {
      prev = c;
      c = cJSON_next(c), which--;
    }

  if (!c)
    {
      return true;
    }

  cJSON_NextField(newitem)->next = cJSON_next(c);

  if (c == cJSON_child(array))
    {
      /* 'c' is first item of array, c->next and prev might be NULL. */

      cJSON_ArrayField(array)->child = newitem;
    }
  else
    {
      /* 'c' is not first item of array, prev should not be NULL. */

      DEBUGASSERT(prev != NULL);

      cJSON_NextField(prev)->next = newitem;
    }

  cJSON_NextField(c)->next = NULL;
  cJSON_Delete(c);

  return true;
}

bool cJSON_ReplaceItemInObject(cJSON *object, const char *string,
                               cJSON *newitem)
{
  int i = 0;
  cJSON *c = cJSON_child(object);

  while (c && cJSON_strcasecmp(cJSON_GetNameString(c), string))
    {
      i++;
      c = cJSON_next(c);
    }

  if (c)
    {
      newitem = cJSON_ChangeName(newitem, string);
      if (!newitem)
        {
          return false;
        }

      return cJSON_ReplaceItemInArray(object, i, newitem);
    }

  return true;
}

static void cJSON_SetName(cJSON *item, const char *name)
{
  uint8_t name_type;
  void *name_field;

  (void)cJSON_GetFieldPointers(item, NULL, NULL, NULL, NULL, &name_field,
                               &name_type);

  if (!name)
    {
      DEBUGASSERT(name_type == JSON_NAME_EMPTY);
    }

  if (name_type == JSON_NAME_SHORT)
    {
      struct cJSON_DataStringShort *ss = name_field;
      size_t name_len = strlen(name);

      memmove(ss->valuestring, name, name_len + 1);
      ss->string_size = name_len + 1;
    }
  else if (name_type == JSON_NAME_LONG)
    {
      struct cJSON_DataStringLong *sl = name_field;
      size_t name_len = strlen(name);

      memmove(sl->valuestring, name, name_len + 1);
      sl->string_size = name_len + 1;
    }
}

/* Create named basic types: */

cJSON *cJSON_CreateNamedNull(const char *name)
{
  size_t fields_size;
  uint8_t nametype;
  cJSON *item;

  /* Unpacked format (next pointer), empty name. */

  fields_size = cJSON_CalculateNextFieldSize(JSON_NEXT_PTR);
  fields_size += cJSON_CalculateNameFieldSize(name, &nametype);

  item = cJSON_New_Item(JSON_DATA_NULL | JSON_NEXT_PTR | nametype, fields_size);
  if (item)
    {
      cJSON_SetName(item, name);
    }

  return item;
}

cJSON *cJSON_CreateNamedBool(const char *name, bool b)
{
  uint8_t data_type = b ? JSON_DATA_TRUE : JSON_DATA_FALSE;
  size_t fields_size;
  uint8_t nametype;
  cJSON *item;

  /* Unpacked format (next pointer), empty name. */

  fields_size = cJSON_CalculateNextFieldSize(JSON_NEXT_PTR);
  fields_size += cJSON_CalculateNameFieldSize(name, &nametype);

  item = cJSON_New_Item(data_type | JSON_NEXT_PTR | nametype, fields_size);
  if (item)
    {
      cJSON_SetName(item, name);
    }

  return item;
}

cJSON *cJSON_CreateNamedTrue(const char *name)
{
  return cJSON_CreateNamedBool(name, true);
}

cJSON *cJSON_CreateNamedFalse(const char *name)
{
  return cJSON_CreateNamedBool(name, false);
}

cJSON *cJSON_CreateNamedNumber(const char *name, double num)
{
  uint8_t data_type = 0;
  size_t fields_size;
  uint8_t nametype;
  cJSON *item;

  /* Unpacked format (next pointer), empty name. */

  fields_size = cJSON_CalculateNextFieldSize(JSON_NEXT_PTR);
  fields_size += cJSON_CalculateDataNumberFieldSize(num, &data_type);
  fields_size += cJSON_CalculateNameFieldSize(name, &nametype);

  item = cJSON_New_Item(data_type | JSON_NEXT_PTR | nametype, fields_size);
  if (item)
    {
      cJSON_SetValueNumber(item, num);
      cJSON_SetName(item, name);
    }

  return item;
}

cJSON *cJSON_CreateNamedString(const char *name, const char *string)
{
  size_t string_len = strlen(string);
  uint8_t data_type = 0;
  size_t fields_size;
  uint8_t nametype;
  cJSON *item;

  /* Unpacked format (next pointer), empty name. */

  fields_size = cJSON_CalculateNextFieldSize(JSON_NEXT_PTR);
  fields_size += cJSON_CalculateDataStringFieldSize(string_len, &data_type);
  fields_size += cJSON_CalculateNameFieldSize(name, &nametype);

  item = cJSON_New_Item(data_type | JSON_NEXT_PTR | nametype, fields_size);
  if (item)
    {
      cJSON_SetValueBuffer(item, string, string_len + 1);
      cJSON_SetName(item, name);
    }

  return item;
}

cJSON *cJSON_CreateNamedBuffer(const char *name, const void *buf, size_t len)
{
  size_t fields_size;
  uint8_t nametype;
  cJSON *item;

  /* Unpacked format (next pointer), empty name. */

  fields_size = cJSON_CalculateNextFieldSize(JSON_NEXT_PTR);
  fields_size += cJSON_CalculateDataBufferFieldSize(len);
  fields_size += cJSON_CalculateNameFieldSize(name, &nametype);

  item = cJSON_New_Item(JSON_DATA_BUFFER | JSON_NEXT_PTR | nametype,
                        fields_size);
  if (item)
    {
      cJSON_SetValueBuffer(item, buf, len);
      cJSON_SetName(item, name);
    }

  return item;
}

cJSON *cJSON_CreateNamedArray(const char *name)
{
  size_t fields_size;
  uint8_t nametype;
  cJSON *item;

  /* Unpacked format (next pointer), empty name. */

  fields_size = cJSON_CalculateNextFieldSize(JSON_NEXT_PTR);
  fields_size += sizeof(struct cJSON_DataArray);
  fields_size += cJSON_CalculateNameFieldSize(name, &nametype);

  item = cJSON_New_Item(JSON_DATA_ARRAY | JSON_NEXT_PTR | nametype,
                        fields_size);
  if (item)
    {
      cJSON_SetArrayData(item, NULL);
      cJSON_SetName(item, name);
    }

  return item;
}

cJSON *cJSON_CreateNamedObject(const char *name)
{
  cJSON *item = cJSON_CreateNamedArray(name);

  if (item)
    {
      item->info &= ~JSON_DATA_MASK;
      item->info |= JSON_DATA_OBJECT;
    }

  return item;
}

/* Create Arrays: */

cJSON *cJSON_CreateIntArray(const int *numbers, int count)
{
  cJSON *n = NULL;
  cJSON *p = NULL;
  cJSON *a = cJSON_CreateArray();
  int i;

  for (i = 0; a && i < count; i++)
    {
      n = cJSON_CreateNumber(numbers[i]);
      if (i == 0)
        {
          cJSON_ArrayField(a)->child = n;
        }
      else
        {
          if (!suffix_object(a, p, n))
            {
              cJSON_Delete(a);
              cJSON_Delete(n);
              return NULL;
            }
        }

      p = n;
    }

  return a;
}

cJSON *cJSON_CreateFloatArray(const float *numbers, int count)
{
  cJSON *n = NULL;
  cJSON *p = NULL;
  cJSON *a = cJSON_CreateArray();
  int i;

  for (i = 0; a && i < count; i++)
    {
      n = cJSON_CreateNumber(numbers[i]);
      if (i == 0)
        {
          cJSON_ArrayField(a)->child = n;
        }
      else
        {
          if (!suffix_object(a, p, n))
            {
              cJSON_Delete(a);
              cJSON_Delete(n);
              return NULL;
            }
        }

      p = n;
    }

  return a;
}

cJSON *cJSON_CreateDoubleArray(const double *numbers, int count)
{
  cJSON *n = NULL;
  cJSON *p = NULL;
  cJSON *a = cJSON_CreateArray();
  int i;

  for (i = 0; a && i < count; i++)
    {
      n = cJSON_CreateNumber(numbers[i]);
      if (i == 0)
        {
          cJSON_ArrayField(a)->child = n;
        }
      else
        {
          if (!suffix_object(a, p, n))
            {
              cJSON_Delete(a);
              cJSON_Delete(n);
              return NULL;
            }
        }

      p = n;
    }

  return a;
}

cJSON *cJSON_CreateStringArray(const char **strings, int count)
{
  cJSON *n = NULL;
  cJSON *p = NULL;
  cJSON *a = cJSON_CreateArray();
  int i;

  for (i = 0; a && i < count; i++)
    {
      n = cJSON_CreateString(strings[i]);
      if (i == 0)
        {
          cJSON_ArrayField(a)->child = n;
        }
      else
        {
          if (!suffix_object(a, p, n))
            {
              cJSON_Delete(a);
              cJSON_Delete(n);
              return NULL;
            }
        }

      p = n;
    }

  return a;
}

/* Get allocation size of cJSON object. */

size_t cJSON_GetAllocMemSize(cJSON *item)
{
  return GetAllocMemSize(item, false);
}

/* Return type of cJSON object. */

enum cJSON_type_e cJSON_type(const cJSON *item)
{
  if (!item)
    return cJSON_NULL;

  return cJSON_InfoToType(item->info);
}

/* Return name of cJSON object. */

const char *cJSON_name(const cJSON *item)
{
  if (!item)
    return "";

  return cJSON_GetNameString((cJSON *)item);
}

/* Return string value of cJSON object. */

const char *cJSON_string(const cJSON *item)
{
  int type = cJSON_type(item);

  if (type != cJSON_String)
    {
      return "";
    }
  else
    {
      struct cJSON_buffer_s buf = cJSON_GetValueBuffer((cJSON *)item);

      return buf.ptr;
    }
}

/* Return buffer value of cJSON object. */

struct cJSON_buffer_s cJSON_buffer(const cJSON *item)
{
  int type = cJSON_type(item);

  if (type == cJSON_Buffer || type == cJSON_String)
    {
      return cJSON_GetValueBuffer((cJSON *)item);
    }
  else
    {
      struct cJSON_buffer_s ret =
        {
          .len = 0,
          .ptr = NULL
        };
      return ret;
    }
}

/* Return integer value of cJSON object. */

int cJSON_int(const cJSON *item)
{
  int type = cJSON_type(item);

  if (type != cJSON_Number)
    return 0;

  return cJSON_GetValueInt((cJSON *)item);
}

/* Return floating-point value of cJSON object. */

double cJSON_double(const cJSON *item)
{
  int type = cJSON_type(item);

  if (type != cJSON_Number)
    return 0.0;

  return cJSON_GetValueDouble((cJSON *)item);
}

/* Return boolean value of cJSON object. */

bool cJSON_boolean(const cJSON *item)
{
  int type = cJSON_type(item);

  switch (type)
    {
    default:
    case cJSON_False:
    case cJSON_NULL:
      return false;

    case cJSON_True:
      return true;

    case cJSON_String:
      return cJSON_string(item)[0] != 0; /* not empty */

    case cJSON_Number:
      return cJSON_int(item) != 0;

    case cJSON_Array:
    case cJSON_Object:
      return cJSON_child((cJSON *)item) != NULL; /* has child */
    }
}

/* Return child object of cJSON object. */

cJSON *cJSON_child(cJSON *item)
{
  const struct cJSON_DataArray *data;

  if (!item)
    return NULL;

  data = cJSON_ArrayField(item);
  if (!data)
    return NULL;

  return data->child;
}

/* Return next object of cJSON object. */

cJSON *cJSON_next(cJSON *item)
{
  if (!item)
    return NULL;

  return cJSON_GetNextPointer(item);
}
