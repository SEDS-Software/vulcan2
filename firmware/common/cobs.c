/*
 * Copyright (c) 2019 Alex Forencich
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
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "cobs.h"

size_t cobs_encode(const uint8_t *ptr, size_t len, uint8_t *dst)
{
  const uint8_t *start = dst;
  const uint8_t *end = ptr + len;
  uint8_t code = 1;
  uint8_t *code_ptr = dst++;

  while (ptr < end)
  {
    if (code != 0xff)
    {
      uint8_t c = *ptr++;
      if (c != 0)
      {
        *dst++ = c;
        code++;
        continue;
      }
    }
    *code_ptr = code;
    code_ptr = dst++;
    code = 1;
  }

  *code_ptr = code;
  return dst - start;
}

size_t cobs_decode(const uint8_t *ptr, size_t len, uint8_t *dst)
{
  const uint8_t *start = dst;
  const uint8_t *end = ptr + len;
  uint8_t code = 0xff;
  uint8_t copy = 0;

  for (; ptr < end; copy--)
  {
    if (copy != 0)
    {
      *dst++ = *ptr++;
    }
    else
    {
      if (code != 0xff)
      {
        *dst++ = 0;
      }
      copy = code = *ptr++;
      if (code == 0)
      {
        break;
      }
    }
  }
  return dst - start;
}
