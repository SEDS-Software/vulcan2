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

#include "ms56xx.h"

uint8_t ms56xx_prom_crc(struct ms56xx *m)
{
  uint32_t n_rem = 0x00000000;
  for (uint8_t cnt = 0; cnt < 8; cnt++)
  {
    n_rem ^= m->c[cnt];
    if (cnt == 7)
      n_rem &= 0x00ffff00; // mask off CRC
    for (uint8_t n_bit = 0; n_bit < 16; n_bit++)
    {
      if (n_rem & 0x800000)
        n_rem = (n_rem << 1) ^ 0x300000;
      else
        n_rem = n_rem << 1;
    }
  }
  return (n_rem >> 20) & 0x000f;
}

int ms56xx_check_prom_crc(struct ms56xx *m)
{
  return ms56xx_prom_crc(m) == (m->c[7] & 0x000f);
}

void ms56xx_convert_temp(struct ms56xx *m, int32_t *temp)
{
  int32_t dt = *temp - ((int32_t)m->c[5] << 8);
  *temp = 2000 + (((int64_t)dt * (int64_t)m->c[6]) >> 23);
}

void ms56xx_convert(struct ms56xx *m, int32_t *p, int32_t *temp)
{
  int32_t dt = *temp - ((int32_t)m->c[5] << 8);
  *temp = 2000 + (((int64_t)dt * (int64_t)m->c[6]) >> 23);
  int64_t off = ((int64_t)m->c[2] << 17) + (((int64_t)m->c[4] * (int64_t)dt) >> 6);
  int64_t sens = ((int64_t)m->c[1] << 16) + (((int64_t)m->c[3] * (int64_t)dt) >> 7);
  *p = (((*p * sens) >> 21) - off) >> 15;
}

void ms56xx_convert_2(struct ms56xx *m, int32_t *p, int32_t *temp)
{
  int32_t dt = *temp - ((int32_t)m->c[5] << 8);
  *temp = 2000 + (((int64_t)dt * (int64_t)m->c[6]) >> 23);
  int64_t off = ((int64_t)m->c[2] << 17) + (((int64_t)m->c[4] * (int64_t)dt) >> 6);
  int64_t sens = ((int64_t)m->c[1] << 16) + (((int64_t)m->c[3] * (int64_t)dt) >> 7);
  
  if (*temp < 2000)
  {
    int32_t t2 = ((int64_t) dt * (int64_t) dt) >> 31;
    int64_t temp2 = *temp - 2000;
    temp2 = temp2 * temp2;
    int64_t off2 = (61 * temp2) >> 4;
    int64_t sens2 = temp2 << 1;

    if (*temp < -1500)
    {
      temp2 = *temp + 1500;
      temp2 = temp2 * temp2;
      off2 += 15 * temp2;
      sens2 += temp2 << 3;
    }

    *temp -= t2;
    off -= off2;
    sens -= sens2;
  }

  *p = (((*p * sens) >> 21) - off) >> 15;
}

