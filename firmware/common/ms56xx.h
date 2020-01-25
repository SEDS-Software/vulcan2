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

#ifndef MS56XX_H_
#define MS56XX_H_

#include <stdint.h>

#define MS56XX_RESET        0x1E
#define MS56XX_CONV_D1_256  0x40
#define MS56XX_CONV_D1_512  0x42
#define MS56XX_CONV_D1_1024 0x44
#define MS56XX_CONV_D1_2048 0x46
#define MS56XX_CONV_D1_4096 0x48
#define MS56XX_CONV_D2_256  0x50
#define MS56XX_CONV_D2_512  0x52
#define MS56XX_CONV_D2_1024 0x54
#define MS56XX_CONV_D2_2048 0x56
#define MS56XX_CONV_D2_4096 0x58
#define MS56XX_ADC_READ     0x00
#define MS56XX_PROM_READ    0xA0

struct ms56xx
{
  uint16_t c[8];
};

uint8_t ms56xx_prom_crc(struct ms56xx *m);
int ms56xx_check_prom_crc(struct ms56xx *m);
void ms56xx_convert(struct ms56xx *m, int32_t *p, int32_t *temp);
void ms56xx_convert_2(struct ms56xx *m, int32_t *p, int32_t *temp);

#endif /* MS56XX_H_ */
