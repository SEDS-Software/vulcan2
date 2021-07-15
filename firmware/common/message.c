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

#include <string.h>

#include "message.h"
#include "crc16.h"

int parse_message(struct message *msg, uint8_t *buffer, int len)
{
  // check length
  if (len < MSG_OVERHEAD)
    return -1;

  // check CRC
  if (crc16_block(buffer, len) != 0)
    return -1;

  msg->dest    = buffer[0];
  msg->src     = buffer[1];
  msg->seq     = buffer[2];
  msg->flags   = buffer[3];
  msg->ptype   = buffer[4];
  msg->tx_mask = MSG_TX_NONE;
  msg->len     = len-MSG_OVERHEAD;
  memcpy(&msg->data, &buffer[MSG_HDR_SIZE], msg->len);

  return 0;
}

int pack_message(struct message *msg, uint8_t *buffer, int len)
{
  uint16_t crc;

  if (len < msg->len+MSG_OVERHEAD)
    return -1;

  buffer[0] = msg->dest;
  buffer[1] = msg->src;
  buffer[2] = msg->seq;
  buffer[3] = msg->flags;
  buffer[4] = msg->ptype;
  memcpy(&buffer[MSG_HDR_SIZE], &msg->data, msg->len);

  crc = crc16_block(buffer, msg->len+MSG_HDR_SIZE);

  buffer[msg->len+MSG_HDR_SIZE] = crc & 0xff;
  buffer[msg->len+MSG_HDR_SIZE+1] = crc >> 8;

  return 0;
}
