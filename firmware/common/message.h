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

#ifndef MESSAGE_H_
#define MESSAGE_H_

#include <stdint.h>

#define MSG_HDR_SIZE 5
#define MSG_CRC_SIZE 2
#define MSG_MAX_PAYLOAD 64
#define MSG_OVERHEAD (MSG_HDR_SIZE+MSG_CRC_SIZE)

#define MSG_DEST_BCAST 0xff

#define MSG_FLAG_RADIO 0x10

#define MSG_TYPE_COMMAND 0x08
#define MSG_TYPE_DIO_STATE 0x10
#define MSG_TYPE_DIO_SET_BIT 0x11
#define MSG_TYPE_ANALOG_VALUE 0x20
#define MSG_TYPE_PING_REQ 0xfe
#define MSG_TYPE_PING_RESP 0xff

#define MSG_TX_MASK(x) (1 << (x))
#define MSG_INV_TX_MASK(x) ((~(1 << (x))) & 0xff)

#define MSG_RX_NONE 0xff

#define MSG_TX_ALL 0xff
#define MSG_TX_NONE 0x00

struct message
{
  uint8_t dest;
  uint8_t src;
  uint8_t seq;
  uint8_t flags;
  uint8_t ptype;
  uint16_t len;
  uint8_t rx_int;
  uint8_t tx_mask;
  char data[MSG_MAX_PAYLOAD];
};

int parse_message(struct message *msg, uint8_t *buffer, int len);
int pack_message(struct message *msg, uint8_t *buffer, int len);

int register_message(struct message *msg);
int is_duplicate_message(struct message *msg);

#endif /* MESSAGE_H_ */
