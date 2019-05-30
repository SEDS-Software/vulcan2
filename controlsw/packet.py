"""

Copyright (c) 2019 Alex Forencich

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

"""
import array
import struct
import sys

import crc16

packet_types = {}


def register(cls, ptype):
    if ptype in packet_types:
        raise Exception("ptype already registered")
    assert issubclass(cls, Packet)
    packet_types[ptype] = cls


def parse(data):
    pkt = Packet()
    if not pkt.parse(data):
        return None

    if pkt.ptype in packet_types:
        pkt = packet_types[pkt.ptype](pkt)
        pkt.parse()
        return pkt

    return pkt


class Packet(object):
    def __init__(self, payload=b'', dest=0xff, source=0x00, flags=0x00, ptype=0x00):
        self.payload = payload
        self.dest = dest
        self.source = source
        self.flags = flags
        self.ptype = ptype

        if isinstance(payload, Packet):
            self.payload = bytearray(payload.payload)
            self.dest = payload.dest
            self.source = payload.source
            self.flags = payload.flags
            self.ptype = payload.ptype

    def build(self):
        data = struct.pack('BBBB', self.dest, self.source, self.flags, self.ptype)

        data += self.payload

        data += struct.pack('<H', crc16.crc16(data))

        return data

    def parse(self, data):
        data = bytearray(data)

        if len(data) < 6:
            return None

        self.dest, self.source, self.flags, self.ptype = struct.unpack_from('BBBB', data)

        self.payload = data[4:-2]

        return crc16.crc16(data) == 0

    def __eq__(self, other):
        if isinstance(payload, Packet):
            return (self.dest == other.dest and
                self.source == other.source and
                self.flags == other.flags and
                self.ptype == other.ptype and
                self.payload == other.payload)
        return False

    def __repr__(self):
        return '%s(payload=%s, dest=0x%x, source=0x%x, flags=0x%x, ptype=%d)' % (type(self).__name__, repr(self.payload), self.dest, self.source, self.flags, self.ptype)


class PingRequestPacket(Packet):
    def __init__(self, payload=b'', dest=0xff, source=0x00, flags=0x00, ptype=0xfe):
        super(PingRequestPacket, self).__init__(payload, dest, source, flags, ptype)

register(PingRequestPacket, 0xfe)


class PingResponsePacket(Packet):
    def __init__(self, payload=b'', dest=0xff, source=0x00, flags=0x00, ptype=0xff):
        super(PingResponsePacket, self).__init__(payload, dest, source, flags, ptype)

register(PingResponsePacket, 0xff)


class DIOStatePacket(Packet):
    def __init__(self, payload=b'', dest=0xff, source=0x00, flags=0x00, ptype=0x10):
        super(DIOStatePacket, self).__init__(payload, dest, source, flags, ptype)

        self.bank = 0
        self.state = 0
        self.width = 2

    def build(self):
        self.payload = struct.pack('B', self.bank) + self.state.to_bytes(self.width, 'little')

        return super(DIOStatePacket, self).build()

    def parse(self, data=None):
        if data is not None:
            super(DIOStatePacket, self).parse(data)

        self.width = len(self.payload)-1
        self.bank = struct.unpack_from('B', self.payload, 0)[0]
        self.state = int.from_bytes(self.payload[1:], 'little')

register(DIOStatePacket, 0x10)


class DIOSetBitPacket(Packet):
    def __init__(self, payload=b'', dest=0xff, source=0x00, flags=0x00, ptype=0x11):
        super(DIOSetBitPacket, self).__init__(payload, dest, source, flags, ptype)

        self.bank = 0
        self.bit = 0
        self.state = 0

    def build(self):
        self.payload = struct.pack('BBB', self.bank, self.bit, 1 if self.state else 0)

        return super(DIOSetBitPacket, self).build()

    def parse(self, data=None):
        if data is not None:
            super(DIOSetBitPacket, self).parse(data)

        self.bank, self.bit, self.state = struct.unpack('BBB', self.payload)

register(DIOSetBitPacket, 0x11)


class AnalogValuePacket(Packet):
    def __init__(self, payload=b'', dest=0xff, source=0x00, flags=0x00, ptype=0x20):
        super(AnalogValuePacket, self).__init__(payload, dest, source, flags, ptype)

        self.bank = 0
        self.type = 0
        self.values = [0]

    def build(self):
        a = array.array('H', list(self.values))
        if sys.byteorder == 'big':
            a.byteswap()
        self.payload = struct.pack('BB', self.bank, self.type) + a.tobytes()

        return super(AnalogValuePacket, self).build()

    def parse(self, data=None):
        if data is not None:
            super(AnalogValuePacket, self).parse(data)

        self.bank, self.type = struct.unpack_from('BB', self.payload, 0)
        self.values = array.array('H', self.payload[2:])
        if sys.byteorder == 'big':
            self.values.byteswap()

register(AnalogValuePacket, 0x20)

