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

import serial
import struct

packet_types = {}


def register(cls, cmdid):
    if cmdid in packet_types:
        raise Exception("cmdid already registered")
    assert issubclass(cls, Packet)
    packet_types[cmdid] = cls


def parse(data):
    pkt = Packet()
    pkt.parse(data)

    if pkt.cmdid in packet_types:
        pkt = packet_types[pkt.cmdid](pkt)
        pkt.parse()
        return pkt

    return pkt


class Packet(object):
    def __init__(self, payload=b'', cmdid=0):
        self.payload = payload
        self.cmdid = cmdid

        if isinstance(payload, Packet):
            self.payload = bytearray(payload.payload)
            self.cmdid = payload.cmdid

    def build(self):
        data = struct.pack('>BHB', 0x7e, len(self.payload)+1, self.cmdid)

        data += self.payload

        data += struct.pack('B', 0xff-(sum(data[3:]) & 0xff))

        return data

    def parse(self, data):
        data = bytearray(data)

        assert data[0] == 0x7e

        length, self.cmdid = struct.unpack_from('>HB', data, 1)

        self.payload = data[4:4+length-1]

        return sum(data[3:]) & 0xff == 0xff

    def __eq__(self, other):
        if isinstance(other, Packet):
            return (self.cmdid == other.cmdid and
                self.payload == other.payload)
        return False

    def __repr__(self):
        return '%s(payload=%s, cmdid=0x%02x)' % (type(self).__name__, repr(self.payload), self.cmdid)


class RfModuleStatusPacket(Packet):
    def __init__(self, payload=b'', cmdid=0x8a):
        super(RfModuleStatusPacket, self).__init__(payload, cmdid)

        self.status = 0

        if isinstance(payload, RfModuleStatusPacket):
            self.status = payload.status

    def build(self):
        self.payload = struct.pack('B', self.status)

        return super(RfModuleStatusPacket, self).build()

    def parse(self, data=None):
        if data is not None:
            super(RfModuleStatusPacket, self).parse(data)

        self.status = struct.unpack_from('B', self.payload)

register(RfModuleStatusPacket, 0x8a)


class TxRequestPacket(Packet):
    def __init__(self, payload=b'', cmdid=0x01):
        super(TxRequestPacket, self).__init__(payload, cmdid)

        self.frame_id = 0
        self.dest_addr = 0
        self.options = 0
        self.data = b''

        if isinstance(payload, TxRequestPacket):
            self.frame_id = payload.frame_id
            self.dest_addr = payload.dest_addr
            self.options = payload.options
            self.data = payload.data

    def build(self):
        self.payload = struct.pack('>BHB', self.frame_id, self.dest_addr, self.options)
        self.payload += self.data

        return super(TxRequestPacket, self).build()

    def parse(self, data=None):
        if data is not None:
            super(TxRequestPacket, self).parse(data)

        self.frame_id, self.dest_addr, self.options = struct.unpack_from('>BHB', self.payload)
        self.data = self.payload[4:]

register(TxRequestPacket, 0x01)


class TxStatusPacket(Packet):
    def __init__(self, payload=b'', cmdid=0x89):
        super(TxStatusPacket, self).__init__(payload, cmdid)

        self.frame_id = 0
        self.status = 0

        if isinstance(payload, TxStatusPacket):
            self.frame_id = payload.frame_id
            self.status = payload.status

    def build(self):
        self.payload = struct.pack('BB', self.frame_id, self.status)

        return super(TxStatusPacket, self).build()

    def parse(self, data=None):
        if data is not None:
            super(TxStatusPacket, self).parse(data)

        self.frame_id, self.status = struct.unpack_from('BB', self.payload)

register(TxStatusPacket, 0x89)


class RxPacket(Packet):
    def __init__(self, payload=b'', cmdid=0x81):
        super(RxPacket, self).__init__(payload, cmdid)

        self.src_addr = 0
        self.rssi = 0
        self.options = 0
        self.data = b''

        if isinstance(payload, RxPacket):
            self.src_addr = payload.src_addr
            self.rssi = payload.rssi
            self.options = payload.options
            self.data = payload.data

    def build(self):
        self.payload = struct.pack('>HBB', self.src_addr, self.rssi, self.options)
        self.payload += self.data

        return super(RxPacket, self).build()

    def parse(self, data=None):
        if data is not None:
            super(RxPacket, self).parse(data)

        self.src_addr, self.rssi, self.options = struct.unpack_from('>HBB', self.payload)
        self.data = self.payload[4:]

register(RxPacket, 0x81)


class SerialInterface(object):
    def __init__(self, port='/dev/ttyUSB0', baud=115200, timeout=10):
        self.port = port
        self.baud = baud
        self.serial_port = serial.Serial(port, baud, timeout=timeout)
        self.pkt_buffer = bytearray()
        self.raw_log_callback = None

    @property
    def timeout(self):
        return self.serial_port.timeout

    @timeout.setter
    def timeout(self, value):
        self.serial_port.timeout = value

    def send(self, pkt):
        data = pkt.build()
        if self.raw_log_callback:
            self.raw_log_callback(1, data)
        self.serial_port.write(data)

    def receive(self):
        while True:
            if self.serial_port.in_waiting > 0:
                self.pkt_buffer.extend(self.serial_port.read(self.serial_port.in_waiting))

            while len(self.pkt_buffer) > 0 and self.pkt_buffer[0] != 0x7e:
                self.pkt_buffer.pop(0)

            if len(self.pkt_buffer) < 4:
                self.pkt_buffer.extend(self.serial_port.read(1))
                continue

            length = struct.unpack_from('>H', self.pkt_buffer, 1)[0]
            if len(self.pkt_buffer) >= length+4:
                data = self.pkt_buffer[0:length+4]
                del self.pkt_buffer[0:length+4]

                pkt = parse(data)

                if pkt is not None:
                    if self.raw_log_callback:
                        self.raw_log_callback(0, data)
                    return pkt
            else:
                self.pkt_buffer.extend(self.serial_port.read(1))

    def poll(self):
        if self.serial_port.in_waiting > 0:
            self.pkt_buffer.extend(self.serial_port.read(self.serial_port.in_waiting))

        while True:
            while len(self.pkt_buffer) > 0 and self.pkt_buffer[0] != 0x7e:
                self.pkt_buffer.pop(0)

            if len(self.pkt_buffer) < 4:
                return None

            length = struct.unpack_from('>H', self.pkt_buffer, 1)[0]
            if len(self.pkt_buffer) >= length+4:
                data = self.pkt_buffer[0:length+4]
                del self.pkt_buffer[0:length+4]

                pkt = parse(data)

                if pkt is not None:
                    if self.raw_log_callback:
                        self.raw_log_callback(0, data)
                    return pkt
            else:
                return None
