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
import datetime
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


class CommandPacket(Packet):
    def __init__(self, payload=b'', dest=0xff, source=0x00, flags=0x00, ptype=0x08):
        super(CommandPacket, self).__init__(payload, dest, source, flags, ptype)

        self.cmd = 0
        self.data = b''

    def build(self):
        self.payload = struct.pack('<L', self.cmd) + self.data

        return super(CommandPacket, self).build()

    def parse(self, data=None):
        if data is not None:
            super(CommandPacket, self).parse(data)

        self.cmd = struct.unpack_from('<L', self.payload, 0)
        self.data = self.payload[4:]

register(CommandPacket, 0x08)


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


class GpsPositionPacket(Packet):
    def __init__(self, payload=b'', dest=0xff, source=0x00, flags=0x00, ptype=0x31):
        super(GpsPositionPacket, self).__init__(payload, dest, source, flags, ptype)

        self.latitude = 0
        self.longitude = 0
        self.altitude = 0
        self.speed = 0
        self.heading = 0
        self.satellites = 0
        self.fix_type = 0
        self.date = None
        self.time = None
        self.hdop = 0

    def build(self):
        self.payload = struct.pack('<l', int(self.latitude * 1e7))
        self.payload += struct.pack('<l', int(self.longitude * 1e7))
        self.payload += struct.pack('<l', int(self.altitude * 1e2))
        self.payload += struct.pack('<l', int(self.speed * 1e2))
        self.payload += struct.pack('<l', int(self.heading * 1e2))
        self.payload += struct.pack('B', self.satellites)
        self.payload += struct.pack('B', self.fix_type)
        self.payload += struct.pack('<l', self.date.day * 10000 + self.date.month * 100 + self.date.year % 100)
        self.payload += struct.pack('<l', self.time.hour * 1000000 + self.time.minute * 10000 + int(self.time.second * 1e2 + self.time.microsecond / 10000))
        self.payload += struct.pack('<h', int(self.hdop * 1e2))

        return super(GpsPositionPacket, self).build()

    def parse(self, data=None):
        if data is not None:
            super(GpsPositionPacket, self).parse(data)

        self.latitude = struct.unpack_from('<l', self.payload, 0)[0] / 1e7
        self.longitude = struct.unpack_from('<l', self.payload, 4)[0] / 1e7
        self.altitude = struct.unpack_from('<l', self.payload, 8)[0] / 1e2
        self.speed = struct.unpack_from('<l', self.payload, 12)[0] / 1e2
        self.heading = struct.unpack_from('<l', self.payload, 16)[0] / 1e2
        self.satellites = struct.unpack_from('B', self.payload, 20)[0]
        self.fix_type = struct.unpack_from('B', self.payload, 21)[0]
        d = struct.unpack_from('<l', self.payload, 22)[0]
        day = int(d / 10000) % 100
        month = int(d / 100) % 100
        year = d % 100 + 2000
        try:
            self.date = datetime.date(year, month, day)
        except:
            self.date = None
        d = struct.unpack_from('<l', self.payload, 26)[0]
        hour = int(d / 1000000) % 100
        minute = int(d / 10000) % 100
        second = d % 10000
        try:
            self.time = datetime.time(hour, minute, int(second/100), (second%100) * 10000)
        except:
            self.time = None
        self.hdop = struct.unpack_from('<h', self.payload, 30)[0] / 1e2

register(GpsPositionPacket, 0x31)


class FlightStatusPacket(Packet):
    def __init__(self, payload=b'', dest=0xff, source=0x00, flags=0x00, ptype=0xf0):
        super(FlightStatusPacket, self).__init__(payload, dest, source, flags, ptype)

        self.time = 0
        self.flight_phase = 0
        self.status_flags = 0
        self.baro_pressure = 0
        self.baro_temperature = 0
        self.imu_accel = 0
        self.baro_raw_altitude = 0
        self.baro_altitude = 0
        self.baro_pad_altitude = 0
        self.baro_speed = 0
        self.imu_speed = 0
        self.imu_altitude = 0

    def build(self):
        self.payload = struct.pack('<l', self.time)
        self.payload += struct.pack('B', self.flight_phase)
        self.payload += struct.pack('B', self.status_flags)
        self.payload += struct.pack('<l', self.baro_pressure)
        self.payload += struct.pack('<l', int(self.baro_temperature*1e2))
        self.payload += struct.pack('<l', int(self.imu_accel)*1e2)
        self.payload += struct.pack('<l', int(self.baro_raw_altitude*1e2))
        self.payload += struct.pack('<l', int(self.baro_altitude*1e2))
        self.payload += struct.pack('<l', int(self.baro_pad_altitude*1e2))
        self.payload += struct.pack('<l', int(self.baro_speed*1e2))
        self.payload += struct.pack('<l', int(self.imu_speed*1e2))
        self.payload += struct.pack('<l', int(self.imu_altitude*1e2))

        return super(FlightStatusPacket, self).build()

    def parse(self, data=None):
        if data is not None:
            super(FlightStatusPacket, self).parse(data)

        self.time = struct.unpack_from('<l', self.payload, 0)[0]
        self.flight_phase = struct.unpack_from('B', self.payload, 4)[0]
        self.status_flags = struct.unpack_from('B', self.payload, 5)[0]
        self.baro_pressure = struct.unpack_from('<l', self.payload, 6)[0]
        self.baro_temperature = struct.unpack_from('<l', self.payload, 10)[0] / 1e2
        self.imu_accel = struct.unpack_from('<l', self.payload, 14)[0] / 1e2
        self.baro_raw_altitude = struct.unpack_from('<l', self.payload, 18)[0] / 1e2
        self.baro_altitude = struct.unpack_from('<l', self.payload, 22)[0] / 1e2
        self.baro_pad_altitude = struct.unpack_from('<l', self.payload, 26)[0] / 1e2
        self.baro_speed = struct.unpack_from('<l', self.payload, 30)[0] / 1e2
        self.imu_speed = struct.unpack_from('<l', self.payload, 34)[0] / 1e2
        self.imu_altitude = struct.unpack_from('<l', self.payload, 38)[0] / 1e2

register(FlightStatusPacket, 0xf0)

