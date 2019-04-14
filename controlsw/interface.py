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
import socket

import cobs


class Interface(object):
    def __init__(self):
        self._root = None

    def send(self, packet):
        raise NotImplementedError()

    def receive(self):
        raise NotImplementedError()


class SerialInterface(Interface):
    def __init__(self, port='/dev/ttyUSB0', baud=115200, timeout=10):
        self.port = port
        self.baud = baud
        self.serial_port = serial.Serial(port, baud, timeout=timeout)

    @property
    def timeout(self):
        return self.serial_port.timeout

    @timeout.setter
    def timeout(self, value):
        self.serial_port.timeout = value

    def send(self, pkt):
        self.serial_port.write(cobs.encode(pkt.build())+b'\x00')

    def receive(self):
        return packet.parse(cobs.decode(self.serial_port.read_until(b'\x00')[:-1]))


class UDPInterface(Interface):
    def __init__(self, host, port=14000, timeout=10):
        if ':' in host:
            host, port = host.rsplit(':', 2)
            port = int(port)

        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.settimeout(timeout)

    @property
    def timeout(self):
        return self.socket.gettimeout()

    @timeout.setter
    def timeout(self, value):
        self.socket.settimeout(value)

    def send(self, pkt):
        self.socket.sendto(pkt.build(), (self.host, self.port))

    def receive(self):
        return packet.parse(self.socket.recvfrom(1500)[0])

