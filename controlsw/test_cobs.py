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

from . import cobs

def test_encoder():
    assert cobs.encode(b'') == b'\x01'
    assert cobs.encode(b'\x00') == b'\x01\x01'
    assert cobs.encode(b'\x11\x22\x00\x33') == b'\x03\x11\x22\x02\x33'
    assert cobs.encode(b'\x11\x22\x33\x44') == b'\x05\x11\x22\x33\x44'
    assert cobs.encode(b'\x11\x00\x00\x00') == b'\x02\x11\x01\x01\x01'
    assert cobs.encode(bytearray(range(1, 255))) == b'\xff'+bytearray(range(1, 255))
    assert cobs.encode(bytearray(range(255))) == b'\x01\xff'+bytearray(range(1, 255))
    assert cobs.encode(bytearray(range(1, 256))) == b'\xff'+bytearray(range(1, 255))+b'\x02\xff'
    assert cobs.encode(bytearray(range(2, 256))+b'\x00') == b'\xff'+bytearray(range(2, 256))+b'\x01\x01'
    assert cobs.encode(bytearray(range(3, 256))+b'\x00\x01') == b'\xfe'+bytearray(range(3, 256))+b'\x02\x01'

def test_decoder():
    assert cobs.decode(b'\x01') == b''
    assert cobs.decode(b'\x01\x01') == b'\x00'
    assert cobs.decode(b'\x03\x11\x22\x02\x33') == b'\x11\x22\x00\x33'
    assert cobs.decode(b'\x05\x11\x22\x33\x44') == b'\x11\x22\x33\x44'
    assert cobs.decode(b'\x02\x11\x01\x01\x01') == b'\x11\x00\x00\x00'
    assert cobs.decode(b'\xff'+bytearray(range(1, 255))) == bytearray(range(1, 255))
    assert cobs.decode(b'\x01\xff'+bytearray(range(1, 255))) == bytearray(range(255))
    assert cobs.decode(b'\xff'+bytearray(range(1, 255))+b'\x02\xff') == bytearray(range(1, 256))
    assert cobs.decode(b'\xff'+bytearray(range(2, 256))+b'\x01\x01') == bytearray(range(2, 256))+b'\x00'
    assert cobs.decode(b'\xfe'+bytearray(range(3, 256))+b'\x02\x01') == bytearray(range(3, 256))+b'\x00\x01'

