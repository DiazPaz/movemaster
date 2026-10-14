"""Teach pendant frame: 8 bytes, the payload of one classic CAN frame.

    byte 0     sequence counter
    byte 1     flags: bit 0 MANUAL switch, bit 1 gripper
    byte 2     selection: high nibble X joint (1..15), low nibble Y joint; 0 = none
    bytes 3-4  joystick X, int16 little-endian, -1000..1000
    bytes 5-6  joystick Y, int16 little-endian, -1000..1000
    byte 7     CRC-8 (polynomial 0x07, init 0) of bytes 0-6

Over UART each frame goes after a start byte, SYNC. Over CAN the 8 bytes are the payload.
"""

from dataclasses import dataclass
import struct

SYNC = 0xA5
SIZE = 8
FULL_SCALE = 1000
_BODY = struct.Struct('<BBBhh')


def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


@dataclass(frozen=True)
class PendantFrame:
    seq: int = 0
    manual: bool = False
    gripper: bool = False
    x_joint: int = 0
    y_joint: int = 0
    x: int = 0
    y: int = 0

    def encode(self) -> bytes:
        flags = int(self.manual) | int(self.gripper) << 1
        selection = (self.x_joint & 0x0F) << 4 | (self.y_joint & 0x0F)
        body = _BODY.pack(self.seq & 0xFF, flags, selection, self.x, self.y)
        return body + bytes([crc8(body)])

    @classmethod
    def decode(cls, payload: bytes) -> 'PendantFrame':
        if len(payload) != SIZE or crc8(payload[:-1]) != payload[-1]:
            raise ValueError('Invalid pendant frame')
        seq, flags, selection, x, y = _BODY.unpack(payload[:-1])
        if abs(x) > FULL_SCALE or abs(y) > FULL_SCALE:
            raise ValueError('Joystick value out of range')
        return cls(seq, bool(flags & 1), bool(flags & 2), selection >> 4, selection & 0x0F, x, y)


class StreamParser:
    """Recovers frames from a UART byte stream, resynchronizing after any corrupt byte."""

    def __init__(self):
        self._buffer = bytearray()

    def feed(self, data: bytes) -> list:
        self._buffer += data
        frames = []
        while True:
            start = self._buffer.find(SYNC)
            if start < 0:
                self._buffer.clear()
                return frames
            del self._buffer[:start]
            if len(self._buffer) < 1 + SIZE:
                return frames
            try:
                frames.append(PendantFrame.decode(bytes(self._buffer[1:1 + SIZE])))
                del self._buffer[:1 + SIZE]
            except ValueError:
                del self._buffer[0]  # A false SYNC: look for the next one.
