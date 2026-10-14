"""Where pendant frames come from. Only the transport changes between links.

SerialLink: USB/UART, start byte + 8-byte frame. A future CanLink returns the same frames
from a SocketCAN socket filtered to the pendant's CAN ID.
"""

import time

import serial

from movemaster_teleop.pendant_frame import StreamParser


class SerialLink:
    """Nonblocking reader; reopens the port after the cable is unplugged."""

    def __init__(self, port, baudrate=115200, retry_s=1.0):
        self.port = port
        self.baudrate = baudrate
        self.retry_s = retry_s
        self._serial = None
        self._next_open = 0.0
        self._parser = StreamParser()

    def poll(self):
        """Every complete frame received since the previous call, oldest first."""
        if self._serial is None and not self._open():
            return []
        try:
            data = self._serial.read(self._serial.in_waiting or 1)
        except (serial.SerialException, OSError):
            self.close()
            return []
        return self._parser.feed(data)

    def _open(self):
        if time.monotonic() < self._next_open:
            return False
        self._next_open = time.monotonic() + self.retry_s
        link = serial.Serial(baudrate=self.baudrate, timeout=0)
        link.port = self.port
        # On ESP32 boards DTR/RTS drive reset and boot mode: keep them released so that
        # opening the port does not reset the pendant.
        link.dtr = False
        link.rts = False
        try:
            link.open()
        except (serial.SerialException, OSError):
            return False
        self._serial = link
        self._parser = StreamParser()
        return True

    @property
    def connected(self):
        return self._serial is not None

    def close(self):
        if self._serial is not None:
            self._serial.close()
            self._serial = None
