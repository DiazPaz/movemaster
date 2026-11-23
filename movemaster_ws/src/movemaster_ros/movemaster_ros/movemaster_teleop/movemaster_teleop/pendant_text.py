"""The pendant's text protocol (hmi/esp32/main.cpp): readable in a serial monitor, and read by
jog_node as the same PendantFrame as the binary protocol. Without ROS so it can be tested alone.

ESP32 -> Raspberry Pi, a block every 20 ms:

    Freedrive: ON
    Joint Y: J1 | Angulo: 12.3 grados | Joystick Y: -450
    Joint X: J4 | Angulo: -5.0 grados | Joystick X: 0
    Gripper: OFF
    -------------------------

Freedrive is the MANUAL switch. Joystick goes from -1000 to 1000 around the calibrated center,
and is 0 in AUTO. Gripper is the flag the joystick button toggles: ON closes the gripper.
Angulo is only shown: the ESP32 prints what the Raspberry Pi last sent it ('--' before that).

Raspberry Pi -> ESP32, the measured angles of J1..Jn in degrees, Home at 0:

    ANGLES,12.3,-20.5,33.3,0.0,45.0
"""

import re

from movemaster_teleop.pendant_frame import FULL_SCALE, PendantFrame

_FREEDRIVE = re.compile(r'Freedrive: (ON|OFF)')
_JOINT = re.compile(r'Joint ([XY]): J(\d{1,2}) \| Angulo: \S+ grados \| Joystick \1: (-?\d{1,4})')
_GRIPPER = re.compile(r'Gripper: (ON|OFF)')
_END = re.compile(r'-{5,}')
_MAX_LINE = 160  # longer means garbage: a block line is about 60 bytes
SEPARATOR = '-' * 25


class TextStreamParser:
    """Recovers complete blocks from the byte stream.

    Any line out of place drops the block it is in, so only a block read whole, in order, makes
    a frame: the text has no CRC, the strict format is its check.
    """

    def __init__(self):
        self._line = bytearray()
        self._block = None

    def feed(self, data):
        """Every frame completed by these bytes, oldest first."""
        self._line += data
        frames = []
        while True:
            end = self._line.find(b'\n')
            if end < 0:
                if len(self._line) > _MAX_LINE:
                    self._line.clear()
                    self._block = None
                return frames
            line = self._line[:end].decode('ascii', errors='replace').strip()
            del self._line[:end + 1]
            frame = self._take(line)
            if frame is not None:
                frames.append(frame)

    def _take(self, line):
        if not line:
            return None
        match = _FREEDRIVE.fullmatch(line)
        if match:
            self._block = {'manual': match.group(1) == 'ON'}
            return None
        if self._block is None:
            return None  # outside a block, e.g. the ESP32's boot messages
        match = _JOINT.fullmatch(line)
        if match and int(match.group(3)) in range(-FULL_SCALE, FULL_SCALE + 1) \
                and int(match.group(2)) <= 15:
            self._block[match.group(1)] = (int(match.group(2)), int(match.group(3)))
            return None
        match = _GRIPPER.fullmatch(line)
        if match:
            self._block['gripper'] = match.group(1) == 'ON'
            return None
        block, self._block = self._block, None
        if not _END.fullmatch(line) or not {'manual', 'X', 'Y', 'gripper'} <= block.keys():
            return None
        (x_joint, x), (y_joint, y) = block['X'], block['Y']
        return PendantFrame(manual=block['manual'], gripper=block['gripper'],
                            x_joint=x_joint, y_joint=y_joint, x=x, y=y)


def encode(frame, angles_deg=None):
    """The block the ESP32 prints for a frame; angles_deg (J1..Jn) fills Angulo."""
    def angle(joint):
        if angles_deg is None or not 1 <= joint <= len(angles_deg):
            return '--'
        return f'{angles_deg[joint - 1]:.1f}'
    lines = [f'Freedrive: {"ON" if frame.manual else "OFF"}',
             f'Joint Y: J{frame.y_joint} | Angulo: {angle(frame.y_joint)} grados | '
             f'Joystick Y: {frame.y}',
             f'Joint X: J{frame.x_joint} | Angulo: {angle(frame.x_joint)} grados | '
             f'Joystick X: {frame.x}',
             f'Gripper: {"ON" if frame.gripper else "OFF"}',
             SEPARATOR]
    return ('\r\n'.join(lines) + '\r\n').encode('ascii')


def format_angles(angles_deg):
    """The line that tells the ESP32 the measured angles."""
    return ('ANGLES,' + ','.join(f'{angle:.1f}' for angle in angles_deg) + '\n').encode('ascii')
