import os
import pty
import time
import tty

import pytest

from movemaster_teleop import pendant_text
from movemaster_teleop.pendant_frame import PendantFrame
from movemaster_teleop.pendant_link import SerialLink
from movemaster_teleop.pendant_text import TextStreamParser

FRAME = PendantFrame(manual=True, gripper=True, x_joint=5, y_joint=2, x=-1000, y=345)
# What hmi/esp32/main.cpp prints, byte for byte.
BLOCK = (b'Freedrive: ON\r\n'
         b'Joint Y: J2 | Angulo: -20.5 grados | Joystick Y: 345\r\n'
         b'Joint X: J5 | Angulo: -- grados | Joystick X: -1000\r\n'
         b'Gripper: ON\r\n'
         b'-------------------------\r\n')


def test_a_block_is_a_frame():
    assert TextStreamParser().feed(BLOCK) == [FRAME]


def test_encode_prints_what_the_esp32_prints():
    assert pendant_text.encode(FRAME, [0.0, -20.5]) == BLOCK
    assert TextStreamParser().feed(pendant_text.encode(FRAME)) == [FRAME]


def test_blocks_split_anywhere_still_parse():
    parser = TextStreamParser()
    stream = b'rst:0x1 (POWERON_RESET),boot:0x13\r\nets Jun  8 2016\r\n' + BLOCK * 3
    frames = []
    for start in range(0, len(stream), 7):
        frames += parser.feed(stream[start:start + 7])
    assert frames == [FRAME] * 3


def test_a_broken_block_is_dropped():
    parser = TextStreamParser()
    lines = BLOCK.split(b'\r\n')
    missing_gripper = b'\r\n'.join(lines[:3] + lines[4:])
    garbled = BLOCK.replace(b'Joystick Y: 345', b'Joystick Y: 3x5')
    out_of_range = BLOCK.replace(b'Joystick Y: 345', b'Joystick Y: 1001')
    cut = BLOCK[:40] + b'\r\n'  # a line cut in half
    assert parser.feed(missing_gripper + garbled + out_of_range + cut) == []
    assert parser.feed(BLOCK) == [FRAME]  # and the next whole block is read


def test_angles_line():
    assert pendant_text.format_angles([12.34, -20.5, 0.0]) == b'ANGLES,12.3,-20.5,0.0\n'


@pytest.mark.parametrize('protocol, sent', [
    ('text', BLOCK),
    ('binary', bytes([0xA5]) + FRAME.encode()),
])
def test_the_serial_link_reads_either_protocol_and_writes(protocol, sent):
    master, slave = pty.openpty()
    tty.setraw(slave)
    link = SerialLink(os.ttyname(slave), protocol=protocol)
    try:
        assert link.poll() == [] and link.connected
        os.write(master, sent)
        time.sleep(0.05)
        frames = link.poll()
        assert [(f.manual, f.gripper, f.x_joint, f.y_joint, f.x, f.y) for f in frames] == [
            (True, True, 5, 2, -1000, 345)]
        link.send(b'ANGLES,1.0\n')
        time.sleep(0.05)
        assert os.read(master, 64) == b'ANGLES,1.0\n'
    finally:
        link.close()
        os.close(master)
        os.close(slave)


def test_an_unknown_protocol_is_refused():
    with pytest.raises(ValueError, match='binary, text'):
        SerialLink('/dev/null', protocol='json')
