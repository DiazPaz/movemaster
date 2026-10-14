import pytest

from movemaster_teleop.pendant_frame import crc8, PendantFrame, SIZE, StreamParser, SYNC


def test_crc8_check_value():
    # CRC-8 (polynomial 0x07, init 0) of "123456789" is 0xF4.
    assert crc8(b'123456789') == 0xF4


def test_round_trip():
    frame = PendantFrame(seq=200, manual=True, gripper=True, x_joint=5, y_joint=3, x=-1000, y=742)
    payload = frame.encode()
    assert len(payload) == SIZE
    assert payload[2] == 0x53  # X joint in the high nibble, Y joint in the low one
    assert PendantFrame.decode(payload) == frame


def test_rejects_corrupt_frames():
    payload = bytearray(PendantFrame(x=10).encode())
    payload[3] ^= 0x01
    with pytest.raises(ValueError):
        PendantFrame.decode(bytes(payload))
    out_of_range = PendantFrame(x=1001).encode()
    with pytest.raises(ValueError):
        PendantFrame.decode(out_of_range)


def test_stream_parser_resynchronizes():
    first = PendantFrame(seq=1, x_joint=1, x=300)
    second = PendantFrame(seq=2, y_joint=2, y=-300)
    # Noise, a false start byte and a frame split across two reads.
    stream = b'\x00\x13' + bytes([SYNC, 1, 2]) + bytes([SYNC]) + first.encode() \
        + bytes([SYNC]) + second.encode()
    parser = StreamParser()
    frames = parser.feed(stream[:9]) + parser.feed(stream[9:])
    assert frames == [first, second]
