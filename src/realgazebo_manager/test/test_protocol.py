import struct

import pytest
from realgazebo_manager.protocol import (
    parse_packet, pack_pose, pack_destroy, SpawnCommand, DespawnCommand)


def test_roundtrip_pose():
    pkt = pack_pose(7, 1, (1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0))
    cmd = parse_packet(pkt)
    assert cmd == SpawnCommand(7, 1, (1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0))


def test_golden_vector():
    # vehicle_num=2, vehicle_code=0 (x500), pos=(0,0,0), quat=identity
    # header bytes 02 00 01, then 6x f32 0.0 and f32 1.0 (little-endian)
    expected = bytes([2, 0, 1]) + struct.pack('<7f', 0, 0, 0, 0, 0, 0, 1.0)
    assert pack_pose(2, 0, (0, 0, 0), (0, 0, 0, 1.0)) == expected
    cmd = parse_packet(expected)
    assert cmd.vehicle_num == 2
    assert cmd.vehicle_code == 0
    assert cmd.position == (0.0, 0.0, 0.0)
    assert cmd.quaternion == (0.0, 0.0, 0.0, 1.0)


def test_non_pose_message_returns_none():
    # MessageID=2 (motor RPM) is not an inbound spawn command
    assert parse_packet(bytes([0, 0, 2]) + b'\x00' * 4) is None


def test_short_packet_raises():
    with pytest.raises(ValueError):
        parse_packet(b'\x00')


def test_pose_payload_too_short_raises():
    with pytest.raises(ValueError):
        parse_packet(bytes([0, 0, 1]) + b'\x00' * 8)  # need 28 bytes, give 8


def test_parse_destroy():
    cmd = parse_packet(pack_destroy(2, 0))
    assert cmd == DespawnCommand(2, 0)
