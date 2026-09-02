import struct

import pytest
from realgazebo.protocol import (
    parse_packet, pack_pose, pack_destroy, pack_wind, pack_wind_query,
    triage_batch, SpawnCommand, DespawnCommand, WindCommand, WindQuery)


def test_roundtrip_pose():
    pkt = pack_pose(7, 1, (1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0))
    cmd = parse_packet(pkt)
    assert cmd == SpawnCommand(7, 1, (1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0))


def test_golden_vector():
    # byte 0 = instance id (2), byte 1 = type code (0 = x500), byte 2 = MSG_POSE,
    # then 6x f32 0.0 and f32 1.0 (little-endian)
    expected = bytes([2, 0, 1]) + struct.pack('<7f', 0, 0, 0, 0, 0, 0, 1.0)
    assert pack_pose(2, 0, (0, 0, 0), (0, 0, 0, 1.0)) == expected
    # Asserted positionally, never by field name: this pins the wire layout
    # itself, so a rename of the header fields cannot quietly change it.
    assert tuple(parse_packet(expected)) == (
        2, 0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


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


# -- MessageID=6: world wind (UE -> manager) --------------------------------

def test_wind_golden_vector():
    # byte 0-1 = header ids (unused, 0), byte 2 = MSG_WIND, byte 3 = enable,
    # then 3x f32 little-endian: wind velocity in the Gazebo world frame, m/s
    expected = bytes([0, 0, 6, 1]) + struct.pack('<3f', 5.0, -2.0, 0.5)
    assert pack_wind(True, (5.0, -2.0, 0.5)) == expected
    assert len(expected) == 16
    # positional, so a field rename cannot quietly change the wire layout
    assert tuple(parse_packet(expected)) == (True, (5.0, -2.0, 0.5))


def test_wind_off_ignores_vector():
    # OFF carries whatever vector the sender had; it is normalised to zeros
    cmd = parse_packet(pack_wind(False, (5.0, -2.0, 0.5)))
    assert cmd == WindCommand(False, (0.0, 0.0, 0.0))
    assert cmd.enable is False
    # ...even a non-finite one: OFF must always be able to land
    nan_off = bytes([0, 0, 6, 0]) + struct.pack('<3f', float('nan'), 0, 0)
    assert parse_packet(nan_off) == WindCommand(False, (0.0, 0.0, 0.0))


def test_wind_query_golden_vector():
    # op byte 2 = "tell me the current wind"; the vector is unused (zeros)
    expected = bytes([0, 0, 6, 2]) + struct.pack('<3f', 0, 0, 0)
    assert pack_wind_query() == expected
    assert isinstance(parse_packet(expected), WindQuery)
    # the vector is ignored on a query, even garbage
    assert isinstance(
        parse_packet(bytes([0, 0, 6, 2]) + b'\xff' * 12), WindQuery)


@pytest.mark.parametrize('op', [3, 255])
def test_wind_unknown_op_rejected(op):
    with pytest.raises(ValueError):
        parse_packet(bytes([0, 0, 6, op]) + struct.pack('<3f', 1, 0, 0))


def test_wind_is_world_level_not_an_entity_command():
    # Header ids are ignored for wind (world-level command) and must NOT be
    # carried: type_code 0 is a real vehicle (x500), so a WindCommand with
    # entity fields would be mistaken for x500_0 by the entity path.
    cmd = parse_packet(bytes([7, 3, 6, 1]) + struct.pack('<3f', 1.0, 0, 0))
    assert isinstance(cmd, WindCommand)
    assert not hasattr(cmd, 'entity_id')
    assert not hasattr(cmd, 'type_code')


def test_wind_payload_too_short_raises():
    with pytest.raises(ValueError):
        parse_packet(bytes([0, 0, 6, 1]) + b'\x00' * 8)  # need 12, give 8
    with pytest.raises(ValueError):
        parse_packet(bytes([0, 0, 6]))  # no enable byte at all


def test_wind_trailing_bytes_tolerated():
    pkt = pack_wind(True, (1.0, 2.0, 3.0)) + b'\xff' * 4
    assert parse_packet(pkt) == WindCommand(True, (1.0, 2.0, 3.0))


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), float('-inf')])
def test_wind_non_finite_velocity_rejected(bad):
    # UE textboxes are free-form; a NaN/inf wind fed into physics would
    # poison every enable_wind link, so it is refused at the parser
    with pytest.raises(ValueError):
        parse_packet(pack_wind(True, (bad, 0.0, 0.0)))


def test_unknown_message_id_returns_none():
    assert parse_packet(bytes([0, 0, 7]) + b'\x00' * 13) is None


def test_triage_batch_keeps_newest_wind_and_entity_order():
    pose_a = parse_packet(pack_pose(0, 0, (1, 2, 3), (0, 0, 0, 1)))
    wind_1 = parse_packet(pack_wind(True, (1.0, 0.0, 0.0)))
    kill_b = parse_packet(pack_destroy(1, 0))
    wind_2 = parse_packet(pack_wind(False, (0.0, 0.0, 0.0)))
    wind, askers, rest = triage_batch([
        (pose_a, '10.0.0.1'), (wind_1, '10.0.0.1'),
        (kill_b, '10.0.0.1'), (wind_2, '10.0.0.1')])
    assert wind == wind_2  # newest world setting wins
    assert askers == []
    assert rest == [pose_a, kill_b]  # entity commands untouched, in order


def test_triage_batch_collects_askers_once_each_in_order():
    query = parse_packet(pack_wind_query())
    pose_a = parse_packet(pack_pose(0, 0, (1, 2, 3), (0, 0, 0, 1)))
    unknown = parse_packet(bytes([0, 0, 7]) + b'\x00' * 13)  # -> None
    wind, askers, rest = triage_batch([
        (query, '10.0.0.2'), (pose_a, '10.0.0.1'), (unknown, '10.0.0.9'),
        (query, '10.0.0.1'), (query, '10.0.0.2')])
    assert wind is None
    assert askers == ['10.0.0.2', '10.0.0.1']  # deduplicated, arrival order
    assert rest == [pose_a]  # None packets dropped


def test_triage_batch_empty():
    assert triage_batch([]) == (None, [], [])
