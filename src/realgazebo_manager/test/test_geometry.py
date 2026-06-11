import math
from realgazebo_manager.geometry import quat_to_euler


def test_identity():
    roll, pitch, yaw = quat_to_euler(0, 0, 0, 1)
    assert abs(roll) < 1e-9 and abs(pitch) < 1e-9 and abs(yaw) < 1e-9


def test_yaw_90deg():
    s = math.sqrt(0.5)  # quaternion for +90 deg about z
    roll, pitch, yaw = quat_to_euler(0, 0, s, s)
    assert abs(roll) < 1e-9
    assert abs(pitch) < 1e-9
    assert abs(yaw - math.pi / 2) < 1e-9


def test_roll_90deg():
    s = math.sqrt(0.5)  # quaternion for +90 deg about x
    roll, pitch, yaw = quat_to_euler(s, 0, 0, s)
    assert abs(roll - math.pi / 2) < 1e-9
    assert abs(pitch) < 1e-9
    assert abs(yaw) < 1e-9
