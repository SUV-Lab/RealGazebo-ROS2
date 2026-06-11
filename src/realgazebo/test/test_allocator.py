import pytest
from realgazebo.allocator import allocate, MAX_VEHICLE_ID


def test_allocate_golden_vehicle_0():
    """Golden values copied from the retired generate_compose.py / docker-compose.override.yml flow."""
    r = allocate(0)
    assert r.gz_ip == '172.20.0.10'
    assert r.vehicle_ip == '172.30.0.10'
    assert r.mavlink_port == 18570
    assert r.dds_profile_path == '/tmp/dds_profiles/px4_participant_0.xml'
    assert r.ros_namespace == 'vehicle1'


def test_allocate_golden_vehicle_7():
    r = allocate(7)
    assert r.gz_ip == '172.20.0.17'
    assert r.vehicle_ip == '172.30.0.17'
    assert r.mavlink_port == 18577
    assert r.dds_profile_path == '/tmp/dds_profiles/px4_participant_7.xml'
    assert r.ros_namespace == 'vehicle8'


def test_allocate_range_limits():
    allocate(MAX_VEHICLE_ID)  # last valid id -> octet 254
    with pytest.raises(ValueError):
        allocate(-1)
    with pytest.raises(ValueError):
        allocate(MAX_VEHICLE_ID + 1)
