import pytest

from realgazebo.entity import Entity

CODE_MAP = {0: 'x500', 5: 'x500_lidar_2d', 201: 'rock'}


def test_name_formats_type_and_id():
    assert Entity('x500', 3, 0).name == 'x500_3'
    assert Entity('rock', 9, 201).name == 'rock_9'


def test_is_prop_derived_from_code_threshold():
    # 0..199 = PX4 vehicles, >= 200 = static props (201 = rock)
    assert not Entity('x500', 0, 0).is_prop
    assert not Entity('edge', 0, 199).is_prop
    assert Entity('edge', 0, 200).is_prop
    assert Entity('rock', 0, 201).is_prop


def test_create_resolves_code_from_map():
    assert Entity.create('rock', 9, CODE_MAP) == Entity('rock', 9, 201)
    assert Entity.create('x500', '3', CODE_MAP).id == 3  # coerces str ids


def test_create_unknown_type_raises():
    # fail-closed: an unknown type must never fall through to the PX4 path
    with pytest.raises(ValueError):
        Entity.create('warehouse', 1, CODE_MAP)


def test_frozen_identity():
    e = Entity('x500', 0, 0)
    with pytest.raises(AttributeError):
        e.id = 1
