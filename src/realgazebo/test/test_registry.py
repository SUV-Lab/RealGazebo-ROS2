import pytest
from realgazebo.registry import VehicleRegistry


def test_add_and_is_active():
    r = VehicleRegistry()
    assert not r.is_active('x500', 0)
    r.add('x500', 0)
    assert r.is_active('x500', 0)


def test_duplicate_raises():
    r = VehicleRegistry()
    r.add('x500', 0)
    with pytest.raises(ValueError):
        r.add('x500', 0)


def test_active_ids_sorted():
    r = VehicleRegistry()
    r.add('boat', 1)
    r.add('x500', 0)
    assert r.active_ids() == [('boat', 1), ('x500', 0)]


def test_remove():
    r = VehicleRegistry()
    rec = r.add('x500', 0)
    assert r.remove('x500', 0) is rec
    assert not r.is_active('x500', 0)
    assert r.remove('x500', 0) is None


def test_type_of_finds_holder_across_types():
    r = VehicleRegistry()
    r.add('x500', 0)
    r.add('rock', 9)
    assert r.type_of(0) == 'x500'
    assert r.type_of(9) == 'rock'
    assert r.type_of(7) is None
    r.remove('rock', 9)
    assert r.type_of(9) is None


def test_record_prop_defaults_false():
    r = VehicleRegistry()
    rec = r.add('rock', 9)
    assert rec.prop is False  # callers opt in explicitly
    rec.prop = True
    assert r.get('rock', 9).prop is True
