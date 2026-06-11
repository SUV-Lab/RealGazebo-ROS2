import pytest
from realgazebo_manager.registry import VehicleRegistry


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
