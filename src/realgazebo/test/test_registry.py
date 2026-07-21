import pytest
from realgazebo.entity import Entity
from realgazebo.registry import EntityRegistry


def _x500(entity_id):
    return Entity('x500', entity_id, 0)


def _rock(entity_id):
    return Entity('rock', entity_id, 201)


def test_add_and_is_active():
    r = EntityRegistry()
    assert not r.is_active('x500', 0)
    r.add(_x500(0))
    assert r.is_active('x500', 0)


def test_duplicate_raises():
    r = EntityRegistry()
    r.add(_x500(0))
    with pytest.raises(ValueError):
        r.add(_x500(0))


def test_active_ids_sorted():
    r = EntityRegistry()
    r.add(Entity('boat', 1, 2))
    r.add(_x500(0))
    assert r.active_ids() == [('boat', 1), ('x500', 0)]


def test_records_snapshot_holds_entities():
    r = EntityRegistry()
    r.add(_x500(0))
    r.add(_rock(9))
    assert [rec.entity.name for rec in r.records()] == ['rock_9', 'x500_0']


def test_remove():
    r = EntityRegistry()
    rec = r.add(_x500(0))
    assert r.remove('x500', 0) is rec
    assert not r.is_active('x500', 0)
    assert r.remove('x500', 0) is None


def test_type_of_finds_holder_across_types():
    r = EntityRegistry()
    r.add(_x500(0))
    r.add(_rock(9))
    assert r.type_of(0) == 'x500'
    assert r.type_of(9) == 'rock'
    assert r.type_of(7) is None
    r.remove('rock', 9)
    assert r.type_of(9) is None


def test_record_prop_derived_from_entity_code():
    # no imperative flag to forget: prop-ness comes from the wire code
    r = EntityRegistry()
    assert r.add(_rock(9)).prop is True
    assert r.add(_x500(0)).prop is False
