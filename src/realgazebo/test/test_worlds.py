import pytest

from realgazebo import worlds


def _write_world(tmp_path, file_stem, declared_name):
    d = tmp_path / 'worlds'
    d.mkdir(exist_ok=True)
    p = d / f'{file_stem}.sdf'
    p.write_text(
        f"<sdf version='1.9'><world name='{declared_name}'>"
        f'<gravity>0 0 -9.8</gravity></world></sdf>')
    return p


# -- declared_world_name ---------------------------------------------------

def test_declared_world_name_reads_the_attribute(tmp_path):
    p = _write_world(tmp_path, 'c-track', 'c-track')
    assert worlds.declared_world_name(str(p)) == 'c-track'


def test_declared_world_name_is_none_when_unreadable(tmp_path):
    assert worlds.declared_world_name(str(tmp_path / 'nope.sdf')) is None


def test_declared_world_name_is_none_on_malformed_xml(tmp_path):
    p = tmp_path / 'broken.sdf'
    p.write_text('<sdf><world name="x">')
    assert worlds.declared_world_name(str(p)) is None


def test_declared_world_name_is_none_without_a_world_element(tmp_path):
    p = tmp_path / 'noworld.sdf'
    p.write_text('<sdf version="1.9"><model name="x"/></sdf>')
    assert worlds.declared_world_name(str(p)) is None


# -- available_worlds ------------------------------------------------------

def test_available_worlds_lists_stems_sorted(tmp_path):
    _write_world(tmp_path, 'c-track', 'c-track')
    _write_world(tmp_path, 'apron', 'apron')
    assert worlds.available_worlds(str(tmp_path)) == ['apron', 'c-track']


def test_available_worlds_is_empty_without_a_worlds_dir(tmp_path):
    assert worlds.available_worlds(str(tmp_path)) == []


# -- resolve_world_file ----------------------------------------------------

def test_resolve_returns_the_matching_sdf(tmp_path):
    p = _write_world(tmp_path, 'c-track', 'c-track')
    assert worlds.resolve_world_file(str(tmp_path), 'c-track') == str(p)


def test_resolve_rejects_a_missing_world_and_names_the_alternatives(tmp_path):
    _write_world(tmp_path, 'c-track', 'c-track')
    with pytest.raises(RuntimeError) as exc:
        worlds.resolve_world_file(str(tmp_path), 'urban')
    msg = str(exc.value)
    assert 'urban' in msg
    # The operator most likely wanted a terrain crop, not a world.
    assert 'terrain' in msg
    assert 'c-track' in msg


def test_resolve_rejects_a_name_that_disagrees_with_the_file(tmp_path):
    """The bug this whole split exists to prevent.

    A world whose <world name=> differs from its file name puts the gz server
    in one namespace and the manager in another: models spawn but every
    despawn is addressed to a world that does not exist, and manager_node's
    check=False swallows it. Fail at launch instead.
    """
    _write_world(tmp_path, 'apron', 'c-track')
    with pytest.raises(RuntimeError) as exc:
        worlds.resolve_world_file(str(tmp_path), 'apron')
    assert "name='c-track'" in str(exc.value)


def test_resolve_accepts_an_sdf_whose_name_cannot_be_read(tmp_path):
    """Unparseable means 'cannot check', not 'mismatch' - let gz report it."""
    d = tmp_path / 'worlds'
    d.mkdir()
    (d / 'weird.sdf').write_text('not xml at all')
    assert worlds.resolve_world_file(str(tmp_path), 'weird').endswith('weird.sdf')
