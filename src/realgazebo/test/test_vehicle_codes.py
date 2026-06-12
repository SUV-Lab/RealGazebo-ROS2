import os

import pytest
from realgazebo.vehicle_codes import (
    scan_vehicle_codes, type_for_code, code_for_type, is_prop_code,
    LEGACY_CODE_TO_TYPE)

REPO_MODELS_DIR = os.path.join(os.path.dirname(__file__), '..', 'models')


def _write(tmp_path, rel, code=None):
    path = tmp_path / rel
    path.parent.mkdir(parents=True, exist_ok=True)
    body = '<sdf><model>'
    if code is not None:
        body += f'<plugin filename="libRealGazebo.so" name="custom::RealGazebo">' \
                f'<vehicle_code>{code}</vehicle_code></plugin>'
    body += '</model></sdf>'
    path.write_text(body)


def test_scan_basic_and_subdirectory(tmp_path):
    _write(tmp_path, 'x500.sdf.jinja', 0)
    _write(tmp_path, 'boat.sdf.jinja', 2)
    _write(tmp_path, 'rock/rock.sdf.jinja', 201)
    _write(tmp_path, 'world/model.sdf.jinja')  # no code -> skipped
    assert scan_vehicle_codes(str(tmp_path)) == {
        0: 'x500', 2: 'boat', 201: 'rock'}


def test_scan_duplicate_code_shortest_name_wins(tmp_path):
    _write(tmp_path, 'x500_lidar_2d.sdf.jinja', 0)
    _write(tmp_path, 'x500.sdf.jinja', 0)
    assert scan_vehicle_codes(str(tmp_path)) == {0: 'x500'}


def test_scan_empty_falls_back_to_legacy(tmp_path):
    assert scan_vehicle_codes(str(tmp_path)) == LEGACY_CODE_TO_TYPE


def test_scan_golden_against_repo_templates():
    """The real templates must reproduce the wire protocol's code map."""
    assert scan_vehicle_codes(REPO_MODELS_DIR) == {
        0: 'x500',
        1: 'rover_ackermann',
        2: 'boat',
        3: 'lc_62',
        5: 'x500_lidar_2d',   # own code so it is UDP-spawnable (UE renders as x500)
        201: 'rock',
    }


def test_type_for_code_with_mapping():
    assert type_for_code(2, {2: 'boat'}) == 'boat'
    with pytest.raises(ValueError):
        type_for_code(99, {2: 'boat'})


def test_type_for_code_legacy_default():
    assert type_for_code(0) == 'x500'
    assert type_for_code(4) == 'ugv_kimm'
    with pytest.raises(ValueError):
        type_for_code(255)


def test_code_for_type_reverse_lookup():
    mapping = {0: 'x500', 201: 'rock'}
    assert code_for_type('x500', mapping) == 0
    assert code_for_type('rock', mapping) == 201
    with pytest.raises(ValueError):
        code_for_type('boat', mapping)


def test_is_prop_code_convention():
    # 0..199 = PX4 vehicles, >= 200 = static props (201 = rock)
    assert not is_prop_code(0)
    assert not is_prop_code(5)
    assert not is_prop_code(199)
    assert is_prop_code(200)
    assert is_prop_code(201)
    assert is_prop_code(255)
