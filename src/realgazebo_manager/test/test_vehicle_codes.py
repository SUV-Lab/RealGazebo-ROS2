import pytest
from realgazebo_manager.vehicle_codes import type_for_code


def test_known_codes():
    assert type_for_code(0) == 'x500'
    assert type_for_code(1) == 'rover_ackermann'
    assert type_for_code(2) == 'boat'
    assert type_for_code(3) == 'lc_62'
    assert type_for_code(4) == 'ugv_kimm'
    assert type_for_code(201) == 'rock'


def test_unknown_code_raises():
    with pytest.raises(ValueError):
        type_for_code(255)
