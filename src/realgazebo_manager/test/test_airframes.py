import pytest
from realgazebo_manager.airframes import scan_airframes, get_autostart_id


def _make_airframes(tmp_path, names):
    d = tmp_path / "ROMFS/px4fmu_common/init.d-posix/airframes"
    d.mkdir(parents=True)
    for n in names:
        (d / n).write_text("")
    return str(tmp_path)


def test_scan_airframes(tmp_path):
    px4 = _make_airframes(tmp_path, ["4001_gz_x500", "4009_gz_boat", "README.md"])
    assert scan_airframes(px4) == {"x500": "4001", "boat": "4009"}


def test_scan_airframes_missing_dir(tmp_path):
    assert scan_airframes(str(tmp_path / "nope")) == {}


def test_get_autostart_id_ok(tmp_path):
    px4 = _make_airframes(tmp_path, ["4001_gz_x500"])
    assert get_autostart_id("x500", px4) == "4001"


def test_get_autostart_id_missing(tmp_path):
    px4 = _make_airframes(tmp_path, ["4001_gz_x500"])
    with pytest.raises(ValueError):
        get_autostart_id("rover_ackermann", px4)
