from realgazebo.yaml_config import parse_vehicles, VehicleSpec


def test_parse_vehicles_basic():
    config = {
        'px4_target': {0: '/opt/px4'},
        'vehicles': {
            0: {'type': 'x500', 'build_target': 0, 'spawnpoint': '(1.0, 2.0, 3.0, 0.5)'},
            1: {'type': 'boat', 'build_target': 0, 'spawnpoint': '(0, 0, 0, 0)'},
        },
    }
    specs = parse_vehicles(config)
    assert specs[0] == VehicleSpec(0, 'x500', '/opt/px4', (1.0, 2.0, 3.0, 0.5))
    assert specs[1].vehicle_type == 'boat'
    assert specs[1].spawnpoint == (0.0, 0.0, 0.0, 0.0)


def test_parse_vehicles_sorted_by_key():
    config = {'px4_target': {0: '/p'}, 'vehicles': {
        2: {'type': 'a', 'build_target': 0, 'spawnpoint': '(0,0,0,0)'},
        0: {'type': 'b', 'build_target': 0, 'spawnpoint': '(0,0,0,0)'},
    }}
    specs = parse_vehicles(config)
    assert [s.vehicle_id for s in specs] == [0, 2]


def test_parse_vehicles_prop_without_build_target_gets_none():
    # props (rock) need no PX4; vehicles keep their resolved target path
    config = {'px4_target': {0: '/p'}, 'vehicles': {
        0: {'type': 'rock', 'spawnpoint': '(0,0,0,0)'},
        1: {'type': 'x500', 'build_target': 0, 'spawnpoint': '(0,0,0,0)'},
    }}
    specs = parse_vehicles(config)
    assert [s.vehicle_id for s in specs] == [0, 1]
    assert specs[0].build_target_path is None
    assert specs[1].build_target_path == '/p'
