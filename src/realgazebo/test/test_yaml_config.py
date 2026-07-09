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


def test_parse_vehicles_hitl_fields():
    # a HITL vehicle: mode + fc endpoint + motors, build_target omitted
    config = {'px4_target': {0: '/p'}, 'vehicles': {
        0: {'type': 'x500', 'mode': 'hitl', 'spawnpoint': '(0, 0, 0.2, 0)',
            'motors': 4, 'sys_id': 1,
            'fc': {'device': '/dev/ttyACM0', 'baud': 921600}},
    }}
    s = parse_vehicles(config)[0]
    assert s.mode == 'hitl'
    assert s.build_target_path is None       # no SITL build for a real FC
    assert s.fc_endpoint == {'device': '/dev/ttyACM0', 'baud': 921600}
    assert s.motors == 4 and s.sys_id == 1


def test_parse_vehicles_defaults_to_sitl():
    # a normal vehicle has mode 'sitl' and no HITL fields
    config = {'px4_target': {0: '/p'}, 'vehicles': {
        0: {'type': 'x500', 'build_target': 0, 'spawnpoint': '(0,0,0,0)'},
    }}
    s = parse_vehicles(config)[0]
    assert s.mode == 'sitl'
    assert s.fc_endpoint is None and s.motors is None and s.sys_id is None
