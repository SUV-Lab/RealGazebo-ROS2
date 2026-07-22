import pytest

from realgazebo.entity import Entity
from realgazebo.yaml_config import VehicleSpec
from realgazebo import spawn_core


def test_build_create_argv():
    argv = spawn_core.build_create_argv(
        Entity('x500', 3, 0), '/tmp/models/x500.sdf', 'c-track',
        (1.0, 2.0, 3.0), (0.0, 0.0, 0.5))
    assert argv == [
        'ros2', 'run', 'ros_gz_sim', 'create',
        '-world', 'c-track', '-file', '/tmp/models/x500.sdf',
        '-name', 'x500_3',
        '-x', '1.0', '-y', '2.0', '-z', '3.0',
        '-R', '0.0', '-P', '0.0', '-Y', '0.5']


def test_build_px4_command(tmp_path):
    d = tmp_path / "ROMFS/px4fmu_common/init.d-posix/airframes"
    d.mkdir(parents=True)
    (d / "4001_gz_x500").write_text("")
    spec = VehicleSpec(2, 'x500', str(tmp_path), (0.0, 0.0, 0.0, 0.0))
    argv, env, cwd = spawn_core.build_px4_command(spec, 'urban')
    assert argv == [str(tmp_path / 'build/px4_sitl_default/bin/px4'), '-i', '2']
    assert cwd == str(tmp_path / 'build/px4_sitl_default')
    assert env['PX4_GZ_STANDALONE'] == '1'
    assert env['PX4_SYS_AUTOSTART'] == '4001'
    assert env['PX4_GZ_MODEL_NAME'] == 'x500_2'
    assert env['PX4_UXRCE_DDS_NS'] == 'vehicle3'
    assert env['PX4_GZ_WORLD'] == 'urban'


def test_build_param_argv():
    spec = VehicleSpec(2, 'x500', '/opt/px4', (0.0, 0.0, 0.0, 0.0))
    argv = spawn_core.build_param_argv(spec, 'NAV_DLL_ACT', 0)
    assert argv == [
        '/opt/px4/build/px4_sitl_default/bin/px4-param',
        '--instance', '2', 'set', 'NAV_DLL_ACT', '0']


def test_render_sdf(tmp_path):
    models = tmp_path / "models"
    models.mkdir()
    (models / "x500.sdf.jinja").write_text("ip={{ unreal_ip }} port={{ unreal_port }}")
    out = spawn_core.render_sdf(
        'x500', '10.0.0.5', 5005,
        models_dir=str(models), output_dir=str(tmp_path / "out"))
    assert "ip=10.0.0.5 port=5005" in open(out).read()


def test_render_sdf_nested_obstacle_template(tmp_path):
    # obstacle-style templates live in <name>/<name>.sdf.jinja
    models = tmp_path / "models"
    (models / "rock").mkdir(parents=True)
    (models / "rock" / "rock.sdf.jinja").write_text("rock ip={{ unreal_ip }}")
    out = spawn_core.render_sdf(
        'rock', '10.0.0.5', 5005,
        models_dir=str(models), output_dir=str(tmp_path / "out"))
    assert out.endswith('/rock.sdf')
    assert "rock ip=10.0.0.5" in open(out).read()


def test_build_remove_argv():
    argv = spawn_core.build_remove_argv('c-track', Entity('x500', 2, 0))
    assert argv == [
        'gz', 'service', '-s', '/world/c-track/remove',
        '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
        '--timeout', '3000', '--req', 'name: "x500_2" type: MODEL']


def test_build_set_pose_argv():
    argv = spawn_core.build_set_pose_argv(
        'c-track', Entity('rock', 9, 201), (1.0, -2.0, 0.5),
        (0.0, 0.0, 0.1, 0.9))
    assert argv[:3] == ['gz', 'service', '-s']
    assert argv[3] == '/world/c-track/set_pose'
    assert '--reqtype' in argv and 'gz.msgs.Pose' in argv
    req = argv[-1]
    assert 'name: "rock_9"' in req
    assert 'position {x: 1.0, y: -2.0, z: 0.5}' in req
    assert 'orientation {x: 0.0, y: 0.0, z: 0.1, w: 0.9}' in req


def test_build_hitl_command_serial():
    # HITL: --world and --motors MUST be passed (bridge defaults are wrong)
    spec = VehicleSpec(0, 'x500', None, (0.0, 0.0, 0.2, 0.0),
                       mode='hitl', motors=4,
                       fc_endpoint={'device': '/dev/ttyACM0', 'baud': 921600})
    argv, env, cwd = spawn_core.build_hitl_command(
        spec, 'c-track', '/opt/px4', '172.17.0.1', 14550)
    assert argv == [
        '/opt/px4/build/px4_sitl_default/bin/gz-hitl-bridge',
        '--model', 'x500_0', '--world', 'c-track', '--sysid', '1',
        '--qgc', '172.17.0.1:14550',
        '--motors', '4', '--device', '/dev/ttyACM0', '--baud', '921600']
    assert env is None and cwd is None


def test_build_hitl_command_sysid_defaults_to_id_plus_one():
    """PX4's own convention: SITL's rcS does MAV_SYS_ID = instance + 1, and
    the ROS namespace is /vehicle{id+1}, so HITL uses the same offset."""
    spec = VehicleSpec(7, 'x500', None, (0.0,) * 4, mode='hitl',
                       fc_endpoint={'udp': '10.0.0.2:14560'})
    argv, _, _ = spawn_core.build_hitl_command(
        spec, 'c-track', '/opt/px4', '172.17.0.1', 14550)
    assert argv[argv.index('--sysid') + 1] == '8'


def test_build_hitl_command_sysid_override():
    """An explicit sys_id wins, for an FC whose MAV_SYS_ID is not id+1."""
    spec = VehicleSpec(7, 'x500', None, (0.0,) * 4, mode='hitl', sys_id=42,
                       fc_endpoint={'udp': '10.0.0.2:14560'})
    argv, _, _ = spawn_core.build_hitl_command(
        spec, 'c-track', '/opt/px4', '172.17.0.1', 14550)
    assert argv[argv.index('--sysid') + 1] == '42'


def test_build_hitl_command_udp():
    spec = VehicleSpec(1, 'x500', None, (0.0, 0.0, 0.2, 0.0),
                       mode='hitl', motors=4,
                       fc_endpoint={'udp': '192.168.1.36:14560',
                                    'local_port': 14541})
    argv, _, _ = spawn_core.build_hitl_command(
        spec, 'c-track', '/opt/px4', '172.17.0.1', 14550)
    assert argv[argv.index('--udp') + 1] == '192.168.1.36:14560'
    assert argv[argv.index('--local-port') + 1] == '14541'
    assert '--device' not in argv
    # an ethernet FC reaches QGC on its own GCS instance, so relaying a
    # second copy through the bridge is pure duplication -> off by default
    assert '--qgc' not in argv


def test_build_hitl_command_qgc_relay_override():
    """qgc_relay forces the relay on/off regardless of the link type."""
    base = dict(mode='hitl', motors=4)
    udp_on = VehicleSpec(1, 'x500', None, (0,) * 4, qgc_relay=True,
                         fc_endpoint={'udp': '10.0.0.2:14560'}, **base)
    serial_off = VehicleSpec(0, 'x500', None, (0,) * 4, qgc_relay=False,
                             fc_endpoint={'device': '/dev/ttyACM0'}, **base)
    on_argv, _, _ = spawn_core.build_hitl_command(
        udp_on, 'c-track', '/opt/px4', '172.17.0.1', 14550)
    off_argv, _, _ = spawn_core.build_hitl_command(
        serial_off, 'c-track', '/opt/px4', '172.17.0.1', 14550)
    assert on_argv[on_argv.index('--qgc') + 1] == '172.17.0.1:14550'
    assert '--qgc' not in off_argv


def test_build_hitl_command_requires_endpoint():
    spec = VehicleSpec(0, 'x500', None, (0.0, 0.0, 0.2, 0.0), mode='hitl')
    with pytest.raises(ValueError):
        spawn_core.build_hitl_command(spec, 'c-track', '/p', '1.2.3.4', 14550)
