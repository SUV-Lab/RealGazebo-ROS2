import os

import pytest

from realgazebo.entity import Entity
from realgazebo.yaml_config import VehicleSpec
from realgazebo import spawn_core


# -- HIL actuator channels scanned from the rendered SDF ------------------

def _write_sdf(tmp_path, motors, moveables):
    body = ['<sdf><model name="m">',
            '<plugin filename="libRealGazebo.so" name="custom::RealGazebo">']
    if motors:
        body.append('<motorJointList>')
        body += [f'<motorJoint name="rotor_{i}_joint"/>' for i in range(motors)]
        body.append('</motorJointList>')
    if moveables:
        body.append('<moveableLinkList>')
        body += [f'<moveableLink name="surf_{i}_joint"/>' for i in range(moveables)]
        body.append('</moveableLinkList>')
    body.append('</plugin></model></sdf>')
    p = tmp_path / 'm.sdf'
    p.write_text(''.join(body))
    return str(p)


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


def test_build_hitl_command_serial(tmp_path):
    # --world/--motors/--servos MUST be passed (bridge defaults are wrong)
    spec = VehicleSpec(0, 'x500', None, (0.0, 0.0, 0.2, 0.0), mode='hitl',
                       fc_endpoint={'device': '/dev/ttyACM0', 'baud': 921600})
    argv, env, cwd = spawn_core.build_hitl_command(
        spec, 'c-track', '/opt/px4', '172.17.0.1', 14550,
        _write_sdf(tmp_path, 4, 0))
    assert argv == [
        '/opt/px4/build/px4_sitl_default/bin/gz-hitl-bridge',
        '--model', 'x500_0', '--world', 'c-track', '--sysid', '1',
        '--qgc', '172.17.0.1:14550',
        '--motors', '4', '--servos', '0',
        '--device', '/dev/ttyACM0', '--baud', '921600']
    assert env is None and cwd is None


def test_build_hitl_command_servos_from_the_model(tmp_path):
    """A VTOL's control surfaces follow its motors on the HIL channels."""
    spec = VehicleSpec(0, 'lc_62', None, (0.0,) * 4, mode='hitl',
                       fc_endpoint={'udp': '10.0.0.2:14560'})
    argv, _, _ = spawn_core.build_hitl_command(
        spec, 'c-track', '/opt/px4', '172.17.0.1', 14550,
        _write_sdf(tmp_path, 8, 5))
    assert argv[argv.index('--motors') + 1] == '8'
    assert argv[argv.index('--servos') + 1] == '5'


def test_build_hitl_command_sysid_defaults_to_id_plus_one(tmp_path):
    """PX4's own convention: SITL's rcS does MAV_SYS_ID = instance + 1, and
    the ROS namespace is /vehicle{id+1}, so HITL uses the same offset."""
    spec = VehicleSpec(7, 'x500', None, (0.0,) * 4, mode='hitl',
                       fc_endpoint={'udp': '10.0.0.2:14560'})
    argv, _, _ = spawn_core.build_hitl_command(
        spec, 'c-track', '/opt/px4', '172.17.0.1', 14550,
        _write_sdf(tmp_path, 4, 0))
    assert argv[argv.index('--sysid') + 1] == '8'


def test_build_hitl_command_local_port_defaults_to_base_plus_id(tmp_path):
    """14600 + id. PX4 leaves 146xx free, while 145xx is crowded: QGC 14550,
    SDK 14540, simulator 14560, plus SITL's per-instance 14550+N/14540+N.
    A 14540 + id scheme would land on QGC's own port at id 10."""
    spec = VehicleSpec(10, 'x500', None, (0.0,) * 4, mode='hitl',
                       fc_endpoint={'udp': '10.0.0.2:14560'})
    argv, _, _ = spawn_core.build_hitl_command(
        spec, 'c-track', '/opt/px4', '172.17.0.1', 14550,
        _write_sdf(tmp_path, 4, 0))
    assert argv[argv.index('--local-port') + 1] == '14610'


def test_build_hitl_command_sysid_override(tmp_path):
    """An explicit sys_id wins, for an FC whose MAV_SYS_ID is not id+1."""
    spec = VehicleSpec(7, 'x500', None, (0.0,) * 4, mode='hitl', sys_id=42,
                       fc_endpoint={'udp': '10.0.0.2:14560'})
    argv, _, _ = spawn_core.build_hitl_command(
        spec, 'c-track', '/opt/px4', '172.17.0.1', 14550,
        _write_sdf(tmp_path, 4, 0))
    assert argv[argv.index('--sysid') + 1] == '42'


def test_build_hitl_command_udp(tmp_path):
    spec = VehicleSpec(1, 'x500', None, (0.0, 0.0, 0.2, 0.0), mode='hitl',
                       fc_endpoint={'udp': '192.168.1.36:14560',
                                    'local_port': 14541})
    argv, _, _ = spawn_core.build_hitl_command(
        spec, 'c-track', '/opt/px4', '172.17.0.1', 14550,
        _write_sdf(tmp_path, 4, 0))
    assert argv[argv.index('--udp') + 1] == '192.168.1.36:14560'
    assert argv[argv.index('--local-port') + 1] == '14541'
    assert '--device' not in argv
    # an ethernet FC reaches QGC on its own GCS instance, so relaying a
    # second copy through the bridge is pure duplication -> off by default
    assert '--qgc' not in argv


def test_build_hitl_command_qgc_relay_override(tmp_path):
    """qgc_relay forces the relay on/off regardless of the link type."""
    base = dict(mode='hitl')
    udp_on = VehicleSpec(1, 'x500', None, (0,) * 4, qgc_relay=True,
                         fc_endpoint={'udp': '10.0.0.2:14560'}, **base)
    serial_off = VehicleSpec(0, 'x500', None, (0,) * 4, qgc_relay=False,
                             fc_endpoint={'device': '/dev/ttyACM0'}, **base)
    on_argv, _, _ = spawn_core.build_hitl_command(
        udp_on, 'c-track', '/opt/px4', '172.17.0.1', 14550,
        _write_sdf(tmp_path, 4, 0))
    off_argv, _, _ = spawn_core.build_hitl_command(
        serial_off, 'c-track', '/opt/px4', '172.17.0.1', 14550,
        _write_sdf(tmp_path, 4, 0))
    assert on_argv[on_argv.index('--qgc') + 1] == '172.17.0.1:14550'
    assert '--qgc' not in off_argv


def test_build_hitl_command_requires_endpoint(tmp_path):
    spec = VehicleSpec(0, 'x500', None, (0.0, 0.0, 0.2, 0.0), mode='hitl')
    with pytest.raises(ValueError):
        spawn_core.build_hitl_command(spec, 'c-track', '/p', '1.2.3.4', 14550,
                                      _write_sdf(tmp_path, 4, 0))


def test_scan_hil_actuators_counts_motors_and_servos(tmp_path):
    """The model declares the actuators; nothing is configured by hand."""
    assert spawn_core.scan_hil_actuators(_write_sdf(tmp_path, 8, 5)) == (8, 5)


def test_scan_hil_actuators_multirotor_has_no_servos(tmp_path):
    assert spawn_core.scan_hil_actuators(_write_sdf(tmp_path, 4, 0)) == (4, 0)


def test_scan_hil_actuators_rejects_a_model_without_motors(tmp_path):
    with pytest.raises(ValueError):
        spawn_core.scan_hil_actuators(_write_sdf(tmp_path, 0, 0))


def test_scan_hil_actuators_rejects_more_than_16_channels(tmp_path):
    # HIL_ACTUATOR_CONTROLS carries 16 channels (ActuatorOutputs.msg)
    with pytest.raises(ValueError):
        spawn_core.scan_hil_actuators(_write_sdf(tmp_path, 12, 5))


def test_scan_hil_actuators_against_the_real_lc62_template(tmp_path):
    """The shipped VTOL template must agree with its HITL airframe.

    1003_realgazebo_lc_62.hil assigns HIL_ACT_FUNC1..8 to motors (101..108)
    and 9..13 to servos (201..205); the bridge splits the HIL channels at
    the motor count this scan returns, so a drift between the two would
    silently drive the wrong actuators.
    """
    sdf = spawn_core.render_sdf(
        'lc_62', '127.0.0.1', 5005,
        models_dir=os.path.join(os.path.dirname(__file__), '..', 'models'),
        output_dir=str(tmp_path))
    assert spawn_core.scan_hil_actuators(sdf) == (8, 5)


def test_shipped_multirotor_templates_have_no_control_surfaces(tmp_path):
    models = os.path.join(os.path.dirname(__file__), '..', 'models')
    for vehicle_type, motors in (('x500', 4), ('x500_lidar_2d', 4)):
        sdf = spawn_core.render_sdf(vehicle_type, '127.0.0.1', 5005,
                                    models_dir=models, output_dir=str(tmp_path))
        assert spawn_core.scan_hil_actuators(sdf) == (motors, 0)
