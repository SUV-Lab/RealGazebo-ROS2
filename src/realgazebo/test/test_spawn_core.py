from realgazebo.yaml_config import VehicleSpec
from realgazebo import spawn_core


def test_build_create_argv():
    argv = spawn_core.build_create_argv(
        'x500', 3, '/tmp/models/x500.sdf', 'c-track', (1.0, 2.0, 3.0), (0.0, 0.0, 0.5))
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
    argv = spawn_core.build_remove_argv('c-track', 'x500', 2)
    assert argv == [
        'gz', 'service', '-s', '/world/c-track/remove',
        '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
        '--timeout', '3000', '--req', 'name: "x500_2" type: MODEL']


def test_build_set_pose_argv():
    argv = spawn_core.build_set_pose_argv(
        'c-track', 'rock', 9, (1.0, -2.0, 0.5), (0.0, 0.0, 0.1, 0.9))
    assert argv[:3] == ['gz', 'service', '-s']
    assert argv[3] == '/world/c-track/set_pose'
    assert '--reqtype' in argv and 'gz.msgs.Pose' in argv
    req = argv[-1]
    assert 'name: "rock_9"' in req
    assert 'position {x: 1.0, y: -2.0, z: 0.5}' in req
    assert 'orientation {x: 0.0, y: 0.0, z: 0.1, w: 0.9}' in req
