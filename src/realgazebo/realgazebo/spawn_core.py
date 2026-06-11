import os

from jinja2 import Environment, FileSystemLoader

from .airframes import get_autostart_id

MODEL_OUTPUT_DIR = '/tmp/models'


def render_sdf(vehicle_type, unreal_ip, unreal_port,
               models_dir=None, output_dir=MODEL_OUTPUT_DIR):
    """Render realgazebo's {type}.sdf.jinja to {output_dir}/{type}.sdf; return path."""
    if models_dir is None:
        # lazy import so this module imports/unit-tests on a host without ROS
        from ament_index_python.packages import get_package_share_directory
        models_dir = os.path.join(get_package_share_directory('realgazebo'), 'models')
    env = Environment(loader=FileSystemLoader(models_dir))
    template = env.get_template(f'{vehicle_type}.sdf.jinja')
    rendered = template.render(unreal_ip=unreal_ip, unreal_port=unreal_port)
    os.makedirs(output_dir, exist_ok=True)
    out_path = os.path.join(output_dir, f'{vehicle_type}.sdf')
    with open(out_path, 'w') as f:
        f.write(rendered)
    return out_path


def build_create_argv(vehicle_type, vehicle_id, sdf_path, world, position, rpy):
    """Build the `ros2 run ros_gz_sim create` argv for one vehicle.

    position is (x, y, z) in meters; rpy is (roll, pitch, yaw) in radians.
    """
    x, y, z = position
    roll, pitch, yaw = rpy
    return [
        'ros2', 'run', 'ros_gz_sim', 'create',
        '-world', world, '-file', sdf_path,
        '-name', f'{vehicle_type}_{vehicle_id}',
        '-x', str(x), '-y', str(y), '-z', str(z),
        '-R', str(roll), '-P', str(pitch), '-Y', str(yaw),
    ]


def build_px4_command(spec, world):
    """Return (argv, env, cwd).

    env holds only PX4_GZ_* (caller merges with os.environ). cwd is the PX4
    build dir containing etc/init.d-posix/rcS; PX4 resolves its rootfs
    relative to the working directory, so it must run from there.
    """
    autostart_id = get_autostart_id(spec.vehicle_type, spec.build_target_path)
    env = {
        'PX4_GZ_STANDALONE': '1',
        'PX4_SYS_AUTOSTART': autostart_id,
        'PX4_GZ_MODEL_NAME': f'{spec.vehicle_type}_{spec.vehicle_id}',
        'PX4_UXRCE_DDS_NS': f'vehicle{spec.vehicle_id + 1}',
        'PX4_GZ_WORLD': world,
    }
    build_dir = os.path.join(spec.build_target_path, 'build/px4_sitl_default')
    argv = [os.path.join(build_dir, 'bin/px4'), '-i', str(spec.vehicle_id)]
    return argv, env, build_dir


def build_param_argv(spec, name, value):
    binary = os.path.join(
        spec.build_target_path, 'build/px4_sitl_default/bin/px4-param')
    return [binary, '--instance', str(spec.vehicle_id), 'set', name, str(value)]


def build_remove_argv(world, vehicle_type, vehicle_id):
    """Build a `gz service` call to remove a spawned model entity by name."""
    name = f'{vehicle_type}_{vehicle_id}'
    return [
        'gz', 'service', '-s', f'/world/{world}/remove',
        '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
        '--timeout', '3000', '--req', f'name: "{name}" type: MODEL',
    ]
