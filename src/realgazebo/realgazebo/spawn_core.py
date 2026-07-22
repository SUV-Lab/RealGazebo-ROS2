import os

from jinja2 import Environment, FileSystemLoader, TemplateNotFound

from .airframes import get_autostart_id

MODEL_OUTPUT_DIR = '/tmp/models'

# Base for a HITL bridge's own UDP port ({base} + vehicle_id). 146xx is
# unused by PX4, unlike 145xx (QGC 14550, SDK 14540, simulator 14560, and
# SITL's per-instance 14550+N / 14540+N).
HITL_LOCAL_PORT_BASE = 14600


def render_sdf(vehicle_type, unreal_ip, unreal_port,
               models_dir=None, output_dir=MODEL_OUTPUT_DIR):
    """Render realgazebo's {type}.sdf.jinja to {output_dir}/{type}.sdf; return path."""
    if models_dir is None:
        # lazy import so this module imports/unit-tests on a host without ROS
        from ament_index_python.packages import get_package_share_directory
        models_dir = os.path.join(get_package_share_directory('realgazebo'), 'models')
    env = Environment(loader=FileSystemLoader(models_dir))
    try:
        template = env.get_template(f'{vehicle_type}.sdf.jinja')
    except TemplateNotFound:
        # obstacle-style templates live in <name>/<name>.sdf.jinja
        # (same convention gazebo.launch.py scans for the gz server)
        template = env.get_template(f'{vehicle_type}/{vehicle_type}.sdf.jinja')
    rendered = template.render(unreal_ip=unreal_ip, unreal_port=unreal_port)
    os.makedirs(output_dir, exist_ok=True)
    out_path = os.path.join(output_dir, f'{vehicle_type}.sdf')
    with open(out_path, 'w') as f:
        f.write(rendered)
    return out_path


def build_create_argv(entity, sdf_path, world, position, rpy):
    """Build the `ros2 run ros_gz_sim create` argv for one entity.

    position is (x, y, z) in meters; rpy is (roll, pitch, yaw) in radians.
    """
    x, y, z = position
    roll, pitch, yaw = rpy
    return [
        'ros2', 'run', 'ros_gz_sim', 'create',
        '-world', world, '-file', sdf_path,
        '-name', entity.name,
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


def build_hitl_command(spec, world, px4_path, qgc_host, qgc_port):
    """Return (argv, env, cwd) for the gz-hitl-bridge process of a HITL vehicle.

    A HITL vehicle has NO PX4 SITL process; a real flight controller runs the
    firmware and the bridge relays MAVLink HIL between the shared gz model and
    that FC over serial (device+baud) or Ethernet (udp+local_port). px4_path
    locates the bridge binary in the PX4 build tree. env/cwd are None so the
    process inherits the manager's environment (GZ_PARTITION / GZ_IP) and
    thus sees the shared world's topics.

    Two RealGazebo-specific values that MUST be passed (bridge defaults are
    wrong here): --world (RealGazebo world is not the bridge default) and
    --motors (x500 is a quad; bridge defaults to 8).

    --sysid is the MAVLink system id the bridge stamps on the HIL messages it
    sends. It defaults to vehicle_id + 1, matching PX4's own convention
    (SITL's rcS sets MAV_SYS_ID = instance + 1, and the ROS namespace is
    /vehicle{id+1}), so a HITL vehicle shares the numbering of its SITL
    siblings. spec.sys_id overrides it for an FC whose MAV_SYS_ID differs.
    Without a distinct sysid per vehicle, every bridge in a multi-HITL fleet
    would transmit as system 1 and QGC would merge them into one vehicle.

    --qgc (the bridge's FC<->QGC relay) is opt-in. A serial FC has a single
    pipe carrying HIL and telemetry, so the relay is QGC's ONLY path and is
    enabled. An ethernet FC keeps its own GCS MAVLink instance, so QGC
    connects directly and a relayed second copy is pure duplication (measured
    1219 pps of redundant loopback traffic) - disabled. spec.qgc_relay
    overrides the choice either way.
    """
    binary = os.path.join(
        px4_path, 'build/px4_sitl_default/bin/gz-hitl-bridge')
    sysid = spec.sys_id if spec.sys_id is not None else spec.vehicle_id + 1
    argv = [
        binary,
        '--model', f'{spec.vehicle_type}_{spec.vehicle_id}',
        '--world', world,
        '--sysid', str(sysid),
    ]
    relay = spec.qgc_relay
    if relay is None:
        relay = not (spec.fc_endpoint or {}).get('udp')
    if relay:
        argv += ['--qgc', f'{qgc_host}:{qgc_port}']
    if spec.motors is not None:
        argv += ['--motors', str(spec.motors)]
    fc = spec.fc_endpoint or {}
    if fc.get('device'):
        argv += ['--device', str(fc['device'])]
        if fc.get('baud') is not None:
            argv += ['--baud', str(fc['baud'])]
    elif fc.get('udp'):
        # Local port the bridge binds for this vehicle's FC socket. Derived
        # as HITL_LOCAL_PORT_BASE + id so a fleet never collides; fc.local_port
        # overrides it. 14540 + id (the obvious choice) would hit QGC's 14550
        # at id 10 and reproduce the self-feeding storm documented in HITL.md.
        local_port = fc.get('local_port')
        if local_port is None:
            local_port = HITL_LOCAL_PORT_BASE + spec.vehicle_id
        argv += ['--udp', str(fc['udp']), '--local-port', str(local_port)]
    else:
        raise ValueError(
            f"HITL vehicle {spec.vehicle_type}_{spec.vehicle_id} needs an "
            f"fc: endpoint (device+baud for serial, or udp+local_port)")
    return argv, None, None


def build_param_argv(spec, name, value):
    binary = os.path.join(
        spec.build_target_path, 'build/px4_sitl_default/bin/px4-param')
    return [binary, '--instance', str(spec.vehicle_id), 'set', name, str(value)]


def build_remove_argv(world, entity):
    """Build a `gz service` call to remove a spawned model entity by name."""
    return [
        'gz', 'service', '-s', f'/world/{world}/remove',
        '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
        '--timeout', '3000', '--req', f'name: "{entity.name}" type: MODEL',
    ]


def build_set_pose_argv(world, entity, position, quaternion):
    """Build a `gz service` call to teleport an existing entity (prop move).

    quaternion is (x, y, z, w) in the Gazebo frame, straight from the wire.
    """
    name = entity.name
    x, y, z = position
    qx, qy, qz, qw = quaternion
    req = (f'name: "{name}" '
           f'position {{x: {x}, y: {y}, z: {z}}} '
           f'orientation {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}')
    return [
        'gz', 'service', '-s', f'/world/{world}/set_pose',
        '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
        '--timeout', '1000', '--req', req,
    ]
