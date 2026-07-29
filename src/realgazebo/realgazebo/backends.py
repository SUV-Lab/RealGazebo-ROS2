import os
import re
import time
import types
import signal
import threading
import subprocess

import yaml

from .spawn_core import (
    render_sdf, build_create_argv, build_px4_command, build_param_argv,
    build_hitl_command)
from .vehicle_extras import (
    VEHICLE_CAMERAS, RTSP_PORT, get_sensor_bridges,
    build_image_receiver_argv, build_sensor_bridge_argv)

# PX4 params applied after spawn (same as legacy realgazebo.launch.py)
POST_SPAWN_PARAMS = [
    ('NAV_DLL_ACT', 0),
    ('COM_RCL_EXCEPT', 31),
    ('COM_RC_IN_MODE', 4),
]
PARAM_APPLY_DELAY_SEC = 8.0


class SubprocessBackend:
    """Monolithic-mode backend: materialize one vehicle as local subprocesses.

    launch() renders the SDF, creates the gz entity, and boots PX4 SITL in
    its own process group; kill() tears the group down. The v3 DockerBackend
    implements the same launch()/kill() contract by running one vehicle
    container (which does its own SDF render / gz spawn / PX4) instead.
    Model removal on despawn is NOT a backend concern — the manager removes
    the gz entity itself in both modes.
    """

    def launch(self, spec, world, position, rpy, unreal_ip, unreal_port,
               roster=None):
        """Spawn the vehicle; return an opaque handle for kill().

        roster is accepted for interface parity with DockerBackend and
        ignored here (the monolithic mode has no per-container network_sim).
        """
        sdf_path = render_sdf(spec.vehicle_type, unreal_ip, unreal_port)
        subprocess.run(
            build_create_argv(spec.entity, sdf_path, world, position, rpy),
            check=True)
        argv, px4_env, px4_cwd = build_px4_command(spec, world)
        # start_new_session so PX4 + any children form one killable group
        proc = subprocess.Popen(
            argv, env={**os.environ, **px4_env}, cwd=px4_cwd,
            start_new_session=True)
        extras = self._launch_extras(spec, world, sdf_path, unreal_ip)
        threading.Thread(
            target=self._apply_params_later, args=(spec,), daemon=True).start()
        # Composite handle: PX4 decides liveness; extras (each its own
        # session, so the ros2-run wrapper and its node die together) are
        # torn down alongside it. A process can only join groups within its
        # own session, so the extras cannot share PX4's group.
        return types.SimpleNamespace(pid=proc.pid, px4=proc, extras=extras)

    def _launch_extras(self, spec, world, sdf_path, unreal_ip):
        """Per-vehicle extras, matching what vehicle.launch.py provides in
        docker mode: camera receivers (UNCONFIGURED; image_viewer drives
        their lifecycle) and the lidar sensor bridge. Each runs in its own
        session/group so killpg reaps the ros2-run wrapper together with
        the node it spawns. Best-effort: a failing extra must not fail the
        spawn itself. Returns the started Popen handles.
        """
        extras = []
        try:
            for camera_type in VEHICLE_CAMERAS.get(spec.vehicle_type, ['front']):
                extras.append(subprocess.Popen(
                    build_image_receiver_argv(
                        spec.vehicle_type, spec.vehicle_id, camera_type,
                        unreal_ip, RTSP_PORT),
                    start_new_session=True))

            # HITL vehicles have no build_target_path; fall back to the
            # backend's px4_path (HitlBackend sets _px4_path) for the gz model
            # search dir used by the lidar sensor bridge.
            search_base = spec.build_target_path or getattr(self, '_px4_path', None)
            search_paths = (
                [os.path.join(search_base, 'Tools/simulation/gz/models')]
                if search_base else [])
            try:
                from ament_index_python.packages import get_package_share_directory
                search_paths.insert(0, os.path.join(
                    get_package_share_directory('realgazebo'), 'models'))
            except Exception:
                pass
            bridges = get_sensor_bridges(
                spec.vehicle_type, spec.vehicle_id, world, sdf_path, search_paths)
            if bridges:
                os.makedirs('/tmp/bridges', exist_ok=True)
                config_path = f'/tmp/bridges/{spec.vehicle_type}_{spec.vehicle_id}.yaml'
                with open(config_path, 'w') as f:
                    yaml.dump(bridges, f)
                extras.append(subprocess.Popen(
                    build_sensor_bridge_argv(
                        config_path,
                        f'sensor_bridge_{spec.vehicle_type}_{spec.vehicle_id}'),
                    start_new_session=True))
        except Exception as exc:
            print(f'[SubprocessBackend] vehicle extras failed for '
                  f'{spec.vehicle_type}_{spec.vehicle_id}: {exc}', flush=True)
        return extras

    def kill(self, handle):
        """Terminate the vehicle's process groups (PX4 + extras)."""
        if handle is None:
            return
        px4 = getattr(handle, 'px4', handle)
        for proc in [px4, *getattr(handle, 'extras', [])]:
            self._kill_group(proc)

    @staticmethod
    def _kill_group(proc):
        if proc is None:
            return
        try:
            pgid = os.getpgid(proc.pid)
            os.killpg(pgid, signal.SIGTERM)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass

    def alive(self, handle):
        """True while the vehicle's PX4 process is still running."""
        px4 = getattr(handle, 'px4', handle)
        return px4 is not None and px4.poll() is None

    def _apply_params_later(self, spec, delay=PARAM_APPLY_DELAY_SEC):
        """Best-effort PX4 param set once the instance has had time to boot."""
        time.sleep(delay)
        for name, value in POST_SPAWN_PARAMS:
            try:
                subprocess.run(
                    build_param_argv(spec, name, value), check=False, timeout=15)
            except Exception:
                pass


class HitlBackend(SubprocessBackend):
    """HITL backend: a real flight controller drives the vehicle.

    Subclasses SubprocessBackend to reuse its extras (camera receivers +
    lidar sensor bridge), kill() and alive(), but there is NO PX4 SITL
    process. launch() creates the gz entity (so UE telemetry via
    libRealGazebo.so and the MulticopterMotorModel joints exist), starts the
    same per-vehicle extras as a SITL vehicle (so image_viewer / lidar work
    identically), and Popen's gz-hitl-bridge, which relays MAVLink HIL
    between the shared gz model and the real FC over serial/UDP and relays
    FC<->QGC. No PX4 post-spawn params (the FC owns its own).

    Runs as a manager-local subprocess in BOTH fleet modes: the manager /
    gazebo container already has /dev (serial FCs), GZ_PARTITION=realgazebo
    (shared world) and the host route (QGC), so a HITL vehicle never needs
    its own container, device passthrough, vehicle-network or DDS. The bridge
    Popen inherits the manager's env, so GZ_PARTITION / GZ_IP carry through.
    """

    def __init__(self, px4_path, qgc_host, qgc_port=14550):
        self._px4_path = px4_path
        self._qgc_host = qgc_host
        self._qgc_port = qgc_port

    def launch(self, spec, world, position, rpy, unreal_ip, unreal_port,
               roster=None):
        """Create the gz entity, its extras, and the bridge; return a handle.

        roster is accepted for interface parity and ignored (a HITL vehicle
        has no PX4 SITL / network_sim participant of its own).
        """
        sdf_path = render_sdf(spec.vehicle_type, unreal_ip, unreal_port)
        subprocess.run(
            build_create_argv(spec.entity, sdf_path, world, position, rpy),
            check=True)
        argv, env, cwd = build_hitl_command(
            spec, world, self._px4_path, self._qgc_host, self._qgc_port,
            sdf_path)
        # start_new_session so the bridge is a killable group; inherit env so
        # GZ_PARTITION / GZ_IP reach the bridge and it sees the shared world.
        proc = subprocess.Popen(
            argv, env=({**os.environ, **env} if env else None), cwd=cwd,
            start_new_session=True)
        # Same per-vehicle extras a SITL vehicle gets (camera receivers, lidar
        # bridge). The image_viewer needs the camera receiver's lifecycle
        # service to exist, so a HITL vehicle must start these too.
        extras = self._launch_extras(spec, world, sdf_path, unreal_ip)
        # Composite handle shape shared with SubprocessBackend (px4 slot holds
        # the bridge) so the inherited kill()/alive() and the manager's crash
        # watcher work unchanged.
        return types.SimpleNamespace(pid=proc.pid, px4=proc, extras=extras)


class PilsBackend(SubprocessBackend):
    """PILS backend: the vehicle's PX4 SITL runs on another host.

    Same division of labour as HITL — the manager owns only the gz side:
    launch() creates the gz entity and the per-vehicle extras (camera
    receivers, lidar bridge). The autopilot is a PX4 SITL container on a
    remote PC (scripts/run_pils_vehicle.sh) that attaches to the shared
    world over gz-transport (PX4_GZ_MODEL_NAME + GZ_PARTITION, both hosts
    advertising a routable GZ_IP). There is no local autopilot process, so
    alive() is unconditionally True: the remote peer may come and go
    (container restart, PC reboot) without the crash watcher reaping the
    model — exactly like a power-cycled HITL FC.
    """

    def __init__(self, px4_path):
        # only used by the inherited _launch_extras as a model search dir
        self._px4_path = px4_path

    def launch(self, spec, world, position, rpy, unreal_ip, unreal_port,
               roster=None):
        """Create the gz entity and its extras; return a handle.

        roster is accepted for interface parity and ignored (the remote
        SITL is not a local network_sim participant).
        """
        sdf_path = render_sdf(spec.vehicle_type, unreal_ip, unreal_port)
        subprocess.run(
            build_create_argv(spec.entity, sdf_path, world, position, rpy),
            check=True)
        extras = self._launch_extras(spec, world, sdf_path, unreal_ip)
        # px4 slot deliberately None: the inherited kill() skips it and
        # reaps only the extras.
        return types.SimpleNamespace(pid=None, px4=None, extras=extras)

    def alive(self, handle):
        return True


class DockerBackend:
    """Multi-container backend: one vehicle = one sibling docker container.

    Replicates at runtime the per-vehicle service that generate_compose.py
    used to emit statically. The container runs vehicle.launch.py, which
    renders its own SDF, spawns the gz entity, writes its DDS isolation
    profile, and boots PX4 — so launch() here only assembles the container.
    Requires /var/run/docker.sock mounted into the manager's container.
    Note: vehicle.launch.py only supports yaw at spawn, so roll/pitch from
    a UDP pose are dropped in this mode.
    """

    def __init__(self, image, px4_path, gazebo_network, vehicle_network,
                 mavlink_gcs_ip, roster_fn, client=None):
        from .docker_api import DockerClient
        self._docker = client if client is not None else DockerClient()
        self._image = image
        self._px4_path = px4_path
        self._gazebo_network = gazebo_network
        self._vehicle_network = vehicle_network
        self._mavlink_gcs_ip = mavlink_gcs_ip
        # roster_fn() -> list of active 'type_id' names (for network_sim V2V)
        self._roster_fn = roster_fn

    def launch(self, spec, world, position, rpy, unreal_ip, unreal_port,
               roster=None):
        """Spawn one vehicle container; return its container id.

        roster: complete vehicle_models list for network_sim. Boot-time YAML
        spawns pass the full fleet (known upfront); when omitted (runtime UDP
        spawns) it falls back to currently-active vehicles + this one.
        """
        from .allocator import allocate
        res = allocate(spec.vehicle_id)
        name = f'vehicle_{spec.vehicle_id}'
        model_name = f'{spec.vehicle_type}_{spec.vehicle_id}'
        if roster is None:
            roster = sorted(set(self._roster_fn()) | {model_name})
        x, y, z = position
        # world:= is not optional. The container spawns its own model (unlike
        # the subprocess backend, where the manager does it), so if it is left
        # to vehicle.launch.py's default the container and the manager address
        # different worlds the moment `world` is not 'c-track' - the model
        # lands in one world while every despawn is sent to the other, and
        # _despawn_one swallows the failure.
        command = (
            'source /opt/ros/jazzy/setup.bash && '
            'source /home/user/realgazebo/RealGazebo-ROS2/install/setup.bash && '
            f'ros2 launch realgazebo vehicle.launch.py '
            f'instance_id:={spec.vehicle_id} vehicle_type:={spec.vehicle_type} '
            f'world:={world} '
            f'spawnpoint:={x},{y},{z},{rpy[2]} px4_path:={self._px4_path} '
            f'unreal_ip:={unreal_ip} unreal_port:={unreal_port} '
            f'vehicle_models:={",".join(roster)}'
        )
        port_key = f'{res.mavlink_port}/udp'
        config = {
            'Image': self._image,
            'Hostname': name,
            'Env': [
                f'DISPLAY={os.environ.get("DISPLAY", ":0")}',
                'QT_X11_NO_MITSHM=1',
                f'GZ_IP={res.gz_ip}',
                'GZ_PARTITION=realgazebo',
                'PX4_GZ_STANDALONE=1',
                # LOCAL_USER_ID intentionally NOT set: the image entrypoint
                # would usermod/chown the multi-GB baked home dir, delaying
                # vehicle boot by minutes. Runtime-spawned containers are
                # ephemeral and privileged anyway, so they run as root.
                f'MAVLINK_GCS_IP={self._mavlink_gcs_ip}',
                f'FASTRTPS_DEFAULT_PROFILES_FILE={res.dds_profile_path}',
            ],
            'Cmd': ['bash', '-c', command],
            'ExposedPorts': {port_key: {}},
            'HostConfig': {
                'Privileged': True,
                'Binds': ['/tmp/.X11-unix:/tmp/.X11-unix'],
                'PortBindings': {port_key: [{'HostPort': str(res.mavlink_port)}]},
                'ExtraHosts': ['host.docker.internal:host-gateway'],
                'Memory': 4 * 1024 ** 3,
                'NetworkMode': self._gazebo_network,
            },
            'NetworkingConfig': {
                'EndpointsConfig': {
                    self._gazebo_network: {
                        'IPAMConfig': {'IPv4Address': res.gz_ip}},
                },
            },
        }
        container_id = self._docker.create_container(name, config)
        try:
            # the API attaches one network at create; the isolated DDS
            # network must be connected before start
            self._docker.connect_network(
                self._vehicle_network, container_id, ipv4=res.vehicle_ip)
            self._docker.start_container(container_id)
        except Exception:
            self._docker.remove_container(container_id, force=True)
            raise
        return container_id

    def kill(self, handle):
        # Force-remove in one call: sim vehicles are disposable, and the
        # shutdown teardown must beat ros2 launch's SIGKILL escalation —
        # a graceful per-container stop (seconds each) does not.
        if not handle:
            return
        self._docker.remove_container(handle, force=True)

    def alive(self, handle):
        """True while the vehicle's container is still running."""
        if not handle:
            return False
        try:
            return bool(self._docker.inspect_container(handle)['State']['Running'])
        except Exception:
            return False

    def find_existing(self):
        """Discover vehicle containers left over from a previous manager run.

        Returns [(vehicle_type, vehicle_id, container_id)] for every running
        container named vehicle_<id>, recovering the type from the
        vehicle_type:= argument in its command. Lets a restarted manager
        adopt (and later despawn) survivors of an uncleanly killed manager.
        """
        found = []
        for entry in self._docker.list_containers(all_states=False):
            for name in entry.get('Names', []):
                m = re.fullmatch(r'/vehicle_(\d+)', name)
                if not m:
                    continue
                cmd = ' '.join(entry.get('Command', '').split())
                t = re.search(r'vehicle_type:=(\S+)', cmd)
                if t:
                    found.append((t.group(1), int(m.group(1)), entry['Id']))
        return found


def make_backend(name: str, **opts):
    """Backend factory keyed by the manager's 'backend' parameter."""
    if name == 'subprocess':
        return SubprocessBackend()
    if name == 'docker':
        return DockerBackend(
            image=opts['image'],
            px4_path=opts['px4_path'],
            gazebo_network=opts['gazebo_network'],
            vehicle_network=opts['vehicle_network'],
            mavlink_gcs_ip=opts['mavlink_gcs_ip'],
            roster_fn=opts['roster_fn'],
        )
    raise ValueError(f"unknown backend '{name}' (supported: subprocess, docker)")
