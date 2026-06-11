import os
import re
import time
import signal
import threading
import subprocess

from .spawn_core import (
    render_sdf, build_create_argv, build_px4_command, build_param_argv)

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
            build_create_argv(spec.vehicle_type, spec.vehicle_id, sdf_path,
                              world, position, rpy),
            check=True)
        argv, px4_env, px4_cwd = build_px4_command(spec, world)
        # start_new_session so PX4 + any children form one killable group
        proc = subprocess.Popen(
            argv, env={**os.environ, **px4_env}, cwd=px4_cwd,
            start_new_session=True)
        threading.Thread(
            target=self._apply_params_later, args=(spec,), daemon=True).start()
        return proc

    def kill(self, handle):
        """Terminate the vehicle's process group (PX4 may fork children)."""
        if handle is None:
            return
        try:
            pgid = os.getpgid(handle.pid)
            os.killpg(pgid, signal.SIGTERM)
            try:
                handle.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass

    def alive(self, handle):
        """True while the vehicle's PX4 process is still running."""
        return handle is not None and handle.poll() is None

    def _apply_params_later(self, spec, delay=PARAM_APPLY_DELAY_SEC):
        """Best-effort PX4 param set once the instance has had time to boot."""
        time.sleep(delay)
        for name, value in POST_SPAWN_PARAMS:
            try:
                subprocess.run(
                    build_param_argv(spec, name, value), check=False, timeout=15)
            except Exception:
                pass


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
        command = (
            'source /opt/ros/jazzy/setup.bash && '
            'source /home/user/realgazebo/RealGazebo-ROS2/install/setup.bash && '
            f'ros2 launch realgazebo vehicle.launch.py '
            f'instance_id:={spec.vehicle_id} vehicle_type:={spec.vehicle_type} '
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
