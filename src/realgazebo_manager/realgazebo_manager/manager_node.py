import os
import time
import signal
import socket
import threading
import subprocess

import yaml
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

from .yaml_config import parse_vehicles, VehicleSpec
from .spawn_core import (
    render_sdf, build_create_argv, build_px4_command, build_param_argv,
    build_remove_argv)
from .registry import VehicleRegistry
from .protocol import parse_packet, DespawnCommand
from .vehicle_codes import type_for_code
from .geometry import quat_to_euler

# PX4 params applied right after spawn (same as legacy realgazebo.launch.py)
POST_SPAWN_PARAMS = [
    ('NAV_DLL_ACT', 0),
    ('COM_RCL_EXCEPT', 31),
    ('COM_RC_IN_MODE', 4),
]
SPAWN_STAGGER_SEC = 0.5
PARAM_APPLY_DELAY_SEC = 8.0


class ManagerNode(Node):
    def __init__(self):
        super().__init__('realgazebo_manager')
        self.declare_parameter('yaml_path', '')
        self.declare_parameter('world', 'c-track')
        self.declare_parameter('unreal_ip', '127.0.0.1')
        self.declare_parameter('unreal_port', 5005)
        self.declare_parameter('spawn_udp_port', 5006)
        self.declare_parameter(
            'default_px4_path', '/home/user/realgazebo/RealGazebo-PX4')
        self.registry = VehicleRegistry()
        self._spawn_lock = threading.Lock()
        self._clock_seen = False
        self._stop = False
        self._sock = None
        self.create_subscription(Clock, '/clock', self._on_clock, 10)

    # -- startup ----------------------------------------------------------
    def _on_clock(self, _msg):
        self._clock_seen = True

    def wait_for_clock(self, timeout_sec=60.0):
        """Block until Gazebo publishes /clock (i.e. the simulator is ready)."""
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not self._clock_seen:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() > deadline:
                raise TimeoutError("Gazebo /clock not received within timeout")
        self.get_logger().info("Gazebo /clock detected; starting spawn")

    # -- spawn core (one trigger-agnostic path) ---------------------------
    def _spawn_one(self, vehicle_type, vehicle_id, position, rpy,
                   build_target_path, world):
        """Spawn a single vehicle: render SDF, create the gz entity, launch PX4.

        Idempotent: a (type, id) already active is skipped. Called from both
        the boot-time YAML loop and the UDP listener thread, so the registry
        mutation is guarded by a lock.
        """
        with self._spawn_lock:
            if self.registry.is_active(vehicle_type, vehicle_id):
                self.get_logger().warn(
                    f"skip duplicate {vehicle_type}_{vehicle_id}")
                return
            unreal_ip = self.get_parameter('unreal_ip').value
            unreal_port = self.get_parameter('unreal_port').value
            sdf_path = render_sdf(vehicle_type, unreal_ip, unreal_port)
            subprocess.run(
                build_create_argv(vehicle_type, vehicle_id, sdf_path,
                                  world, position, rpy),
                check=True)
            spec = VehicleSpec(
                vehicle_id, vehicle_type, build_target_path,
                (position[0], position[1], position[2], rpy[2]))
            argv, px4_env, px4_cwd = build_px4_command(spec, world)
            # start_new_session so PX4 + any children get their own process
            # group, which despawn can kill as a unit (no orphans).
            proc = subprocess.Popen(
                argv, env={**os.environ, **px4_env}, cwd=px4_cwd,
                start_new_session=True)
            record = self.registry.add(vehicle_type, vehicle_id)
            record.process = proc
            self.get_logger().info(
                f"spawned {vehicle_type}_{vehicle_id} (px4 pid {proc.pid})")
        threading.Thread(
            target=self._apply_params_later, args=(spec,), daemon=True).start()

    def _apply_params_later(self, spec, delay=PARAM_APPLY_DELAY_SEC):
        """Best-effort: set PX4 params once the instance has had time to boot."""
        time.sleep(delay)
        for name, value in POST_SPAWN_PARAMS:
            try:
                subprocess.run(
                    build_param_argv(spec, name, value), check=False, timeout=15)
            except Exception:
                pass

    # -- boot-time YAML trigger -------------------------------------------
    def spawn_all(self):
        yaml_path = self.get_parameter('yaml_path').value
        world = self.get_parameter('world').value
        if not yaml_path:
            return
        with open(yaml_path) as f:
            config = yaml.safe_load(f)
        for spec in parse_vehicles(config):
            x, y, z, yaw = spec.spawnpoint
            self._spawn_one(spec.vehicle_type, spec.vehicle_id, (x, y, z),
                            (0.0, 0.0, yaw), spec.build_target_path, world)
            time.sleep(SPAWN_STAGGER_SEC)
        self.get_logger().info(
            f"spawn complete: {[f'{t}_{i}' for t, i in self.registry.active_ids()]}")

    # -- runtime UDP trigger ----------------------------------------------
    def start_udp_listener(self):
        port = self.get_parameter('spawn_udp_port').value
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(('0.0.0.0', port))
        self._sock.settimeout(0.5)
        threading.Thread(target=self._udp_loop, daemon=True).start()
        self.get_logger().info(f"listening for UDP spawn commands on :{port}")

    def _udp_loop(self):
        world = self.get_parameter('world').value
        default_px4 = self.get_parameter('default_px4_path').value
        while not self._stop:
            try:
                data, _ = self._sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                cmd = parse_packet(data)
            except ValueError as exc:
                self.get_logger().warn(f"bad packet: {exc}")
                continue
            if cmd is None:
                continue  # not a spawn/despawn command
            try:
                vehicle_type = type_for_code(cmd.vehicle_code)
            except ValueError as exc:
                self.get_logger().warn(str(exc))
                continue
            if isinstance(cmd, DespawnCommand):
                self.get_logger().info(
                    f"UDP despawn: {vehicle_type}_{cmd.vehicle_num}")
                try:
                    self._despawn_one(vehicle_type, cmd.vehicle_num, world)
                except Exception as exc:
                    self.get_logger().error(f"despawn failed: {exc}")
                continue
            rpy = quat_to_euler(*cmd.quaternion)
            self.get_logger().info(
                f"UDP spawn: {vehicle_type}_{cmd.vehicle_num} at {cmd.position}")
            try:
                self._spawn_one(vehicle_type, cmd.vehicle_num, cmd.position,
                                rpy, default_px4, world)
            except Exception as exc:
                self.get_logger().error(f"spawn failed: {exc}")

    def _despawn_one(self, vehicle_type, vehicle_id, world):
        """Kill the vehicle's PX4 process and remove its gz model entity."""
        with self._spawn_lock:
            if not self.registry.is_active(vehicle_type, vehicle_id):
                self.get_logger().warn(
                    f"despawn: {vehicle_type}_{vehicle_id} not active")
                return
            record = self.registry.remove(vehicle_type, vehicle_id)
        if record.process is not None:
            # PX4 may fork children; kill the whole process group so nothing
            # is orphaned (the group was created via start_new_session).
            try:
                pgid = os.getpgid(record.process.pid)
                os.killpg(pgid, signal.SIGTERM)
                try:
                    record.process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        subprocess.run(
            build_remove_argv(world, vehicle_type, vehicle_id),
            check=False, timeout=10)
        self.get_logger().info(f"despawned {vehicle_type}_{vehicle_id}")

    def shutdown(self):
        self._stop = True
        if self._sock is not None:
            self._sock.close()


def main(args=None):
    rclpy.init(args=args)
    node = ManagerNode()
    try:
        node.wait_for_clock()
        node.spawn_all()
        node.start_udp_listener()
        rclpy.spin(node)  # keep node alive: hold subprocesses + serve UDP spawns
    except (KeyboardInterrupt, TimeoutError) as exc:
        node.get_logger().error(str(exc))
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
