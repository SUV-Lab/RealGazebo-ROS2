import time
import signal
import socket
import threading
import subprocess

import yaml
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

from .yaml_config import parse_vehicles, VehicleSpec
from .spawn_core import (
    render_sdf, build_create_argv, build_remove_argv, build_set_pose_argv)
from .backends import make_backend
from .registry import VehicleRegistry
from .protocol import parse_packet, DespawnCommand
from .vehicle_codes import (
    scan_vehicle_codes, type_for_code, code_for_type, is_prop_code)
from .geometry import quat_to_euler

SPAWN_STAGGER_SEC = 0.5


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
        # execution backend: how a spawned vehicle is materialized
        # ('subprocess' = monolithic mode, 'docker' = one container per vehicle)
        self.declare_parameter('backend', 'subprocess')
        self.declare_parameter('docker_image', 'mdeagewt/realgazebo:ue5.7')
        self.declare_parameter('docker_gazebo_network', 'gazebo-network')
        self.declare_parameter('docker_vehicle_network', 'vehicle-network')
        self.declare_parameter('mavlink_gcs_ip', '172.17.0.1')
        self.registry = VehicleRegistry()
        # code->type map scanned from the model templates (single source of
        # truth shared with the gz plugin's <vehicle_code> SDF element)
        self._code_map = scan_vehicle_codes()
        self.backend = make_backend(
            self.get_parameter('backend').value,
            image=self.get_parameter('docker_image').value,
            px4_path=self.get_parameter('default_px4_path').value,
            gazebo_network=self.get_parameter('docker_gazebo_network').value,
            vehicle_network=self.get_parameter('docker_vehicle_network').value,
            mavlink_gcs_ip=self.get_parameter('mavlink_gcs_ip').value,
            # props are not V2V participants, keep them out of the roster
            roster_fn=lambda: [
                f'{t}_{i}' for t, i in self.registry.active_ids()
                if not self.registry.get(t, i).prop],
        )
        self._spawn_lock = threading.Lock()
        self._clock_seen = False
        self._stop = False
        self._sock = None
        self.create_subscription(Clock, '/clock', self._on_clock, 10)
        # watch spawned vehicles so a crashed PX4/container frees its slot
        self.create_timer(5.0, self._check_vehicles)

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
    def _is_prop_type(self, vehicle_type):
        """True when the type is a static prop (wire code >= 200)."""
        try:
            return is_prop_code(code_for_type(vehicle_type, self._code_map))
        except ValueError:
            return False

    def _spawn_one(self, vehicle_type, vehicle_id, position, rpy,
                   build_target_path, world, roster=None):
        """Spawn a single vehicle or prop.

        Idempotent: a (type, id) already active is skipped, and an id held
        by a DIFFERENT type is refused (the numeric id drives the ROS
        namespace, MAVLink port and UE num, so it is globally unique).
        Called from both the boot-time YAML loop and the UDP listener
        thread, so the registry mutation is guarded by a lock. roster:
        complete vehicle_models list for network_sim, known upfront for
        boot-time fleets. Props skip the backend entirely: they are a bare
        gz entity created (and later removed) by the manager itself.
        """
        with self._spawn_lock:
            holder = self.registry.type_of(vehicle_id)
            if holder == vehicle_type:
                self.get_logger().warn(
                    f"skip duplicate {vehicle_type}_{vehicle_id}")
                return
            if holder is not None:
                self.get_logger().warn(
                    f"refusing {vehicle_type}_{vehicle_id}: id {vehicle_id} "
                    f"is already active as {holder}")
                return
            if self._is_prop_type(vehicle_type):
                sdf_path = render_sdf(
                    vehicle_type,
                    self.get_parameter('unreal_ip').value,
                    self.get_parameter('unreal_port').value)
                subprocess.run(
                    build_create_argv(vehicle_type, vehicle_id, sdf_path,
                                      world, position, rpy),
                    check=True, timeout=30)
                record = self.registry.add(vehicle_type, vehicle_id)
                record.prop = True
                self.get_logger().info(
                    f"spawned prop {vehicle_type}_{vehicle_id}")
                return
            spec = VehicleSpec(
                vehicle_id, vehicle_type, build_target_path,
                (position[0], position[1], position[2], rpy[2]))
            handle = self.backend.launch(
                spec, world, position, rpy,
                self.get_parameter('unreal_ip').value,
                self.get_parameter('unreal_port').value,
                roster=roster)
            record = self.registry.add(vehicle_type, vehicle_id)
            record.handle = handle
            self.get_logger().info(
                f"spawned {vehicle_type}_{vehicle_id} "
                f"(handle {getattr(handle, 'pid', handle)})")

    # -- lifecycle watching -------------------------------------------------
    def _check_vehicles(self):
        """Reap vehicles whose backend handle died (crashed PX4/container):
        free the registry slot and remove the stale gz model."""
        world = self.get_parameter('world').value
        with self._spawn_lock:
            # props have no autopilot process/container to die - skip them
            dead = [(t, i) for t, i in self.registry.active_ids()
                    if not self.registry.get(t, i).prop
                    and not self.backend.alive(self.registry.get(t, i).handle)]
            for key in dead:
                self.registry.remove(*key)
        for vehicle_type, vehicle_id in dead:
            self.get_logger().warn(
                f"{vehicle_type}_{vehicle_id} died unexpectedly; "
                f"cleaning up its model and freeing the id")
            subprocess.run(
                build_remove_argv(world, vehicle_type, vehicle_id),
                check=False, timeout=10)

    def cleanup_leftovers(self):
        """Remove vehicle containers surviving a previous (killed) manager.

        The gz world restarts together with the manager, so survivors are
        zombies: their PX4 keeps running but their model no longer exists.
        A clean slate is the only consistent state. No-op for backends
        without discovery (subprocess PIDs are gone with the old manager).
        """
        find = getattr(self.backend, 'find_existing', None)
        if find is None:
            return
        for vehicle_type, vehicle_id, container_id in find():
            self.get_logger().warn(
                f"removing leftover {vehicle_type}_{vehicle_id} container "
                f"from a previous run (its model died with the old world)")
            try:
                self.backend.kill(container_id)
            except Exception as exc:
                self.get_logger().error(f"leftover cleanup failed: {exc}")

    # -- boot-time YAML trigger -------------------------------------------
    def spawn_all(self):
        yaml_path = self.get_parameter('yaml_path').value
        world = self.get_parameter('world').value
        if not yaml_path:
            return
        with open(yaml_path) as f:
            config = yaml.safe_load(f)
        specs = parse_vehicles(config)
        # The full fleet is known upfront, so every vehicle gets the complete
        # network_sim roster (matches what generate_compose.py used to bake).
        # Props are not V2V participants and stay out of it.
        roster = sorted(f'{s.vehicle_type}_{s.vehicle_id}' for s in specs
                        if not self._is_prop_type(s.vehicle_type))
        for spec in specs:
            x, y, z, yaw = spec.spawnpoint
            try:
                self._spawn_one(spec.vehicle_type, spec.vehicle_id, (x, y, z),
                                (0.0, 0.0, yaw), spec.build_target_path, world,
                                roster=roster)
            except Exception as exc:
                # One failed vehicle (e.g. its MAVLink host port is taken)
                # must not kill the whole boot — log and keep going.
                self.get_logger().error(
                    f"boot spawn failed for "
                    f"{spec.vehicle_type}_{spec.vehicle_id}: {exc}")
            time.sleep(SPAWN_STAGGER_SEC)
        self.get_logger().info(
            f"spawn complete: {[f'{t}_{i}' for t, i in self.registry.active_ids()]}")

    # -- runtime UDP trigger ----------------------------------------------
    def start_udp_listener(self):
        self.get_logger().info(f"vehicle codes: {self._code_map}")
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
            # Drain whatever else is already queued. Handling a command can
            # take hundreds of ms (gz service / backend launch), while a UI
            # dragging a prop streams MSG_POSE at frame rate - processing
            # the backlog one by one would replay stale teleports for
            # seconds after the user lets go. Acting on a batch lets
            # superseded poses collapse to the newest one instead.
            batch = [data]
            self._sock.setblocking(False)
            try:
                while len(batch) < 512:
                    try:
                        more, _ = self._sock.recvfrom(2048)
                    except (BlockingIOError, OSError):
                        break
                    batch.append(more)
            finally:
                self._sock.settimeout(0.5)
            cmds = []
            for raw in batch:
                try:
                    cmd = parse_packet(raw)
                except ValueError as exc:
                    self.get_logger().warn(f"bad packet: {exc}")
                    continue
                if cmd is not None:  # None = not a spawn/despawn/pose command
                    cmds.append(cmd)
            for idx, cmd in enumerate(cmds):
                if not isinstance(cmd, DespawnCommand) and any(
                        not isinstance(later, DespawnCommand)
                        and later.vehicle_num == cmd.vehicle_num
                        and later.vehicle_code == cmd.vehicle_code
                        for later in cmds[idx + 1:]):
                    continue  # superseded by a newer pose for the same entity
                self._handle_command(cmd, world, default_px4)

    def _handle_command(self, cmd, world, default_px4):
        """Act on one parsed wire command (runs on the UDP listener thread)."""
        try:
            vehicle_type = type_for_code(cmd.vehicle_code, self._code_map)
        except ValueError as exc:
            self.get_logger().warn(str(exc))
            return
        # id/type validation: the packet's code must match whatever type
        # currently holds the num - a mismatch is a sender bug, and acting
        # on it would collide ports/namespaces or hit the wrong entity, so
        # the packet is dropped loudly.
        holder = self.registry.type_of(cmd.vehicle_num)
        if holder is not None and holder != vehicle_type:
            self.get_logger().warn(
                f"dropped packet: id {cmd.vehicle_num} is active as "
                f"{holder}, but the packet says {vehicle_type}")
            return
        if isinstance(cmd, DespawnCommand):
            self.get_logger().info(
                f"UDP despawn: {vehicle_type}_{cmd.vehicle_num}")
            try:
                self._despawn_one(vehicle_type, cmd.vehicle_num, world)
            except Exception as exc:
                self.get_logger().error(f"despawn failed: {exc}")
            return
        if holder is not None and self._is_prop_type(vehicle_type):
            # upsert: MSG_POSE for an active prop means MOVE (mirrors the
            # gz->UE direction, where a known num is a pose update)
            try:
                self._move_prop(vehicle_type, cmd.vehicle_num,
                                cmd.position, cmd.quaternion, world)
            except Exception as exc:
                self.get_logger().error(f"move failed: {exc}")
            return
        rpy = quat_to_euler(*cmd.quaternion)
        self.get_logger().info(
            f"UDP spawn: {vehicle_type}_{cmd.vehicle_num} at {cmd.position}")
        try:
            self._spawn_one(vehicle_type, cmd.vehicle_num, cmd.position,
                            rpy, default_px4, world)
        except Exception as exc:
            self.get_logger().error(f"spawn failed: {exc}")

    def _move_prop(self, vehicle_type, vehicle_id, position, quaternion,
                   world):
        """Teleport an active prop.

        No explicit rate limit: the batch drain in _udp_loop collapses a
        pose stream to one move per prop per cycle, so the (blocking) gz
        service call itself paces this naturally.
        """
        proc = subprocess.run(
            build_set_pose_argv(world, vehicle_type, vehicle_id,
                                position, quaternion),
            capture_output=True, timeout=5)
        if proc.returncode != 0:
            self.get_logger().warn(
                f"move failed for {vehicle_type}_{vehicle_id}: "
                f"{proc.stderr.decode(errors='replace').strip()}")

    def _despawn_one(self, vehicle_type, vehicle_id, world):
        """Tear down the vehicle via the backend and remove its gz model."""
        with self._spawn_lock:
            if not self.registry.is_active(vehicle_type, vehicle_id):
                self.get_logger().warn(
                    f"despawn: {vehicle_type}_{vehicle_id} not active")
                return
            record = self.registry.remove(vehicle_type, vehicle_id)
        if not record.prop:
            self.backend.kill(record.handle)
        # Model removal is manager-side in every mode: killing the autopilot
        # stack (process or container) never removes the gz entity.
        subprocess.run(
            build_remove_argv(world, vehicle_type, vehicle_id),
            check=False, timeout=10)
        self.get_logger().info(f"despawned {vehicle_type}_{vehicle_id}")

    def shutdown(self):
        self._stop = True
        if self._sock is not None:
            self._sock.close()
        # Tear down everything we spawned. Vehicles must not outlive the
        # manager: PX4 runs in its own process group (or container), so a
        # plain Ctrl+C would otherwise leave orphans behind.
        world = self.get_parameter('world').value
        for vehicle_type, vehicle_id in self.registry.active_ids():
            try:
                self._despawn_one(vehicle_type, vehicle_id, world)
            except Exception as exc:
                self.get_logger().warn(
                    f"shutdown teardown failed for "
                    f"{vehicle_type}_{vehicle_id}: {exc}")


def main(args=None):
    rclpy.init(args=args)

    def _sigterm_to_interrupt(*_):
        # `docker stop` / ros2 launch shutdown delivers SIGTERM; python's
        # default disposition would kill us without running the finally
        # teardown below, leaving vehicle containers behind
        raise KeyboardInterrupt()

    signal.signal(signal.SIGTERM, _sigterm_to_interrupt)
    node = ManagerNode()
    try:
        node.wait_for_clock()
        node.cleanup_leftovers()
        node.spawn_all()
        node.start_udp_listener()
        rclpy.spin(node)  # keep node alive: hold vehicle handles + serve UDP
    except (KeyboardInterrupt, ExternalShutdownException, TimeoutError) as exc:
        # SIGINT/SIGTERM land here; the finally block runs the teardown
        node.get_logger().error(str(exc) or type(exc).__name__)
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
