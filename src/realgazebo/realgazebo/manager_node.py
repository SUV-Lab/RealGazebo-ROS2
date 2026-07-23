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
from .backends import make_backend, HitlBackend, PilsBackend
from .entity import Entity
from .registry import EntityRegistry
from .protocol import parse_packet, DespawnCommand
from .type_codes import scan_type_codes, type_for_code
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
        self.registry = EntityRegistry()
        # code->type map scanned from the model templates (single source of
        # truth shared with the gz plugin's <type_code> SDF element)
        self._code_map = scan_type_codes()
        self.backend = make_backend(
            self.get_parameter('backend').value,
            image=self.get_parameter('docker_image').value,
            px4_path=self.get_parameter('default_px4_path').value,
            gazebo_network=self.get_parameter('docker_gazebo_network').value,
            vehicle_network=self.get_parameter('docker_vehicle_network').value,
            mavlink_gcs_ip=self.get_parameter('mavlink_gcs_ip').value,
            # props are not V2V participants, keep them out of the roster
            roster_fn=lambda: [
                r.entity.name for r in self.registry.records()
                if not r.entity.is_prop],
        )
        # HITL vehicles (mode: hitl) route here instead of self.backend. The
        # bridge always runs as a manager-local subprocess (see HitlBackend),
        # so this one instance serves both fleet modes; --qgc reuses the same
        # mavlink_gcs_ip the SITL vehicles beacon to.
        self._hitl_backend = HitlBackend(
            px4_path=self.get_parameter('default_px4_path').value,
            qgc_host=self.get_parameter('mavlink_gcs_ip').value,
        )
        # PILS vehicles (mode: pils): PX4 SITL runs on a remote PC and
        # attaches over gz-transport (see scripts/run_pils_vehicle.sh); the
        # manager owns only the gz entity and its extras, in both fleet modes.
        self._pils_backend = PilsBackend(
            px4_path=self.get_parameter('default_px4_path').value,
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
    def _entity(self, entity_type, entity_id):
        """Validated identity from the scanned code map.

        Raises ValueError for an unknown type — fail-closed, so a typo'd
        type is rejected here instead of falling through to the PX4
        backend path (the old _is_prop_type treated unknowns as vehicles).
        """
        return Entity.create(entity_type, entity_id, self._code_map)

    def _spawn_one(self, entity, position, rpy, build_target_path, world,
                   roster=None, mode='sitl', fc_endpoint=None, sys_id=None):
        """Spawn a single vehicle or prop.

        Idempotent: an entity already active is skipped, and an id held
        by a DIFFERENT type is refused (the numeric id drives the ROS
        namespace, MAVLink port and UE instance, so it is globally unique).
        Called from both the boot-time YAML loop and the UDP listener
        thread, so the registry mutation is guarded by a lock. roster:
        complete vehicle_models list for network_sim, known upfront for
        boot-time fleets. Props skip the backend entirely: they are a bare
        gz entity created (and later removed) by the manager itself.
        """
        with self._spawn_lock:
            holder = self.registry.type_of(entity.id)
            if holder == entity.type:
                self.get_logger().warn(f"skip duplicate {entity.name}")
                return
            if holder is not None:
                self.get_logger().warn(
                    f"refusing {entity.name}: id {entity.id} "
                    f"is already active as {holder}")
                return
            if entity.is_prop:
                sdf_path = render_sdf(
                    entity.type,
                    self.get_parameter('unreal_ip').value,
                    self.get_parameter('unreal_port').value)
                subprocess.run(
                    build_create_argv(entity, sdf_path, world, position, rpy),
                    check=True, timeout=30)
                self.registry.add(entity)
                self.get_logger().info(f"spawned prop {entity.name}")
                return
            spec = VehicleSpec(
                entity.id, entity.type, build_target_path,
                (position[0], position[1], position[2], rpy[2]),
                mode=mode, fc_endpoint=fc_endpoint,
                sys_id=sys_id, entity=entity)
            # HITL vehicles route to the bridge backend, PILS vehicles to the
            # entity-only backend; everything else to the fleet-default one.
            # record.backend remembers the choice so the crash watcher and
            # despawn reap through the same one.
            backend = {'hitl': self._hitl_backend,
                       'pils': self._pils_backend}.get(mode, self.backend)
            handle = backend.launch(
                spec, world, position, rpy,
                self.get_parameter('unreal_ip').value,
                self.get_parameter('unreal_port').value,
                roster=roster)
            record = self.registry.add(entity)
            record.handle = handle
            record.backend = backend
            self.get_logger().info(
                f"spawned {mode} {entity.name} "
                f"(handle {getattr(handle, 'pid', handle)})")

    # -- lifecycle watching -------------------------------------------------
    def _check_vehicles(self):
        """Reap vehicles whose backend handle died (crashed PX4/container):
        free the registry slot and remove the stale gz model."""
        world = self.get_parameter('world').value
        with self._spawn_lock:
            # props have no autopilot process/container to die - skip them
            dead = [r for r in self.registry.records()
                    if not r.entity.is_prop
                    and not (r.backend or self.backend).alive(r.handle)]
            for record in dead:
                self.registry.remove(record.entity.type, record.entity.id)
        for record in dead:
            self.get_logger().warn(
                f"{record.entity.name} died unexpectedly; "
                f"cleaning up its model and freeing the id")
            subprocess.run(
                build_remove_argv(world, record.entity),
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
        # Validate identities upfront: a typo'd type is dropped with an
        # error (and stays out of the roster) instead of killing the boot.
        pending = []
        for spec in parse_vehicles(config):
            try:
                pending.append((self._entity(spec.vehicle_type,
                                             spec.vehicle_id), spec))
            except ValueError as exc:
                self.get_logger().error(
                    f"boot spawn skipped for "
                    f"{spec.vehicle_type}_{spec.vehicle_id}: {exc}")
        # The full fleet is known upfront, so every vehicle gets the complete
        # network_sim roster (matches what generate_compose.py used to bake).
        # Props are not V2V participants and stay out of it.
        roster = sorted(e.name for e, _ in pending if not e.is_prop)
        for entity, spec in pending:
            x, y, z, yaw = spec.spawnpoint
            try:
                self._spawn_one(entity, (x, y, z), (0.0, 0.0, yaw),
                                spec.build_target_path, world, roster=roster,
                                mode=spec.mode, fc_endpoint=spec.fc_endpoint,
                                sys_id=spec.sys_id)
            except Exception as exc:
                # One failed vehicle (e.g. its MAVLink host port is taken)
                # must not kill the whole boot — log and keep going.
                self.get_logger().error(
                    f"boot spawn failed for {entity.name}: {exc}")
            time.sleep(SPAWN_STAGGER_SEC)
        self.get_logger().info(
            f"spawn complete: "
            f"{[r.entity.name for r in self.registry.records()]}")

    # -- runtime UDP trigger ----------------------------------------------
    def start_udp_listener(self):
        self.get_logger().info(f"entity type codes: {self._code_map}")
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
                        and later.entity_id == cmd.entity_id
                        and later.type_code == cmd.type_code
                        for later in cmds[idx + 1:]):
                    continue  # superseded by a newer pose for the same entity
                self._handle_command(cmd, world, default_px4)

    def _handle_command(self, cmd, world, default_px4):
        """Act on one parsed wire command (runs on the UDP listener thread)."""
        try:
            entity = Entity(type_for_code(cmd.type_code, self._code_map),
                            cmd.entity_id, cmd.type_code)
        except ValueError as exc:
            self.get_logger().warn(str(exc))
            return
        # id/type validation: the packet's code must match whatever type
        # currently holds the id - a mismatch is a sender bug, and acting
        # on it would collide ports/namespaces or hit the wrong entity, so
        # the packet is dropped loudly.
        holder = self.registry.type_of(entity.id)
        if holder is not None and holder != entity.type:
            self.get_logger().warn(
                f"dropped packet: id {entity.id} is active as "
                f"{holder}, but the packet says {entity.type}")
            return
        if isinstance(cmd, DespawnCommand):
            self.get_logger().info(f"UDP despawn: {entity.name}")
            try:
                self._despawn_one(entity, world)
            except Exception as exc:
                self.get_logger().error(f"despawn failed: {exc}")
            return
        if holder is not None and entity.is_prop:
            # upsert: MSG_POSE for an active prop means MOVE (mirrors the
            # gz->UE direction, where a known id is a pose update)
            try:
                self._move_prop(entity, cmd.position, cmd.quaternion, world)
            except Exception as exc:
                self.get_logger().error(f"move failed: {exc}")
            return
        rpy = quat_to_euler(*cmd.quaternion)
        self.get_logger().info(
            f"UDP spawn: {entity.name} at {cmd.position}")
        try:
            self._spawn_one(entity, cmd.position, rpy, default_px4, world)
        except Exception as exc:
            self.get_logger().error(f"spawn failed: {exc}")

    def _move_prop(self, entity, position, quaternion, world):
        """Teleport an active prop.

        No explicit rate limit: the batch drain in _udp_loop collapses a
        pose stream to one move per prop per cycle, so the (blocking) gz
        service call itself paces this naturally.
        """
        proc = subprocess.run(
            build_set_pose_argv(world, entity, position, quaternion),
            capture_output=True, timeout=5)
        if proc.returncode != 0:
            self.get_logger().warn(
                f"move failed for {entity.name}: "
                f"{proc.stderr.decode(errors='replace').strip()}")

    def _despawn_one(self, entity, world):
        """Tear down the entity via the backend and remove its gz model."""
        with self._spawn_lock:
            if not self.registry.is_active(entity.type, entity.id):
                self.get_logger().warn(f"despawn: {entity.name} not active")
                return
            record = self.registry.remove(entity.type, entity.id)
        if not record.prop:
            (record.backend or self.backend).kill(record.handle)
        # Model removal is manager-side in every mode: killing the autopilot
        # stack (process or container) never removes the gz entity.
        subprocess.run(
            build_remove_argv(world, entity),
            check=False, timeout=10)
        self.get_logger().info(f"despawned {entity.name}")

    def shutdown(self):
        self._stop = True
        if self._sock is not None:
            self._sock.close()
        # Tear down everything we spawned. Vehicles must not outlive the
        # manager: PX4 runs in its own process group (or container), so a
        # plain Ctrl+C would otherwise leave orphans behind.
        world = self.get_parameter('world').value
        for record in self.registry.records():
            try:
                self._despawn_one(record.entity, world)
            except Exception as exc:
                self.get_logger().warn(
                    f"shutdown teardown failed for "
                    f"{record.entity.name}: {exc}")


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
