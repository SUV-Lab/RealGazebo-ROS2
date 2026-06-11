import os
import time
import subprocess

import yaml
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

from .yaml_config import parse_vehicles
from .spawn_core import (
    render_sdf, build_create_argv, build_px4_command, build_param_argv)
from .registry import VehicleRegistry

# PX4 params applied right after spawn (same as legacy realgazebo.launch.py)
POST_SPAWN_PARAMS = [
    ('NAV_DLL_ACT', 0),
    ('COM_RCL_EXCEPT', 31),
    ('COM_RC_IN_MODE', 4),
]
SPAWN_STAGGER_SEC = 0.5


class ManagerNode(Node):
    def __init__(self):
        super().__init__('realgazebo_manager')
        self.declare_parameter('yaml_path', '')
        self.declare_parameter('world', 'c-track')
        self.declare_parameter('unreal_ip', '127.0.0.1')
        self.declare_parameter('unreal_port', 5005)
        self.registry = VehicleRegistry()
        self._clock_seen = False
        self.create_subscription(Clock, '/clock', self._on_clock, 10)

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

    def spawn_all(self):
        yaml_path = self.get_parameter('yaml_path').value
        world = self.get_parameter('world').value
        unreal_ip = self.get_parameter('unreal_ip').value
        unreal_port = self.get_parameter('unreal_port').value

        with open(yaml_path) as f:
            config = yaml.safe_load(f)
        specs = parse_vehicles(config)

        for spec in specs:
            if self.registry.is_active(spec.vehicle_type, spec.vehicle_id):
                self.get_logger().warn(
                    f"skip duplicate {spec.vehicle_type}_{spec.vehicle_id}")
                continue
            sdf_path = render_sdf(spec.vehicle_type, unreal_ip, unreal_port)
            subprocess.run(
                build_create_argv(spec.vehicle_type, spec.vehicle_id,
                                  sdf_path, world, spec.spawnpoint),
                check=True)
            argv, px4_env, px4_cwd = build_px4_command(spec, world)
            proc = subprocess.Popen(
                argv, env={**os.environ, **px4_env}, cwd=px4_cwd)
            record = self.registry.add(spec.vehicle_type, spec.vehicle_id)
            record.process = proc
            self.get_logger().info(
                f"spawned {spec.vehicle_type}_{spec.vehicle_id} (px4 pid {proc.pid})")
            time.sleep(SPAWN_STAGGER_SEC)

        # Apply PX4 params once all vehicles are up
        time.sleep(2.0)
        for spec in specs:
            for name, value in POST_SPAWN_PARAMS:
                subprocess.run(build_param_argv(spec, name, value), check=False)
        self.get_logger().info(
            f"spawn complete: {[f'{t}_{i}' for t, i in self.registry.active_ids()]}")


def main(args=None):
    rclpy.init(args=args)
    node = ManagerNode()
    try:
        node.wait_for_clock()
        node.spawn_all()
        rclpy.spin(node)  # keep node alive (holds subprocesses; v2 UDP hook point)
    except (KeyboardInterrupt, TimeoutError) as exc:
        node.get_logger().error(str(exc))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
