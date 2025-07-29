import time
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from px4_msgs.msg import LogMessage, VehicleStatus, OffboardControlMode, TrajectorySetpoint, VehicleCommandAck, \
    VehicleCommand, VehicleLocalPosition, VehicleGlobalPosition

from std_msgs.msg import String

from enum import Enum

import math

class nav_state(Enum):
    NAVIGATION_STATE_MANUAL = 0               # Manual mode
    NAVIGATION_STATE_ALTCTL = 1               # Altitude control mode
    NAVIGATION_STATE_POSCTL = 2               # Position control mode
    NAVIGATION_STATE_AUTO_MISSION = 3         # Auto mission mode
    NAVIGATION_STATE_AUTO_LOITER = 4          # Auto loiter mode
    NAVIGATION_STATE_AUTO_RTL = 5             # Auto return to launch mode
    NAVIGATION_STATE_UNUSED3 = 8              # Free slot
    NAVIGATION_STATE_UNUSED = 9               # Free slot
    NAVIGATION_STATE_ACRO = 10                # Acro mode
    NAVIGATION_STATE_UNUSED1 = 11             # Free slot
    NAVIGATION_STATE_DESCEND = 12             # Descend mode (no position control)
    NAVIGATION_STATE_TERMINATION = 13         # Termination mode
    NAVIGATION_STATE_OFFBOARD = 14
    NAVIGATION_STATE_STAB = 15                # Stabilized mode
    NAVIGATION_STATE_UNUSED2 = 16             # Free slot
    NAVIGATION_STATE_AUTO_TAKEOFF = 17        # Takeoff
    NAVIGATION_STATE_AUTO_LAND = 18           # Land
    NAVIGATION_STATE_AUTO_FOLLOW_TARGET = 19  # Auto Follow
    NAVIGATION_STATE_AUTO_PRECLAND = 20       # Precision land with landing target
    NAVIGATION_STATE_ORBIT = 21               # Orbit in a circle
    NAVIGATION_STATE_AUTO_VTOL_TAKEOFF = 22   # Takeoff, transition, establish loiter
    NAVIGATION_STATE_MAX = 23


class DroneController(Node):
    def __init__(self):
        super().__init__("drone_controller")
        self.initialize_node()

    def timer_display_info_callback(self):
        os.system('clear')
        self.get_logger().info(f"Mode : {nav_state(self.vehicle_status_msg_.nav_state).name} ({'ARM' if self.vehicle_status_msg_.arming_state == VehicleStatus.ARMING_STATE_ARMED else 'DISARM'})")
        self.get_logger().info(f"Vehicle local_position(ned) : ({self.vehicle_local_position_msg_.x:.2f}, {self.vehicle_local_position_msg_.y:.2f}, {self.vehicle_local_position_msg_.z:.2f})")
        self.get_logger().info(f"last command : {self.last_command}")

        self.info_count += 1

        match self.info_count:
            case 50:
                self.control_arm()
            case 55:
                self.control_takeoff(5)
            case 145:
                self.control_offboard()
            case 155:
                self.setpoint[0] = self.vehicle_local_position_msg_.x - 50
                self.setpoint[1] = self.vehicle_local_position_msg_.y
                self.setpoint[2] = self.vehicle_local_position_msg_.z
                self.control_setpoint(self.vehicle_local_position_msg_.x - 50, self.vehicle_local_position_msg_.y, self.vehicle_local_position_msg_.z, self.vehicle_local_position_msg_.heading)

        if self.info_count > 155 and math.sqrt((self.setpoint[0] - self.vehicle_local_position_msg_.x)**2 + (self.setpoint[1] - self.vehicle_local_position_msg_.y)**2 + (self.setpoint[2] - self.vehicle_local_position_msg_.z)**2) < 1:            
            self.control_land()

    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position_msg_ = msg

    def vehicle_global_position_callback(self, msg):
        self.vehicle_global_position_msg_ = msg

    def vehicle_status_callback(self, msg):
        self.vehicle_status_msg_ = msg

    def timer_ocm_callback(self):
        self.ocm_publisher_.publish(self.ocm_msg_qhac_)
    
    def control_arm(self):
        self.last_command = "arm"
        arm_cmd = VehicleCommand()
        arm_cmd.target_system = self.system_id_
        arm_cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        arm_cmd.param1 = 1.0
        arm_cmd.confirmation = True
        arm_cmd.from_external = True
        self.vehicle_command_publisher_.publish(arm_cmd)

    def control_takeoff(self, altitude):
        self.last_command = "takeoff"
        takeoff_cmd = VehicleCommand()
        takeoff_cmd.target_system = self.system_id_
        takeoff_cmd.command = VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF
        takeoff_cmd.param1 = -1.0
        takeoff_cmd.param2 = 0.0
        takeoff_cmd.param3 = 0.0
        takeoff_cmd.param4 = self.vehicle_local_position_msg_.heading
        takeoff_cmd.param5 = self.vehicle_global_position_msg_.lat
        takeoff_cmd.param6 = self.vehicle_global_position_msg_.lon
        takeoff_cmd.param7 = self.vehicle_global_position_msg_.alt + altitude
        self.vehicle_command_publisher_.publish(takeoff_cmd)

    def control_disarm(self):
        self.last_command = "disarm"
        disarm_cmd = VehicleCommand()
        disarm_cmd.target_system = self.system_id_
        disarm_cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        disarm_cmd.param1 = 0.0
        disarm_cmd.confirmation = True
        self.vehicle_command_publisher_.publish(disarm_cmd)

    def control_offboard(self):
        self.last_command = "offboard"
        offboard_cmd = VehicleCommand()
        offboard_cmd.target_system = self.system_id_
        offboard_cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        offboard_cmd.param1 = 1.0
        offboard_cmd.param2 = 6.0  #PX4_CUSTOM_MAIN_MODE_OFFBOARD
        offboard_cmd.from_external = True
        self.vehicle_command_publisher_.publish(offboard_cmd)

    def control_setpoint(self, x, y, z, heading=None):
        self.last_command = "move"
        setpoint_cmd = TrajectorySetpoint()
        setpoint_cmd.position[0] = x
        setpoint_cmd.position[1] = y
        setpoint_cmd.position[2] = z
        if heading is not None:
            setpoint_cmd.yaw = heading
        self.traj_setpoint_publisher_.publish(setpoint_cmd)

    def control_land(self):
        self.last_command = "land"
        landing_cmd = VehicleCommand()
        landing_cmd.target_system = self.system_id_
        landing_cmd.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        landing_cmd.from_external = True
        self.vehicle_command_publisher_.publish(landing_cmd)
    
    def initialize_node(self):
        self.declare_parameter('system_id', 1)
        self.system_id_ = self.get_parameter('system_id').get_parameter_value().integer_value
        self.last_command = "idle"

        self.get_logger().info(
            f"Configure drone_controller {self.system_id_}")
        self.topic_prefix_manager_ = f"vehicle{self.get_parameter('system_id').get_parameter_value().integer_value}/manager/"
        self.topic_prefix_fmu_ = f"vehicle{self.get_parameter('system_id').get_parameter_value().integer_value}/fmu/"
        self.vehicle_status_subscriber = self.create_subscription(VehicleStatus,
                                                                  f'{self.topic_prefix_fmu_}out/vehicle_status',
                                                                  self.vehicle_status_callback,
                                                                  qos_profile_sensor_data)
        self.vehicle_status_msg_ = VehicleStatus()

        self.ocm_msg_qhac_ = OffboardControlMode()
        self.ocm_msg_qhac_.position = True
        self.ocm_msg_qhac_.velocity = False
        self.ocm_msg_qhac_.acceleration = False
        self.ocm_msg_qhac_.attitude = False
        self.ocm_msg_qhac_.body_rate = False
        self.ocm_msg_qhac_.actuator = False
        self.ocm_publisher_ = self.create_publisher(OffboardControlMode,
                                                    f'{self.topic_prefix_fmu_}in/offboard_control_mode',
                                                    qos_profile_sensor_data)

        self.traj_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint,
                                                              f'{self.topic_prefix_fmu_}in/trajectory_setpoint',
                                                              qos_profile_sensor_data)

        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand,
                                                                f'{self.topic_prefix_fmu_}in/vehicle_command',
                                                                qos_profile_sensor_data)

        self.vehicle_local_position_subscriber_ = self.create_subscription(VehicleLocalPosition,
                                                                           f'{self.topic_prefix_fmu_}out/vehicle_local_position',
                                                                           self.vehicle_local_position_callback,
                                                                           qos_profile_sensor_data)
        self.vehicle_local_position_msg_ = VehicleLocalPosition()

        self.vehicle_global_position_subscriber = self.create_subscription(VehicleGlobalPosition,
                                                                           f'{self.topic_prefix_fmu_}out/vehicle_global_position',
                                                                           self.vehicle_global_position_callback,
                                                                           qos_profile_sensor_data)
        self.vehicle_global_position_msg_ = VehicleGlobalPosition()

        timer_period_ocm = 0.1
        self.timer_ocm_ = self.create_timer(timer_period_ocm, self.timer_ocm_callback)
        
        self.display_info = self.create_timer(timer_period_ocm, self.timer_display_info_callback)
        self.info_count = 0
        self.setpoint = [0, 0, 0]



def main(args=None):
    rclpy.init(args=args)

    drone_controller = DroneController()

    rclpy.spin(drone_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    drone_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
