import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from px4_msgs.msg import LogMessage, VehicleStatus, OffboardControlMode, TrajectorySetpoint, VehicleCommandAck, \
    VehicleCommand, VehicleLocalPosition, VehicleGlobalPosition

from std_msgs.msg import String

from enum import Enum

import math


class Vehicle(Enum):
    IRIS = 0
    ROVER = 1
    BOAT = 2
    UNKNOWN = 99


class PX4ROS2(Node):
    def __init__(self):
        super().__init__("px4_ros2")
        self.declare_parameter('system_id', 1)
        self.system_id_ = self.get_parameter('system_id').get_parameter_value().integer_value
        self.declare_parameter('vehicle_type', "iris")
        try:
            self.vehicle_type_ = Vehicle[self.get_parameter('vehicle_type').get_parameter_value().string_value.upper()]
        except KeyError:
            self.vehicle_type_ = Vehicle.UNKNOWN
        self.get_logger().info(
            f"Configure px4-ros2 {self.system_id_}")
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
                                                              10)

        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand,
                                                                f'{self.topic_prefix_fmu_}in/vehicle_command',
                                                                10)

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
        self.main_cmd_subscriber_ = self.create_subscription(String,
                                                             f'{self.topic_prefix_manager_}in/main_cmd',
                                                             self.main_cmd_callback,
                                                             10)
        self.main_cmd_msg_ = String()

        timer_period_ocm = 0.1
        self.timer_ocm_ = self.create_timer(timer_period_ocm, self.timer_ocm_callback)
        self.arrive_target_ = False

    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position_msg_ = msg

    def vehicle_global_position_callback(self, msg):
        self.vehicle_global_position_msg_ = msg

    def main_cmd_callback(self, msg):
        match msg.data:
            case "ARM":
                self.control_arm()
            case "DISARM":
                self.control_disarm()
            case "OFFBOARD":
                self.control_offboard()
            case "TAKEOFF":
                self.control_takeoff(20)
            case "START":
                if self.system_id_ == 4:
                    self.control_setpoint(self.vehicle_local_position_msg_.x, self.vehicle_local_position_msg_.y - 300, self.vehicle_local_position_msg_.z, self.vehicle_local_position_msg_.heading)
                elif self.system_id_ == 3:
                    self.control_setpoint(self.vehicle_local_position_msg_.x, self.vehicle_local_position_msg_.y - 300, self.vehicle_local_position_msg_.z, self.vehicle_local_position_msg_.heading)
                elif self.system_id_ == 5:
                    self.control_setpoint(self.vehicle_local_position_msg_.x, self.vehicle_local_position_msg_.y - 300, self.vehicle_local_position_msg_.z, self.vehicle_local_position_msg_.heading)
                elif self.system_id_ == 1:
                    self.control_setpoint(self.vehicle_local_position_msg_.x, self.vehicle_local_position_msg_.y - 300, self.vehicle_local_position_msg_.z, self.vehicle_local_position_msg_.heading)
                elif self.system_id_ == 2:
                    self.control_setpoint(self.vehicle_local_position_msg_.x, self.vehicle_local_position_msg_.y - 300, self.vehicle_local_position_msg_.z, self.vehicle_local_position_msg_.heading)
                elif self.system_id_ == 6:
                    self.control_setpoint(self.vehicle_local_position_msg_.x, self.vehicle_local_position_msg_.y - 300, self.vehicle_local_position_msg_.z, self.vehicle_local_position_msg_.heading)
                elif self.system_id_ == 7:
                    self.control_setpoint(self.vehicle_local_position_msg_.x, self.vehicle_local_position_msg_.y - 300, self.vehicle_local_position_msg_.z, self.vehicle_local_position_msg_.heading)
                elif self.system_id_ == 9:
                    self.control_setpoint(self.vehicle_local_position_msg_.x - 100, self.vehicle_local_position_msg_.y, self.vehicle_local_position_msg_.z, self.vehicle_local_position_msg_.heading)
                elif self.system_id_ == 10:
                    self.control_setpoint(self.vehicle_local_position_msg_.x - 100, self.vehicle_local_position_msg_.y, self.vehicle_local_position_msg_.z, self.vehicle_local_position_msg_.heading)
                else:
                    self.control_setpoint(self.vehicle_local_position_msg_.x, self.vehicle_local_position_msg_.y - 70, self.vehicle_local_position_msg_.z, self.vehicle_local_position_msg_.heading)


    def vehicle_status_callback(self, msg):
        self.vehicle_status_msg_ = msg

    def timer_ocm_callback(self):
        self.ocm_publisher_.publish(self.ocm_msg_qhac_)

    def control_arm(self):
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
        disarm_cmd = VehicleCommand()
        disarm_cmd.target_system = self.system_id_
        disarm_cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        disarm_cmd.param1 = 0.0
        disarm_cmd.confirmation = True
        self.vehicle_command_publisher_.publish(disarm_cmd)

    def control_offboard(self):
        offboard_cmd = VehicleCommand()
        offboard_cmd.target_system = self.system_id_
        offboard_cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        offboard_cmd.param1 = 1.0
        offboard_cmd.param2 = 6.0  #PX4_CUSTOM_MAIN_MODE_OFFBOARD
        offboard_cmd.from_external = True
        self.vehicle_command_publisher_.publish(offboard_cmd)

    def control_setpoint(self, x, y, z, heading=None):
        setpoint_cmd = TrajectorySetpoint()
        setpoint_cmd.position[0] = x
        setpoint_cmd.position[1] = y
        setpoint_cmd.position[2] = z
        if heading is not None:
            setpoint_cmd.yaw = heading
        self.traj_setpoint_publisher_.publish(setpoint_cmd)



def main(args=None):
    rclpy.init(args=args)

    px4ros2 = PX4ROS2()

    rclpy.spin(px4ros2)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    px4ros2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
