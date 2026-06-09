import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from px4_msgs.msg import (
    LogMessage,
    VehicleStatus,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommandAck,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleGlobalPosition,
    VehicleAttitude,
    VehicleAngularVelocity,
    ActuatorMotors,
)

from std_msgs.msg import String, Float32MultiArray, Float64MultiArray

from enum import Enum

import math
import numpy as np
import h5py


def quat2angle(quat):
    """Output: yaw, pitch, roll"""
    qin = (
        quat / np.sqrt(quat[0]**2 + quat[1]**2 + quat[2]**2 + quat[3]**2)
    ).squeeze()
    r11 = 2 * (qin[1] * qin[2] + qin[0] * qin[3])
    r12 = qin[0]**2 + qin[1]**2 - qin[2]**2 - qin[3]**2
    r21 = -2 * (qin[1] * qin[3] - qin[0] * qin[2])
    r31 = 2 * (qin[2] * qin[3] + qin[0] * qin[1])
    r32 = qin[0]**2 - qin[1]**2 - qin[2]**2 + qin[3]**2
    return np.arctan2(r11, r12), np.arcsin(r21), np.arctan2(r31, r32)


class Vehicle(Enum):
    IRIS = 0
    ROVER = 1
    BOAT = 2
    UNKNOWN = 99


# ctrlparams.h5 경로
CTRL_PARAM_PATH = "/home/user/realgazebo/ctrlparams.h5"


WAYPOINTS = {
    1: [
        (-189.96, -53.11, -160.0),
    ],
    2: [
        (-90.6,   35.33,  -170.0),
        (-167.75, -13.78, -170.0),
    ],
}


class PX4ROS2(Node):
    def __init__(self):
        super().__init__("px4_ros2")

        self.declare_parameter('system_id', 1)
        self.system_id_ = self.get_parameter('system_id').get_parameter_value().integer_value

        self.declare_parameter('vehicle_type', "iris")
        try:
            self.vehicle_type_ = Vehicle[
                self.get_parameter('vehicle_type').get_parameter_value().string_value.upper()
            ]
        except KeyError:
            self.vehicle_type_ = Vehicle.UNKNOWN

        self.get_logger().info(f"Configure px4-ros2 {self.system_id_}")

        self.topic_prefix_manager_ = f"vehicle{self.system_id_}/manager/"
        self.topic_prefix_fmu_ = f"vehicle{self.system_id_}/fmu/"


        self.vehicle_status_msg_ = VehicleStatus()
        self.vehicle_local_position_msg_ = VehicleLocalPosition()
        self.vehicle_global_position_msg_ = VehicleGlobalPosition()

 
        self.init_local_pos = None
        self.init_ang = None

        self.local_pos = [0.0, 0.0, 0.0]
        self.vel = [0.0, 0.0, 0.0]
        self.ang = [0.0, 0.0, 0.0]      # roll, pitch, yaw
        self.omega = [0.0, 0.0, 0.0]

        self.is_landing = False
        self.exit_value = -1

        self.use_direct_actuator_ = False

        # 0번 gain = 정상 상태 gain
        self.motor_failure_num = 0
        self.scaling_factor = 100

  
        self.ocm_msg_qhac_ = OffboardControlMode()
        self.set_position_offboard_mode()

        self.ocm_publisher_ = self.create_publisher(
            OffboardControlMode,
            f'{self.topic_prefix_fmu_}in/offboard_control_mode',
            qos_profile_sensor_data
        )

        self.traj_setpoint_publisher_ = self.create_publisher(
            TrajectorySetpoint,
            f'{self.topic_prefix_fmu_}in/trajectory_setpoint',
            10
        )

        self.vehicle_command_publisher_ = self.create_publisher(
            VehicleCommand,
            f'{self.topic_prefix_fmu_}in/vehicle_command',
            10
        )

        # direct actuator 출력
        self.motor_control_pub_ = self.create_publisher(
            ActuatorMotors,
            f'{self.topic_prefix_fmu_}in/actuator_motors',
            10
        )


        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus,
            f'{self.topic_prefix_fmu_}out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile_sensor_data
        )

        self.vehicle_local_position_subscriber_ = self.create_subscription(
            VehicleLocalPosition,
            f'{self.topic_prefix_fmu_}out/vehicle_local_position',
            self.vehicle_local_position_callback,
            qos_profile_sensor_data
        )

        self.vehicle_global_position_subscriber = self.create_subscription(
            VehicleGlobalPosition,
            f'{self.topic_prefix_fmu_}out/vehicle_global_position',
            self.vehicle_global_position_callback,
            qos_profile_sensor_data
        )

        self.vehicle_att_sub = self.create_subscription(
            VehicleAttitude,
            f'{self.topic_prefix_fmu_}out/vehicle_attitude',
            self.vehicle_att_listener_callback,
            qos_profile_sensor_data
        )

        self.vehicle_ang_vel_sub = self.create_subscription(
            VehicleAngularVelocity,
            f'{self.topic_prefix_fmu_}out/vehicle_angular_velocity',
            self.vehicle_ang_vel_listener_callback,
            qos_profile_sensor_data
        )

        self.main_cmd_subscriber_ = self.create_subscription(
            String,
            f'{self.topic_prefix_manager_}in/main_cmd',
            self.main_cmd_callback,
            10
        )

        self.waypoint_list_ = WAYPOINTS.get(self.system_id_, [])
        self.waypoint_index_ = 0

        self.target_x_ = None
        self.target_y_ = None
        self.target_z_ = None


        self.g = 9.81
        self.m = 41.97

        self.J = np.array([
            [10.5222, 0., -0.4004],
            [0., 11.8625, 0.],
            [-0.4004, 0., 21.91995]
        ])
        self.Jinv = np.linalg.inv(self.J)

        dx1, dx2, dx3 = 0.9815, 0.0235, 1.1235
        dy1 = dy2 = 0.717
        c, self.c_th = 0.0338, 130

        self.B = np.array([
            [-1, -1, -1, -1, -1, -1],
            [-dy2, dy1, dy1, -dy2, -dy2, dy1],
            [-dx2, -dx2, dx1, -dx3, dx1, -dx3],
            [-c, c, -c, c, c, -c]
        ])
        self.Binv = np.linalg.pinv(self.B)

        with h5py.File(CTRL_PARAM_PATH, "r") as f:
            self.K = f["Ks"][:]
            self.u_trim = f["u_trims"][:]

        self.get_logger().info(f"LQR parameter loaded: {CTRL_PARAM_PATH}")

  
        timer_period_ocm = 0.1
        self.timer_ocm_ = self.create_timer(timer_period_ocm, self.timer_ocm_callback)


    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position_msg_ = msg

        self.local_pos = [msg.x, msg.y, msg.z]
        self.vel = [msg.vx, msg.vy, msg.vz]

        if self.init_local_pos is None:
            self.init_local_pos = self.local_pos.copy()

    def vehicle_global_position_callback(self, msg):
        self.vehicle_global_position_msg_ = msg

    def vehicle_status_callback(self, msg):
        self.vehicle_status_msg_ = msg

    def vehicle_att_listener_callback(self, msg):
        yaw, pitch, roll = quat2angle(msg.q)
        self.ang = [roll, pitch, yaw]

        if self.init_ang is None:
            self.init_ang = self.ang.copy()

    def vehicle_ang_vel_listener_callback(self, msg):
        self.omega = [msg.xyz[0], msg.xyz[1], msg.xyz[2]]


    def main_cmd_callback(self, msg):
        match msg.data:
            case "ARM":
                self.control_arm()

            case "DISARM":
                self.control_disarm()

            case "TAKEOFF":
                self.control_takeoff(3)

            case "OFFBOARD":
                self.use_direct_actuator_ = True
                self.motor_failure_num = 0
                self.scaling_factor = 100

                self.target_x_ = self.vehicle_local_position_msg_.x
                self.target_y_ = self.vehicle_local_position_msg_.y
                self.target_z_ = self.vehicle_local_position_msg_.z

                self.control_offboard()

                self.get_logger().info(
                    f"[system_id {self.system_id_}] OFFBOARD direct actuator test start. "
                    f"target=({self.target_x_:.2f}, {self.target_y_:.2f}, {self.target_z_:.2f}), "
                    f"motor_failure_num={self.motor_failure_num}, scaling_factor={self.scaling_factor}"
                )

            case "START":
                self.use_direct_actuator_ = False
                self.motor_failure_num = 0
                self.scaling_factor = 100

                self.waypoint_index_ = 0
                self._go_to_current_waypoint()

                self.get_logger().info(
                    f"[system_id {self.system_id_}] START position waypoint mode"
                )

            case _:
                self.get_logger().warn(f"Unknown command: {msg.data}")


    def _go_to_current_waypoint(self):
        if self.waypoint_index_ >= len(self.waypoint_list_):
            self.get_logger().info(f"[system_id {self.system_id_}] ")
            return

        x, y, z = self.waypoint_list_[self.waypoint_index_]

        self.target_x_ = x
        self.target_y_ = y
        self.target_z_ = z

        self.get_logger().info(
            f"[system_id {self.system_id_}] 웨이포인트 {self.waypoint_index_} 이동: ({x}, {y}, {z})"
        )

        if not self.use_direct_actuator_:
            self.control_setpoint(
                x,
                y,
                z,
                self.vehicle_local_position_msg_.heading
            )


    def set_position_offboard_mode(self):
        self.ocm_msg_qhac_.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        self.ocm_msg_qhac_.position = True
        self.ocm_msg_qhac_.velocity = False
        self.ocm_msg_qhac_.acceleration = False
        self.ocm_msg_qhac_.attitude = False
        self.ocm_msg_qhac_.body_rate = False
        self.ocm_msg_qhac_.direct_actuator = False

    def set_direct_actuator_offboard_mode(self):
        self.ocm_msg_qhac_.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        self.ocm_msg_qhac_.position = False
        self.ocm_msg_qhac_.velocity = False
        self.ocm_msg_qhac_.acceleration = False
        self.ocm_msg_qhac_.attitude = False
        self.ocm_msg_qhac_.body_rate = False
        self.ocm_msg_qhac_.direct_actuator = True

    def timer_ocm_callback(self):
        if self.use_direct_actuator_:
            self.set_direct_actuator_offboard_mode()
        else:
            self.set_position_offboard_mode()

        self.ocm_publisher_.publish(self.ocm_msg_qhac_)

        if self.target_x_ is None:
            return

        # direct actuator 모드에서는 LQR이 actuator_motors 직접 publish
        if self.use_direct_actuator_:
            self.LQRcontroller()
            return

        # 기존 position offboard 모드
        self.control_setpoint(
            self.target_x_,
            self.target_y_,
            self.target_z_,
            self.vehicle_local_position_msg_.heading
        )

        # position waypoint 테스트용 도달 판정
        dx = self.vehicle_local_position_msg_.x - self.target_x_
        dy = self.vehicle_local_position_msg_.y - self.target_y_
        dz = self.vehicle_local_position_msg_.z - self.target_z_
        distance = math.sqrt(dx**2 + dy**2 + dz**2)

        if distance < 1.5:
            if self.waypoint_index_ < len(self.waypoint_list_) - 1:
                self.waypoint_index_ += 1
                self._go_to_current_waypoint()

    def get_ref_pos(self):
        if self.target_x_ is None:
            if self.init_local_pos is not None:
                return self.init_local_pos
            return [0.0, 0.0, -3.0]

        return [self.target_x_, self.target_y_, self.target_z_]

    def _publish_motor_control(self, ctrl):
        """ctrl: numpy array [6개 모터값, 0-scaling_factor 범위]"""

        ctrl = np.asarray(ctrl).reshape(-1)

        if np.any(np.isnan(ctrl)) or np.any(np.isinf(ctrl)):
            self.get_logger().warn("[LQR] ctrl contains NaN/Inf. Motor command skipped.")
            return

        ctrl = np.clip(ctrl[:6], 0.0, self.scaling_factor)

        ctrl_norm = ctrl / self.scaling_factor
        ctrl_norm = np.clip(ctrl_norm, 0.0, 1.0)

        actuator_msg = ActuatorMotors()
        actuator_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        actuator_msg.timestamp_sample = actuator_msg.timestamp

        control_list = [float('nan')] * 12
        for i in range(6):
            control_list[i] = float(ctrl_norm[i])

        actuator_msg.control = control_list

        self.motor_control_pub_.publish(actuator_msg)

    def LQRcontroller(self):
        fault_idx = self.motor_failure_num

        posd = self.get_ref_pos()

        ex = self.local_pos[0] - posd[0]
        ey = self.local_pos[1] - posd[1]
        ez = self.local_pos[2] - posd[2]

        x = np.vstack((ex, ey, ez, *self.vel, *self.ang, *self.omega))
        xd = np.zeros((12, 1))

        th = -self.K[:, :, fault_idx] @ (x - xd)

        _th = th.ravel()
        _u_trim = self.u_trim[:6, :, fault_idx].ravel()

        ctrl = (_th + _u_trim) * self.scaling_factor
        ctrl = np.clip(ctrl, 0, self.scaling_factor)

        # ctrl[0] = -ctrl[0]
        # ctrl[2] = -ctrl[2]
        # ctrl[5] = -ctrl[5]

        self._publish_motor_control(ctrl)

    # =========================================================
    # PX4 commands
    # =========================================================
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

    def disarm(self):
        self.control_disarm()

    def control_offboard(self):
        offboard_cmd = VehicleCommand()
        offboard_cmd.target_system = self.system_id_
        offboard_cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        offboard_cmd.param1 = 1.0
        offboard_cmd.param2 = 6.0
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

    px4ros2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
