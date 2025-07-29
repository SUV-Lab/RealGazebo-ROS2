import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from px4_msgs.msg import LogMessage, VehicleStatus, OffboardControlMode, TrajectorySetpoint, VehicleCommandAck, \
    VehicleCommand, VehicleLocalPosition

from std_msgs.msg import String
import sys, select, termios, tty

msg = """
a : arm
o : offboard
t : takeoff(only for drone)
s : start misssion
d : disarm
"""

settings = termios.tcgetattr(sys.stdin)
keyBindings = {
    'a': "ARM",
    'o': "OFFBOARD",
    't': "TAKEOFF",
    "s": "START",
    "d": "DISARM"
}

def getKey():
    # tty.setraw(sys.stdin.fileno())
    tty.setcbreak(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init()
    node = rclpy.create_node('px4_ros2_controller')
    node.declare_parameter('vehicles', 0)
    node.declare_parameter('cmd_vel_needed', False)
    vehicles = node.get_parameter('vehicles').get_parameter_value().integer_value
    cmd_vel_needed = node.get_parameter('cmd_vel_needed').get_parameter_value().bool_value
    main_cmd_publisher_list = []
    if cmd_vel_needed:
        cmd_vel_publisher = node.create_publisher(Twist, "cmd_vel", 10)
    for i in range(1, vehicles + 1):
        topic_prefix_manager_ = f"vehicle{i}/manager/"
        main_cmd_publisher_list.append(node.create_publisher(String,
                                                            f'{topic_prefix_manager_}in/main_cmd',
                                                            10))
    try:
        print(msg)
        print(f"{vehicles} vehicles in use")
        print(f"publish /cmd_vel: {cmd_vel_needed}")
        while(1):
            key = getKey()
            if key in keyBindings.keys():
                if cmd_vel_needed and key == 's':
                    cmd_vel_msg = Twist()
                    cmd_vel_msg.linear.x = 1.0
                    cmd_vel_publisher.publish(cmd_vel_msg)
                cmd_msg = String()
                cmd_msg.data = keyBindings[key]
                for i in main_cmd_publisher_list:
                    i.publish(cmd_msg)
            else:
                if (key == '\x03'):
                    break
    except Exception as e:
        print(e)
