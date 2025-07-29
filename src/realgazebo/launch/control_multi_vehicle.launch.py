import os
import random
import yaml
import ast
import rclpy
import time
from collections import defaultdict

from ament_index_python import get_package_prefix
from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessStart, OnProcessExit


def launch_setup(context, *args, **kwargs):
    uv_process_list = []
    rclpy.init()
    node = rclpy.create_node('topic_getter')
    time.sleep(5)
    topics = node.get_topic_names_and_types()
    vehicles = 0
    cmd_vel_needed = False
    if topics:
        for topic_name, topic_type in topics:
            if '/fmu/out/timesync_status' in topic_name:
                vehicles += 1

            if '/cmd_vel' in topic_name:
                cmd_vel_needed = True
                
    node.destroy_node()
    rclpy.shutdown()
    for i in range(vehicles):
        px4_ros2_node = Node(
            package='manager',
            executable='px4_ros2',
            parameters=[{'system_id': i + 1}]
        )
        uv_process_list.append(px4_ros2_node)

    controller_node = Node(
        package='manager',
        executable='controller',
        parameters=[{'vehicles': vehicles, 'cmd_vel_needed': cmd_vel_needed}],
        prefix='xterm -e'
    )

    nodes_to_start = [ 
        *uv_process_list,
        controller_node,
    ]

    return nodes_to_start

def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
