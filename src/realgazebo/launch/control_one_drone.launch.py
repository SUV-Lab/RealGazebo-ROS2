import os
import random
import yaml
import ast
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
    controller_node = Node(
        package='drone_controller',
        executable='drone_controller',
    )

    nodes_to_start = [ 
        controller_node,
    ]

    return nodes_to_start

def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
