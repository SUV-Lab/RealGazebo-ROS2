"""
Gazebo-only launch file for RealGazebo multi-container setup.

This launch file starts only the Gazebo simulator with the world file.
Vehicles are spawned from separate vehicle containers.
"""

import os

from jinja2 import Environment, FileSystemLoader

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)


def launch_setup(context, *args, **kwargs):
    # Configuration
    current_package_path = get_package_share_directory('realgazebo')
    current_package_prefix = get_package_prefix('realgazebo')

    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    verbose = LaunchConfiguration('verbose').perform(context).lower() == 'true'
    world = LaunchConfiguration('world').perform(context)
    px4_path = LaunchConfiguration('px4_path').perform(context)
    unreal_ip = LaunchConfiguration('unreal_ip').perform(context)
    unreal_port = LaunchConfiguration('unreal_port').perform(context)

    gazebo_path = f"{px4_path}/Tools/simulation/gz"

    # Environment variables
    model_path_env = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        f'$GZ_SIM_RESOURCE_PATH:{current_package_path}/models:{gazebo_path}/models:{gazebo_path}/worlds'
    )

    # Plugin paths - include PX4 plugins and RealGazebo plugins
    plugin_paths = [
        "$GZ_SIM_SYSTEM_PLUGIN_PATH",
        f"{px4_path}/build/px4_sitl_default/src/modules/simulation/gz_plugins",
        f"{current_package_prefix}/lib/realgazebo",
    ]
    plugin_path_env = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        ':'.join(plugin_paths)
    )

    server_config_env = SetEnvironmentVariable(
        'GZ_SIM_SERVER_CONFIG_PATH',
        f"{px4_path}/src/modules/simulation/gz_bridge/server.config"
    )

    # Generate world file from Jinja template
    env = Environment(loader=FileSystemLoader(os.path.join(current_package_path, 'models', 'c-track')))
    world_model = env.get_template('model.sdf.jinja')
    output_world = world_model.render(world=world)
    world_model_path = os.path.join(current_package_path, 'models', 'c-track', 'model.sdf')
    with open(world_model_path, 'w') as f:
        f.write(output_world)
        print(f'c-track model.sdf generated')

    # Generate vehicle SDF templates for all supported types
    # This is done in Gazebo container so vehicle containers can spawn them
    model_save_dir = os.path.join('/tmp', 'models')
    os.makedirs(model_save_dir, exist_ok=True)

    support_vehicle = ["x500", "rover_ackermann", "lc_62", "boat"]
    support_obstacle = ["rock"]
    model_list = support_vehicle + support_obstacle

    for model_type in model_list:
        env = Environment(loader=FileSystemLoader(os.path.join(current_package_path, 'models')))
        template_name = f'{model_type}.sdf.jinja' if model_type not in support_obstacle else f'{model_type}/{model_type}.sdf.jinja'
        model = env.get_template(template_name)
        output_model = model.render(unreal_ip=unreal_ip, unreal_port=unreal_port)
        model_file_path = os.path.join(model_save_dir, f'{model_type}.sdf')
        with open(model_file_path, 'w') as f:
            f.write(output_model)
            print(f'{model_type}.sdf generated')

    # Launch Gazebo
    gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    world_file_path = os.path.join(current_package_path, 'worlds', 'c-track.sdf')

    verbose_level = 4 if verbose else 1
    gz_args = f'--verbose={verbose_level} -r -s {world_file_path}' if headless else f'--verbose={verbose_level} -r {world_file_path}'

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gz_sim_pkg, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': gz_args}.items()
    )

    # Clock bridge for ROS2 time synchronization
    gz_timesync_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    )

    nodes_to_start = [
        model_path_env,
        plugin_path_env,
        server_config_env,
        gazebo_node,
        gz_timesync_node,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'headless',
            default_value='true',
            description='Run Gazebo in headless mode (no GUI)',
            choices=['true', 'false']
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'verbose',
            default_value='false',
            description='Run Gazebo with verbose logging (level 4)',
            choices=['true', 'false']
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'world',
            default_value='c-track',
            description='World type',
            choices=['c-track', 'urban', 'vils']
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'px4_path',
            default_value='/home/user/realgazebo/RealGazebo-PX4',
            description='Path to PX4-Autopilot build'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'unreal_ip',
            default_value='127.0.0.1',
            description='IP address of Unreal Engine server'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'unreal_port',
            default_value='5005',
            description='Port of Unreal Engine server'
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
