import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    ExecuteProcess, SetEnvironmentVariable)
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    realgazebo_share = get_package_share_directory('realgazebo')
    gazebo_launch = os.path.join(realgazebo_share, 'launch', 'gazebo.launch.py')

    args = [
        DeclareLaunchArgument(
            'yaml_path',
            default_value=os.path.join(realgazebo_share, 'yaml', 'example.yaml')),
        DeclareLaunchArgument('world', default_value='c-track'),
        DeclareLaunchArgument('unreal_ip', default_value='127.0.0.1'),
        DeclareLaunchArgument('unreal_port', default_value='5005'),
        DeclareLaunchArgument(
            'px4_path', default_value='/home/user/realgazebo/RealGazebo-PX4'),
        DeclareLaunchArgument('headless', default_value='true'),
    ]

    # Pin GZ transport to localhost for a local monolithic run (matches legacy)
    gz_ip = SetEnvironmentVariable('GZ_IP', '127.0.0.1')

    # gazebo.launch.py brings up the Gazebo server, /clock bridge, and the
    # server-side GZ resource/plugin paths. Vehicle SDFs are rendered by the
    # manager itself, so gazebo.launch.py's limited type list does not matter.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            'headless': LaunchConfiguration('headless'),
            'world': LaunchConfiguration('world'),
            'px4_path': LaunchConfiguration('px4_path'),
            'unreal_ip': LaunchConfiguration('unreal_ip'),
            'unreal_port': LaunchConfiguration('unreal_port'),
        }.items())

    # One shared micro-XRCE-DDS agent for the monolithic run
    xrce_agent = ExecuteProcess(
        cmd=[FindExecutable(name='MicroXRCEAgent'), 'udp4', '-p', '8888'])

    manager = Node(
        package='realgazebo_manager',
        executable='manager_node',
        name='realgazebo_manager',
        output='screen',
        parameters=[{
            'yaml_path': LaunchConfiguration('yaml_path'),
            'world': LaunchConfiguration('world'),
            'unreal_ip': LaunchConfiguration('unreal_ip'),
            'unreal_port': ParameterValue(
                LaunchConfiguration('unreal_port'), value_type=int),
        }])

    return LaunchDescription(args + [gz_ip, gazebo, xrce_agent, manager])
