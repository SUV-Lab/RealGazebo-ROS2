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

support_vehicle = ["iris", "x500", "rover", "boat", "lc_62", "wamv", "ugv_kimm"]
without_px4 = ["ugv_kimm"]

def create_timed_actions(actions_list, initial_delay, interval):
    timed_actions = []
    current_delay = initial_delay
    for action in actions_list:
        timed_action = launch.actions.TimerAction(
            actions=[action],
            period=current_delay
        )
        timed_actions.append(timed_action)
        current_delay += interval
    return timed_actions

def check_px4_build(px4_src_path):
    if not os.path.exists(f'{px4_src_path}/build/px4_sitl_default/build_gazebo-classic/'):
        print(
            f"Cannot find PX4 build file. Check your path or Please run the command 'make px4_sitl_default gazebo_classic' in the path {px4_src_path} to build it first.")
        return False
    return True

def print_usage():
    print("""Usage: ros2 launch realgazebo realgazebo.launch.py server_ip:=[your_server_ip_address] vehicle:=[path to yaml file]""")

def validate_yaml(file_path):
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)

    px4_targets = data.get('px4_target', {})
    for target, path in px4_targets.items():
        if not check_px4_build(path):
            return False

    # Validate vehicles numbering
    vehicles = data.get('vehicles', {})
    vehicle_keys = sorted(vehicles.keys())

    # Check if keys start from 0 and are continuous
    if vehicle_keys != list(range(len(vehicle_keys))):
        print("Error: Vehicle keys are not starting from 0 or are not continuous.")
        return False

    # Validate build_target reference
    px4_targets = data.get('px4_target', {}).keys()
    for key, vehicle in vehicles.items():
        build_target = vehicle.get('build_target')
        if build_target not in px4_targets:
            print(f"Error: Vehicle {key} has an invalid build_target ({build_target}). It is not defined in px4_target.")
            return False

        # Validate spawnpoint format
        try:
            spawnpoint = ast.literal_eval(vehicle.get('spawnpoint'))
        except:
            spawnpoint = None
        if not isinstance(spawnpoint, tuple) or len(spawnpoint) != 4:
            print(f"Error: Vehicle {key} has an invalid spawnpoint. Expected a tuple of 4 values (x, y, z, yaw).")
            return False
    return True

def parse_yaml_to_list(file_path):
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)

    px4_targets = data.get('px4_target', {})

    vehicles_list = []

    vehicles = data.get('vehicles', {})
    for key in sorted(vehicles.keys()):
        vehicle = vehicles[key]

        entity_type = vehicle.get('type')
        build_target_key = vehicle.get('build_target')
        build_target_path = px4_targets.get(build_target_key)
        spawnpoint = ast.literal_eval(vehicle.get('spawnpoint'))

        vehicle_dict = {
            'id': key,
            'type': entity_type,
            'build_target': build_target_path,
            'spawnpoint': spawnpoint
        }

        vehicles_list.append(vehicle_dict)

    return vehicles_list

def launch_setup(context, *args, **kwargs):
    # Configuration
    current_package_path = get_package_share_directory('realgazebo')
    server_ip = LaunchConfiguration('server_ip').perform(context)
    vehicle_str = LaunchConfiguration('vehicle').perform(context)
    if not validate_yaml(vehicle_str):
        exit(1)

    vehicle_lst = parse_yaml_to_list(vehicle_str)
    gazebo_classic_path = f"{vehicle_lst[0]['build_target']}/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
    pairs = vehicle_str.split(',')

    # Environments
    model_path_env = SetEnvironmentVariable('GAZEBO_MODEL_PATH',
                                            f'{current_package_path}/models:{gazebo_classic_path}/models')
    plugin_path_env = SetEnvironmentVariable('GAZEBO_PLUGIN_PATH',
                                             f"{current_package_path}/libs:{vehicle_lst[0]['build_target']}/build/px4_sitl_default/build_gazebo-classic/")

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': f'{gazebo_classic_path}/worlds/c-track.world'}.items()
    )

    uv_process_list = []
    rock_process_list = []

    xrce_agent_process = ExecuteProcess(
        cmd=[FindExecutable(name='MicroXRCEAgent'), 'udp4', '-p', '8888'])

    for vehicle in vehicle_lst:
        vehicle_type = vehicle['type']
        px4_sim_env = SetEnvironmentVariable('PX4_SIM_MODEL', f'gazebo-classic_{vehicle_type}')
        uv_process_list.append(px4_sim_env)
        ## generate sdf file to /tmp/model_X.sdf
        jinja_cmd = [
            f'{gazebo_classic_path}/scripts/jinja_gen.py',
            f'{gazebo_classic_path}/models/{vehicle_type}/{vehicle_type}.sdf.jinja',
            f'{gazebo_classic_path}',
            '--unreal_ip', f"{server_ip}",
            '--unreal_port', f"{5005}",
            '--mavlink_tcp_port', f"{4560 + vehicle['id']}",
            '--mavlink_udp_port', f"{14560 + vehicle['id']}",
            '--mavlink_id', f"{1 + vehicle['id']}",
            '--gst_udp_port', f"{5600 + vehicle['id']}",
            '--video_uri', f"{5600 + vehicle['id']}",
            '--mavlink_cam_udp_port', f"{14530 + vehicle['id']}",
            '--output-file', f"/tmp/{vehicle_type}_{vehicle['id']}.sdf",
        ]

        jinja_process = ExecuteProcess(
            cmd=jinja_cmd)

        uv_process_list.append(jinja_process)

        spawn_entity_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', f"/tmp/{vehicle_type}_{vehicle['id']}.sdf", '-entity', f"{vehicle_type}_{vehicle['id']}",
                       '-x', f"{vehicle['spawnpoint'][0]}",
                       '-y', f"{vehicle['spawnpoint'][1]}",
                       '-z', f"{vehicle['spawnpoint'][2]}",
                       '-Y', f"{vehicle['spawnpoint'][3]}"])

        uv_process_list.append(spawn_entity_node)

        if vehicle_type not in without_px4:
        # PX4
        # build_path/bin/px4 -i $N -d "$build_path/etc" >out.log 2>err.log &
            px4_cmd = [
                f"{vehicle['build_target']}/build/px4_sitl_default/bin/px4",
                '-i', f"{vehicle['id']}",
                '-d',
                f"{vehicle['build_target']}/build/px4_sitl_default/etc",
                '-w', f"{vehicle['build_target']}/build/ROMFS",
                '>out.log', '2>err.log',
            ]
            px4_process = ExecuteProcess(
                cmd=px4_cmd)
            uv_process_list.append(px4_process)


    uv_actions_with_delays = create_timed_actions(
        uv_process_list,
        initial_delay=10.0,
        interval=0.5
    )

    nodes_to_start = [
        model_path_env,
        plugin_path_env,
        xrce_agent_process,
        gazebo_node,
        *uv_actions_with_delays,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'vehicle',
            default_value='/home/user/realgazebo/RealGazebo-ROS2/src/realgazebo/yaml/example.yaml',
            description='path to yaml file  ex)/home/user/realgazebo/RealGazebo-ROS2/src/realgazebo/yaml/example.yaml'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'server_ip',
            default_value='127.0.0.1',
            description='ip of UE5'
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
