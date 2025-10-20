import os
import random
import yaml
import ast

from jinja2 import Environment, FileSystemLoader

from collections import defaultdict

from ament_index_python import get_package_prefix
from ament_index_python.packages import get_package_share_directory, get_package_prefix

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

support_vehicle = ["x500", "rover_ackermann", "lc_62", "boat"]
without_px4 = []

# Vehicle type to autostart ID mapping (will be populated dynamically)

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
    if not os.path.exists(f'{px4_src_path}/build/px4_sitl_default/build_gz/'):
        print(
            f"Cannot find PX4 build file. Check your path or Please run the command 'make px4_sitl_default gz_500' in the path {px4_src_path} to build it first.")
        return False
    return True

def scan_airframes_directory(px4_build_path):
    """Scan PX4 airframes directory and build vehicle type to autostart ID mapping"""
    airframes_dir = os.path.join(px4_build_path, "ROMFS/px4fmu_common/init.d-posix/airframes")
    vehicle_autostart_map = {}
    
    if not os.path.exists(airframes_dir):
        print(f"Warning: Airframes directory not found at {airframes_dir}")
        return vehicle_autostart_map
    
    try:
        for filename in os.listdir(airframes_dir):
            # Look for files matching pattern: {autostart_id}_gz_{vehicle_type}
            if filename.startswith(tuple('0123456789')) and '_gz_' in filename:
                parts = filename.split('_gz_')
                if len(parts) == 2:
                    autostart_id = parts[0]
                    vehicle_type = parts[1]
                    vehicle_autostart_map[vehicle_type] = autostart_id
    except Exception as e:
        print(f"Error scanning airframes directory: {e}")
    
    return vehicle_autostart_map

def get_autostart_id(vehicle_type, px4_build_path):
    """Get PX4 autostart ID for vehicle type by scanning airframes directory"""
    vehicle_autostart_map = scan_airframes_directory(px4_build_path)
    
    if vehicle_type not in vehicle_autostart_map:
        available_types = list(vehicle_autostart_map.keys())
        print(f"ERROR: No airframe file found for vehicle type '{vehicle_type}'")
        print(f"Available vehicle types: {available_types}")
        print(f"Please check if airframe file '{vehicle_type}' exists in:")
        print(f"  {px4_build_path}/ROMFS/px4fmu_common/init.d-posix/airframes/")
        print(f"Expected file format: {{autostart_id}}_gz_{vehicle_type}")
        raise ValueError(f"Unsupported vehicle type: {vehicle_type}")
    
    return vehicle_autostart_map[vehicle_type]

def create_px4_command(vehicle, vehicle_type):
    """Create proper PX4 command with environment variables and arguments"""
    autostart_id = get_autostart_id(vehicle_type, vehicle['build_target'])
    
    # Create environment variables
    env_vars = {
        'PX4_GZ_STANDALONE': '1',
        'PX4_SYS_AUTOSTART': autostart_id,
        'PX4_UXRCE_DDS_NS' : f"vehicle{vehicle['id'] + 1}",
        'PX4_GZ_WORLD' : 'c-track'
    }
    
    # PX4 binary path
    px4_binary = f"{vehicle['build_target']}/build/px4_sitl_default/bin/px4"
    
    # Command arguments
    cmd_args = [
        px4_binary,
        '-i', str(vehicle['id'])
    ]
    
    return cmd_args, env_vars

def create_px4_param_command(vehicle, name, value):
    px4_param_cmd = [f"{vehicle['build_target']}/build/px4_sitl_default/bin/px4-param", "--instance", str(vehicle['id']), "set", name, str(value)]

    return px4_param_cmd

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

    # Validate build_target reference and vehicle type
    px4_target_paths = data.get('px4_target', {})
    for key, vehicle in vehicles.items():
        build_target = vehicle.get('build_target')
        if build_target not in px4_target_paths:
            print(f"Error: Vehicle {key} has an invalid build_target ({build_target}). It is not defined in px4_target.")
            return False

        # Validate vehicle type has corresponding airframe file
        vehicle_type = vehicle.get('type')
        build_target_path = px4_target_paths[build_target]
        try:
            get_autostart_id(vehicle_type, build_target_path)
        except ValueError as e:
            print(f"Error: Vehicle {key} validation failed: {e}")
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
    current_package_prefix = get_package_prefix('realgazebo')
    unreal_ip = LaunchConfiguration('unreal_ip').perform(context)
    unreal_port = LaunchConfiguration('unreal_port').perform(context)
    vehicle_str = LaunchConfiguration('vehicle').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    world = LaunchConfiguration('world').perform(context)
    if not validate_yaml(vehicle_str):
        exit(1)

    vehicle_lst = parse_yaml_to_list(vehicle_str)
    gazebo_path = f"{vehicle_lst[0]['build_target']}/Tools/simulation/gz"
    pairs = vehicle_str.split(',')

    # Load YAML to get px4_targets
    with open(vehicle_str, 'r') as file:
        data = yaml.safe_load(file)
    px4_targets = data.get('px4_target', {})

    # Environments
    model_path_env = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                                            f'$GZ_SIM_RESOURCE_PATH:{current_package_path}/models:{gazebo_path}/models:{gazebo_path}/worlds')

    # Build plugin paths for all px4_targets
    plugin_paths = [f"$GZ_SIM_SYSTEM_PLUGIN_PATH"]
    for target_name, target_path in px4_targets.items():
        plugin_paths.append(f"{target_path}/build/px4_sitl_default/src/modules/simulation/gz_plugins")
    plugin_paths.append(f"{current_package_prefix}/lib/realgazebo")

    plugin_path_env = SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',
                                             ':'.join(plugin_paths))

    # Get first px4_target for server config
    first_px4_target = list(px4_targets.values())[0]
    server_config_env = SetEnvironmentVariable('GZ_SIM_SERVER_CONFIG_PATH',
                                               f"{first_px4_target}/src/modules/simulation/gz_bridge/server.config")

    uxrce_dds_synct_param_env = SetEnvironmentVariable('PX4_PARAM_UXRCE_DDS_SYNCT', '0')
    
    # generate world file to /tmp/c-track.sdf if needed
    env = Environment(loader=FileSystemLoader(os.path.join(current_package_path, 'models', 'c-track')))
    world_model = env.get_template(f'model.sdf.jinja')
    output_world = world_model.render(world=world)
    world_model_path = os.path.join(current_package_path, 'models', 'c-track', 'model.sdf')
    print(world_model_path)
    with open(world_model_path, 'w') as f:
        f.write(output_world)
        print(f'c-track.sdf is generated')

    # it sometimes need to set GZ_IP to 127.0.0.1 or not so just use it
    gz_ip_env = SetEnvironmentVariable('GZ_IP', '127.0.0.1')

    gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    world_file_path = os.path.join(current_package_path, 'worlds', f'c-track.sdf')

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([gz_sim_pkg, 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': f'--verbose=1 -r -s {world_file_path}' if headless else f'--verbose=1 -r {world_file_path}'}.items()
    )

    uv_process_list = []
    rock_process_list = []

    xrce_agent_process = ExecuteProcess(
        cmd=[FindExecutable(name='MicroXRCEAgent'), 'udp4', '-p', '8888'])

    model_save_dir = os.path.join('/tmp', 'models')
    os.makedirs(model_save_dir, exist_ok=True)    
    model_list = support_vehicle + without_px4
    print(model_list)

    for model_type in model_list:
        ## generate sdf file to /tmp/{model_type}.sdf
        env = Environment(loader=FileSystemLoader(os.path.join(current_package_path, 'models')))
        model = env.get_template(f'{model_type}.sdf.jinja')
        output_model = model.render(unreal_ip=unreal_ip, unreal_port=unreal_port)
        model_file_path = os.path.join(model_save_dir, f'{model_type}.sdf')
        with open(model_file_path, 'w') as f:
            f.write(output_model)
            print(f'{model_type}.sdf is generated')

    for vehicle in vehicle_lst:
        vehicle_type = vehicle['type']

        spawn_entity = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([os.path.join(gz_sim_pkg, 'launch', 'gz_spawn_model.launch.py')])),
            launch_arguments={
                'world': 'c-track',
                'file': f'/tmp/models/{vehicle_type}.sdf',
                'entity_name': f'{vehicle_type}_{vehicle["id"]}',
                'x': f'{float(vehicle["spawnpoint"][0])}',
                'y': f'{float(vehicle["spawnpoint"][1])}',
                'z': f'{float(vehicle["spawnpoint"][2])}',
                'R': '0.0',
                'P': '0.0',
                'Y': f'{float(vehicle["spawnpoint"][3])}'
            }.items()
        )
        uv_process_list.append(spawn_entity)

        # Create PX4 process with proper autostart ID
        if vehicle_type not in without_px4:
            px4_cmd, px4_env = create_px4_command(vehicle, vehicle_type)
            
            px4_process = ExecuteProcess(
                cmd=px4_cmd,
                additional_env=px4_env,
            )
            uv_process_list.append(px4_process)

    for vehicle in vehicle_lst:
        px4_param_cmd = create_px4_param_command(vehicle, 'NAV_DLL_ACT', 0)
        px4_param_process = ExecuteProcess(cmd=px4_param_cmd)
        uv_process_list.append(px4_param_process)
        px4_param_cmd = create_px4_param_command(vehicle, 'COM_RCL_EXCEPT', 31)
        px4_param_process = ExecuteProcess(cmd=px4_param_cmd)
        uv_process_list.append(px4_param_process)

    gz_timesync_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ]
    )

    uv_actions_with_delays = create_timed_actions(
        uv_process_list,
        initial_delay=30.0,
        interval=0.5
    )

    nodes_to_start = [
        model_path_env,
        plugin_path_env,
        server_config_env,
        uxrce_dds_synct_param_env,
        gz_ip_env,
        xrce_agent_process,
        gazebo_node,
        *uv_actions_with_delays,
        gz_timesync_node
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
            'unreal_ip',
            default_value='127.0.0.1',
            description='ip of UE5'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'unreal_port',
            default_value='5005',
            description='port of UE5'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'headless',
            default_value='true',
            description='headless mode of Gazebo',
            choices=['true', 'false']
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'world',
            default_value='c-track',
            description='type of world',
            choices=['c-track', 'urban', 'vils']
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
