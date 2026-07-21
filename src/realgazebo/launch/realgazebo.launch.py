import os
import random
import yaml
import ast
import xml.etree.ElementTree as ET

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
from launch.actions import RegisterEventHandler

support_vehicle = ["x500", "x500_lidar_2d", "rover_ackermann", "lc_62", "boat"]
support_obstacle = ["rock"]
without_px4 = []

VEHICLE_CAMERAS = {
    'x500': ['front', 'bottom'],
    'x500_lidar_2d': ['front', 'bottom'],
    'lc_62': ['front', 'bottom'],
    'rover_ackermann': ['front', 'top'],
    'boat': ['front', 'top'],
}

UE5_VEHICLE_TYPE = {
    'x500_lidar_2d': 'x500',
}

SENSOR_BRIDGE_TYPES = {
    'gpu_lidar': [
        ('scan',        'sensor_msgs/msg/LaserScan',   'gz.msgs.LaserScan'),
        ('scan/points', 'sensor_msgs/msg/PointCloud2', 'gz.msgs.PointCloudPacked'),
    ],
}


def get_sensor_bridges(vehicle_type, vehicle_id, world, model_search_paths=None):
    sdf_path = f'/tmp/models/{vehicle_type}.sdf'
    if not os.path.exists(sdf_path):
        return []
    model = ET.parse(sdf_path).getroot().find('model')
    if model is None:
        return []

    bridges = []
    # ROS namespaces are 1-based, unlike the 0-based vehicle_id: /vehicle1
    # belongs to vehicle_id 0. Kept as a separate name so the two never blur.
    ros_index = vehicle_id + 1

    def add_entries(link_name, sensor_name, sensor_type):
        for suffix, ros_type, gz_type in SENSOR_BRIDGE_TYPES.get(sensor_type, []):
            bridges.append({
                'ros_topic_name': f'/vehicle{ros_index}/{suffix}',
                'gz_topic_name': f'/world/{world}/model/{vehicle_type}_{vehicle_id}/link/{link_name}/sensor/{sensor_name}/{suffix}',
                'ros_type_name': ros_type,
                'gz_type_name': gz_type,
                'direction': 'GZ_TO_ROS',
            })

    for link in model.findall('link'):
        for sensor in link.findall('sensor'):
            add_entries(link.get('name'), sensor.get('name'), sensor.get('type'))

    if model_search_paths:
        for include in model.findall('include'):
            uri = include.findtext('uri', '')
            model_name = uri.replace('model://', '')
            for search_path in model_search_paths:
                inc_sdf = os.path.join(search_path, model_name, 'model.sdf')
                if os.path.exists(inc_sdf):
                    inc_model = ET.parse(inc_sdf).getroot().find('model')
                    if inc_model:
                        for link in inc_model.findall('link'):
                            for sensor in link.findall('sensor'):
                                add_entries(link.get('name'), sensor.get('name'), sensor.get('type'))
                    break

    return bridges


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
        if vehicle.get('type') not in support_obstacle:
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
    obstacle_list = []

    vehicles = data.get('vehicles', {})
    for key in sorted(vehicles.keys()):
        vehicle = vehicles[key]

        entity_type = vehicle.get('type')
        build_target_key = vehicle.get('build_target')
        build_target_path = px4_targets.get(build_target_key)
        spawnpoint = ast.literal_eval(vehicle.get('spawnpoint'))

        if entity_type in support_obstacle:
            obstacle_dict = {
                'id': key,
                'type': entity_type,
                'spawnpoint': spawnpoint
            }
            obstacle_list.append(obstacle_dict)
        
        else:
            vehicle_dict = {
                'id': key,
                'type': entity_type,
                'build_target': build_target_path,
                'spawnpoint': spawnpoint
            }

            vehicles_list.append(vehicle_dict)

    return vehicles_list, obstacle_list

def launch_setup(context, *args, **kwargs):
    # Configuration
    current_package_path = get_package_share_directory('realgazebo')
    current_package_prefix = get_package_prefix('realgazebo')
    unreal_ip = LaunchConfiguration('unreal_ip').perform(context)
    unreal_port = LaunchConfiguration('unreal_port').perform(context)
    rtsp_port = LaunchConfiguration('rtsp_port').perform(context)
    vehicle_str = LaunchConfiguration('vehicle').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    verbose = LaunchConfiguration('verbose').perform(context).lower() == 'true'
    world = LaunchConfiguration('world').perform(context)
    if not validate_yaml(vehicle_str):
        exit(1)

    vehicle_lst, obstacle_lst = parse_yaml_to_list(vehicle_str)
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
    with open(world_model_path, 'w') as f:
        f.write(output_world)
        print(f'c-track.sdf is generated')

    # it sometimes need to set GZ_IP to 127.0.0.1 or not so just use it
    gz_ip_env = SetEnvironmentVariable('GZ_IP', '127.0.0.1')

    gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    world_file_path = os.path.join(current_package_path, 'worlds', f'c-track.sdf')

    verbose_level = 4 if verbose else 1
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([gz_sim_pkg, 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': f'--verbose={verbose_level} -r -s {world_file_path}' if headless else f'--verbose={verbose_level} -r {world_file_path}'}.items()
    )

    uv_process_list = []
    obstacle_process_list = []

    xrce_agent_process = ExecuteProcess(
        cmd=[FindExecutable(name='MicroXRCEAgent'), 'udp4', '-p', '8888'])

    model_save_dir = os.path.join('/tmp', 'models')
    os.makedirs(model_save_dir, exist_ok=True)    
    model_list = support_vehicle + support_obstacle + without_px4

    for model_type in model_list:
        ## generate sdf file to /tmp/{model_type}.sdf
        env = Environment(loader=FileSystemLoader(os.path.join(current_package_path, 'models')))
        model = env.get_template(f'{model_type}.sdf.jinja' if model_type not in support_obstacle else f'{model_type}/{model_type}.sdf.jinja')
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

        # Create image_receiver nodes (one per camera)
        cameras = VEHICLE_CAMERAS.get(vehicle_type, ['front'])
        for camera_type in cameras:
            receiver_node = Node(
                package='realgazebo',
                executable='image_receiver_node',
                name=f'image_receiver_{vehicle_type}_{vehicle["id"]}_{camera_type}',
                parameters=[{
                    'vehicle_type': UE5_VEHICLE_TYPE.get(vehicle_type, vehicle_type),
                    'vehicle_id': vehicle['id'],
                    'unreal_ip': unreal_ip,
                    'rtsp_port': int(rtsp_port),
                    'camera_type': camera_type,
                }]
            )
            uv_process_list.append(receiver_node)


    for obstacle in obstacle_lst:
        obstacle_type = obstacle['type']
        spawn_entity = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([os.path.join(gz_sim_pkg, 'launch', 'gz_spawn_model.launch.py')])),
            launch_arguments={
                'world': 'c-track',
                'file': f'/tmp/models/{obstacle_type}.sdf',
                'entity_name': f'{obstacle_type}_{obstacle["id"]}',
                'x': f'{float(obstacle["spawnpoint"][0])}',
                'y': f'{float(obstacle["spawnpoint"][1])}',
                'z': f'{float(obstacle["spawnpoint"][2])}',
                'R': '0.0',
                'P': '0.0',
                'Y': f'{float(obstacle["spawnpoint"][3])}'
            }.items()
        )
        obstacle_process_list.append(spawn_entity)


    # parameter must be change after all vehicles and obstacles are spawned
    for vehicle in vehicle_lst:
        px4_param_cmd = create_px4_param_command(vehicle, 'NAV_DLL_ACT', 0)
        px4_param_process = ExecuteProcess(cmd=px4_param_cmd)
        uv_process_list.append(px4_param_process)
        px4_param_cmd = create_px4_param_command(vehicle, 'COM_RCL_EXCEPT', 31)
        px4_param_process = ExecuteProcess(cmd=px4_param_cmd)
        uv_process_list.append(px4_param_process)
        px4_param_cmd = create_px4_param_command(vehicle, 'COM_RC_IN_MODE', 4)
        px4_param_process = ExecuteProcess(cmd=px4_param_cmd)
        uv_process_list.append(px4_param_process)

    # Build combined bridge config: clock + all vehicle sensor bridges
    gz_bridge_entries = [
        {
            'ros_topic_name': '/clock',
            'gz_topic_name': '/clock',
            'ros_type_name': 'rosgraph_msgs/msg/Clock',
            'gz_type_name': 'gz.msgs.Clock',
            'direction': 'GZ_TO_ROS',
        }
    ]
    model_search_paths = [
        os.path.join(current_package_path, 'models'),
        os.path.join(gazebo_path, 'models'),
    ]
    for vehicle in vehicle_lst:
        vehicle_type = vehicle['type']
        gz_bridge_entries.extend(get_sensor_bridges(vehicle_type, vehicle['id'], 'c-track', model_search_paths))

    os.makedirs('/tmp/bridges', exist_ok=True)
    combined_cfg_path = '/tmp/bridges/combined.yaml'
    with open(combined_cfg_path, 'w') as f:
        yaml.dump(gz_bridge_entries, f)

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        parameters=[{'config_file': combined_cfg_path}]
    )

    # Wait for Gazebo to publish /clock before spawning vehicles/obstacles.
    # 'gz topic -e -t /clock' blocks until the first clock message arrives,
    # so exiting cleanly means Gazebo is fully up.
    gz_ready_check = ExecuteProcess(
        cmd=['bash', '-c', 'gz topic -e -t /clock 2>/dev/null | head -1'],
        output='screen',
        name='gz_ready_check',
    )

    uv_actions_with_delays = create_timed_actions(
        uv_process_list,
        initial_delay=0.5,
        interval=0.5
    )

    obstacle_actions_with_delays = create_timed_actions(
        obstacle_process_list,
        initial_delay=0.5,
        interval=0.5
    )

    spawn_on_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_ready_check,
            on_exit=[
                *uv_actions_with_delays,
                *obstacle_actions_with_delays,
            ]
        )
    )

    nodes_to_start = [
        model_path_env,
        plugin_path_env,
        server_config_env,
        uxrce_dds_synct_param_env,
        gz_ip_env,
        xrce_agent_process,
        gazebo_node,
        gz_ready_check,
        spawn_on_ready,
        gz_bridge_node
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
            'rtsp_port',
            default_value='8554',
            description='RTSP port for UE5 camera streams'
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
            description='type of world',
            choices=['c-track', 'urban', 'vils']
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
