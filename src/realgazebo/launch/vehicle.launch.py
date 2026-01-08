"""
Vehicle launch file for RealGazebo multi-container setup.

This launch file starts a single vehicle instance with:
- MicroXRCEAgent (DDS bridge)
- Vehicle spawn into Gazebo
- PX4 SITL instance
- PX4 parameter configuration
- ROS2 control nodes (optional)
"""

import os

from jinja2 import Environment, FileSystemLoader

from ament_index_python.packages import get_package_share_directory, get_package_prefix

import launch
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)


def scan_airframes_directory(px4_build_path):
    """Scan PX4 airframes directory and build vehicle type to autostart ID mapping"""
    airframes_dir = os.path.join(px4_build_path, "ROMFS/px4fmu_common/init.d-posix/airframes")
    vehicle_autostart_map = {}

    if not os.path.exists(airframes_dir):
        print(f"Warning: Airframes directory not found at {airframes_dir}")
        return vehicle_autostart_map

    try:
        for filename in os.listdir(airframes_dir):
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
        raise ValueError(f"Unsupported vehicle type: {vehicle_type}")

    return vehicle_autostart_map[vehicle_type]


def create_timed_actions(actions_list, initial_delay, interval):
    """Create timed actions with delays"""
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


def launch_setup(context, *args, **kwargs):
    # Configuration
    current_package_path = get_package_share_directory('realgazebo')
    current_package_prefix = get_package_prefix('realgazebo')

    instance_id = int(LaunchConfiguration('instance_id').perform(context))
    vehicle_type = LaunchConfiguration('vehicle_type').perform(context)
    spawnpoint_str = LaunchConfiguration('spawnpoint').perform(context)
    px4_path = LaunchConfiguration('px4_path').perform(context)
    unreal_ip = LaunchConfiguration('unreal_ip').perform(context)
    unreal_port = LaunchConfiguration('unreal_port').perform(context)
    start_control_node = LaunchConfiguration('start_control_node').perform(context).lower() == 'true'

    # Parse spawnpoint: "x,y,z,yaw"
    spawnpoint = [float(x.strip()) for x in spawnpoint_str.split(',')]
    if len(spawnpoint) != 4:
        raise ValueError(f"Spawnpoint must have 4 values (x,y,z,yaw), got: {spawnpoint_str}")

    gazebo_path = f"{px4_path}/Tools/simulation/gz"

    # Environment variables
    model_path_env = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        f'$GZ_SIM_RESOURCE_PATH:{current_package_path}/models:{gazebo_path}/models:{gazebo_path}/worlds'
    )

    plugin_paths = [
        "$GZ_SIM_SYSTEM_PLUGIN_PATH",
        f"{px4_path}/build/px4_sitl_default/src/modules/simulation/gz_plugins",
        f"{current_package_prefix}/lib/realgazebo",
    ]
    plugin_path_env = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        ':'.join(plugin_paths)
    )

    uxrce_dds_synct_param_env = SetEnvironmentVariable('PX4_PARAM_UXRCE_DDS_SYNCT', '0')
    uxrce_dds_ptcfg_env = SetEnvironmentVariable('PX4_PARAM_UXRCE_DDS_PTCFG', '2')  # Use px4_participant profile

    actions = []
    timed_actions = []

    # DDS profile for vehicle-network only communication
    # Generate dynamically with correct IP for this vehicle
    vehicle_network_ip = f'172.30.0.{10 + instance_id}'
    dds_profile_content = f'''<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>vehicle_udp</transport_id>
            <type>UDPv4</type>
            <interfaceWhiteList>
                <address>{vehicle_network_ip}</address>
            </interfaceWhiteList>
        </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="px4_participant" is_default_profile="true">
        <rtps>
            <useBuiltinTransports>false</useBuiltinTransports>
            <userTransports>
                <transport_id>vehicle_udp</transport_id>
            </userTransports>
        </rtps>
    </participant>
</profiles>'''

    dds_profile_dir = '/tmp/dds_profiles'
    os.makedirs(dds_profile_dir, exist_ok=True)
    dds_profile_path = os.path.join(dds_profile_dir, f'px4_participant_{instance_id}.xml')
    with open(dds_profile_path, 'w') as f:
        f.write(dds_profile_content)

    fastrtps_env = SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', dds_profile_path)

    # 1. MicroXRCEAgent - runs on port 8888, uses vehicle-network interface only
    xrce_agent_process = ExecuteProcess(
        cmd=[FindExecutable(name='MicroXRCEAgent'), 'udp4', '-p', '8888', '-r', dds_profile_path]
    )
    actions.append(xrce_agent_process)

    # 2. Generate vehicle SDF from Jinja template
    model_save_dir = os.path.join('/tmp', 'models')
    os.makedirs(model_save_dir, exist_ok=True)

    env = Environment(loader=FileSystemLoader(os.path.join(current_package_path, 'models')))
    model = env.get_template(f'{vehicle_type}.sdf.jinja')
    output_model = model.render(unreal_ip=unreal_ip, unreal_port=unreal_port)
    model_file_path = os.path.join(model_save_dir, f'{vehicle_type}.sdf')
    with open(model_file_path, 'w') as f:
        f.write(output_model)
        print(f'{vehicle_type}.sdf generated for instance {instance_id}')

    # 3. Spawn vehicle into Gazebo (after delay for Gazebo connection)
    gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gz_sim_pkg, 'launch', 'gz_spawn_model.launch.py'])
        ),
        launch_arguments={
            'world': 'c-track',
            'file': model_file_path,
            'entity_name': f'{vehicle_type}_{instance_id}',
            'x': str(spawnpoint[0]),
            'y': str(spawnpoint[1]),
            'z': str(spawnpoint[2]),
            'R': '0.0',
            'P': '0.0',
            'Y': str(spawnpoint[3])
        }.items()
    )
    timed_actions.append(spawn_entity)

    # 4. PX4 SITL instance (after spawn)
    autostart_id = get_autostart_id(vehicle_type, px4_path)

    px4_env = {
        'PX4_GZ_STANDALONE': '1',
        'PX4_SYS_AUTOSTART': autostart_id,
        'PX4_GZ_MODEL_NAME': f'{vehicle_type}_{instance_id}',
        'PX4_UXRCE_DDS_NS': f'vehicle{instance_id + 1}',
        'PX4_GZ_WORLD': 'c-track'
    }

    px4_binary = f"{px4_path}/build/px4_sitl_default/bin/px4"
    px4_process = ExecuteProcess(
        cmd=[px4_binary, '-i', str(instance_id)],
        additional_env=px4_env,
        output='screen',
    )
    timed_actions.append(px4_process)

    # 5. PX4 parameter configuration (after PX4 starts)
    px4_param_binary = f"{px4_path}/build/px4_sitl_default/bin/px4-param"

    param_commands = [
        (px4_param_binary, '--instance', str(instance_id), 'set', 'NAV_DLL_ACT', '0'),
        (px4_param_binary, '--instance', str(instance_id), 'set', 'COM_RCL_EXCEPT', '31'),
        (px4_param_binary, '--instance', str(instance_id), 'set', 'COM_RC_IN_MODE', '4'),
    ]

    for cmd in param_commands:
        param_process = ExecuteProcess(cmd=list(cmd))
        timed_actions.append(param_process)

    # 6. ROS2 control nodes (optional, after everything is ready)
    if start_control_node:
        control_node = Node(
            package='drone_controller',
            executable='drone_controller',
            name=f'drone_controller_{instance_id}',
            parameters=[{
                'use_sim_time': True,
                'vehicle_id': instance_id + 1
            }]
        )
        timed_actions.append(control_node)

    # 7. Network Simulator (V2V + TC Controller integrated with intra-process communication)
    network_sim_node = Node(
        package='network_sim',
        executable='network_sim_node',
        namespace=f'network_sim_{instance_id}',
        parameters=[{
            'reference_vehicle_id': instance_id + 1,
            # TC controller parameters
            'instance_id': instance_id,
            'network_interface': 'eth1',
            'enable_on_startup': True,
            'max_latency_ms': 1000.0,
            'max_jitter_ms': 500.0,
            'max_packet_loss_rate': 0.99,
        }],
        output='screen'
    )
    timed_actions.append(network_sim_node)

    # Apply timing: spawn at T+10s, then 5s interval for subsequent actions
    # Increased delays to ensure gz-transport discovery completes before PX4 subscribes
    timed_action_nodes = create_timed_actions(
        timed_actions,
        initial_delay=10.0,
        interval=5.0
    )

    nodes_to_start = [
        model_path_env,
        plugin_path_env,
        uxrce_dds_synct_param_env,
        uxrce_dds_ptcfg_env,
        fastrtps_env,
        *actions,
        *timed_action_nodes,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'instance_id',
            description='Vehicle instance ID (0, 1, 2, ...)'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'vehicle_type',
            default_value='x500',
            description='Vehicle type (x500, rover_ackermann, lc_62, boat)'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'spawnpoint',
            default_value='0,0,0,0',
            description='Spawn position as "x,y,z,yaw"'
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

    declared_arguments.append(
        DeclareLaunchArgument(
            'start_control_node',
            default_value='false',
            description='Whether to start the drone controller node',
            choices=['true', 'false']
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
