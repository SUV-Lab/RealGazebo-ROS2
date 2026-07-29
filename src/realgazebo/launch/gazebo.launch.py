"""
Gazebo-only launch file for RealGazebo multi-container setup.

This launch file starts only the Gazebo simulator with the world file.
Vehicles are spawned from separate vehicle containers.
"""

import os
import glob

from jinja2 import Environment, FileSystemLoader

from realgazebo.worlds import resolve_world_file

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
    terrain = LaunchConfiguration('terrain').perform(context)
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

    # Render the shipped c-track terrain model with the requested crop.
    # `terrain` picks which STL that model shows: 'c-track' is the full site,
    # 'urban'/'vils' are smaller crops of the SAME site, kept so a small-scale
    # run does not have to load the full 1.15 GB mesh into Gazebo.
    #
    # It is deliberately independent of `world`: a terrain swap must never
    # change the gz world name (see realgazebo/worlds.py). If you run your own
    # world it will not include model://c-track, so this render is inert for
    # you - a small XML write, nothing loaded.
    env = Environment(loader=FileSystemLoader(os.path.join(current_package_path, 'models', 'c-track')))
    world_model = env.get_template('model.sdf.jinja')
    output_world = world_model.render(terrain=terrain)
    world_model_path = os.path.join(current_package_path, 'models', 'c-track', 'model.sdf')
    with open(world_model_path, 'w') as f:
        f.write(output_world)
        print(f'c-track model.sdf generated (terrain={terrain})')

    # Render EVERY vehicle/obstacle template into this container's
    # /tmp/models: spawn requests pass a /tmp/models/<type>.sdf path that the
    # gz server resolves on ITS OWN filesystem, so a type missing here fails
    # with 'Error finding file'. Scanning the templates (instead of a
    # hardcoded list) keeps new types working automatically — a stale list
    # here silently dropped x500_lidar_2d while vehicle containers reported
    # 'Entity creation successful'.
    model_save_dir = os.path.join('/tmp', 'models')
    os.makedirs(model_save_dir, exist_ok=True)

    models_root = os.path.join(current_package_path, 'models')
    template_names = [os.path.basename(p)
                      for p in sorted(glob.glob(os.path.join(models_root, '*.sdf.jinja')))]
    for path in sorted(glob.glob(os.path.join(models_root, '*', '*.sdf.jinja'))):
        # obstacle-style templates live in <name>/<name>.sdf.jinja
        # (world templates like c-track/model.sdf.jinja are skipped)
        stem = os.path.basename(path)[:-len('.sdf.jinja')]
        if os.path.basename(os.path.dirname(path)) == stem:
            template_names.append(f'{stem}/{stem}.sdf.jinja')

    env = Environment(loader=FileSystemLoader(models_root))
    for template_name in template_names:
        model = env.get_template(template_name)
        output_model = model.render(unreal_ip=unreal_ip, unreal_port=unreal_port)
        out_name = os.path.basename(template_name)[:-len('.jinja')]
        with open(os.path.join(model_save_dir, out_name), 'w') as f:
            f.write(output_model)
            print(f'{out_name} generated')

    # Launch Gazebo
    gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    world_file_path = resolve_world_file(current_package_path, world)

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

    # world and terrain are independent. `world` decides WHICH WORLD RUNS -
    # it selects worlds/<world>.sdf and fixes the gz world name that the
    # manager, PX4 and the bridges all address. `terrain` only decides which
    # STL the shipped c-track terrain model shows, and never touches that name.
    # Neither has a choices= list on purpose: adding a world means dropping in
    # worlds/<name>.sdf, adding a terrain crop means dropping in
    # models/c-track/meshes/<name>.stl. Nothing here should have to be taught
    # the names of c-track's crops.
    declared_arguments.append(
        DeclareLaunchArgument(
            'world',
            default_value='c-track',
            description='World to run: loads worlds/<world>.sdf, whose '
                        '<world name=> must equal <world>'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'terrain',
            default_value='c-track',
            description='Which STL the c-track terrain model shows '
                        '(c-track = full site, urban/vils = smaller crops). '
                        'Ignored by worlds that do not include model://c-track'
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
