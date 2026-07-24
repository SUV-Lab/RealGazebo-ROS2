import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    ExecuteProcess, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration, FindExecutable, PythonExpression)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    realgazebo_share = get_package_share_directory('realgazebo')
    gazebo_launch = os.path.join(realgazebo_share, 'launch', 'gazebo.launch.py')

    args = [
        DeclareLaunchArgument(
            'yaml_path',
            default_value='',
            description='Vehicle YAML to spawn at boot; '
                        'empty (default) = no boot spawn, UDP-only mode'),
        DeclareLaunchArgument('world', default_value='c-track'),
        DeclareLaunchArgument('unreal_ip', default_value='127.0.0.1'),
        DeclareLaunchArgument('unreal_port', default_value='5005'),
        DeclareLaunchArgument(
            'px4_path', default_value='/home/user/realgazebo/RealGazebo-PX4'),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument(
            'backend', default_value='subprocess',
            description='How vehicles are materialized: subprocess '
                        '(monolithic) or docker (one container per vehicle)'),
        DeclareLaunchArgument(
            'docker_image', default_value='aware4docker/realgazebo:1.3-rc1',
            description='Image for runtime vehicle containers (docker backend)'),
        DeclareLaunchArgument(
            'mavlink_gcs_ip', default_value='172.17.0.1',
            description='GCS (QGC) address injected into vehicle containers'),
    ]

    # One shared gz-transport partition for every participant (gz server,
    # manager, extras, PX4/bridge subprocesses, docker vehicles, remote PILS
    # SITLs). Without this the default partition is hostname:user, which can
    # never match across hosts/containers.
    # When the fleet spans hosts (operator exported GZ_IP), pin every DDS
    # participant in this container to that one address. Multi-homed hosts
    # (docker bridges, secondary NICs, VPNs) otherwise advertise ALL their
    # locators, and remote uXRCE agents wedge on discovery against the
    # unreachable ones: measured on the bench, >~10 local participants
    # (camera receivers) killed every FC/PILS agent session within ~30 s on
    # a ~40 s relapse cycle, while this whitelist kept the same fleet
    # streaming indefinitely. Local-only runs (no GZ_IP) are untouched.
    launch_actions_dds = []
    if os.environ.get('GZ_IP'):
        _wl_path = '/tmp/realgazebo_dds_whitelist.xml'
        with open(_wl_path, 'w') as _f:
            _f.write(f'''<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_whitelist</transport_id>
      <type>UDPv4</type>
      <interfaceWhiteList><address>{os.environ['GZ_IP']}</address></interfaceWhiteList>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="realgazebo_wl" is_default_profile="true">
    <rtps>
      <userTransports><transport_id>udp_whitelist</transport_id></userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
''')
        launch_actions_dds.append(SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE', _wl_path))

    gz_partition = SetEnvironmentVariable(
        'GZ_PARTITION', os.environ.get('GZ_PARTITION', 'realgazebo'))

    # Pin GZ transport to localhost for the monolithic run (matches legacy)
    # unless the operator already exported GZ_IP: a fleet with PILS vehicles
    # must advertise the host's LAN address so remote SITLs can discover the
    # world (GZ_IP=<lan ip> scripts/run_realgazebo.sh ...).
    # In docker mode this must NOT fire: the gz server has to announce its
    # gazebo-network address (container env GZ_IP, e.g. 172.20.0.2) or
    # sibling vehicle containers can never discover it.
    gz_ip = SetEnvironmentVariable(
        'GZ_IP', os.environ.get('GZ_IP', '127.0.0.1'),
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration('backend'), "' == 'subprocess'"])))

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

    # One shared micro-XRCE-DDS agent — monolithic mode only (in docker
    # mode every vehicle container runs its own isolated agent)
    xrce_agent = ExecuteProcess(
        cmd=[FindExecutable(name='MicroXRCEAgent'), 'udp4', '-p', '8888'],
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration('backend'), "' == 'subprocess'"])))

    manager = Node(
        package='realgazebo',
        executable='manager_node',
        name='realgazebo_manager',
        output='screen',
        parameters=[{
            'yaml_path': LaunchConfiguration('yaml_path'),
            'world': LaunchConfiguration('world'),
            'unreal_ip': LaunchConfiguration('unreal_ip'),
            'unreal_port': ParameterValue(
                LaunchConfiguration('unreal_port'), value_type=int),
            'backend': LaunchConfiguration('backend'),
            'docker_image': LaunchConfiguration('docker_image'),
            'mavlink_gcs_ip': LaunchConfiguration('mavlink_gcs_ip'),
            'default_px4_path': LaunchConfiguration('px4_path'),
        }])

    return LaunchDescription(
        args + launch_actions_dds
        + [gz_partition, gz_ip, gazebo, xrce_agent, manager])
