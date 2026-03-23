from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch vehicle network simulator (V2V + TC controller)."""

    # Launch arguments
    instance_id_arg = DeclareLaunchArgument(
        'instance_id',
        default_value='0',
        description='Vehicle instance ID (container ID, matches Gazebo model suffix)'
    )

    network_interface_arg = DeclareLaunchArgument(
        'network_interface',
        default_value='eth1',
        description='Network interface for TC control'
    )

    gz_world_name_arg = DeclareLaunchArgument(
        'gz_world_name',
        default_value='c-track',
        description='Gazebo world name for pose topic subscription'
    )

    # Network simulator node
    network_sim_node = Node(
        package='network_sim',
        executable='network_sim_node',
        namespace=['network_sim_', LaunchConfiguration('instance_id')],
        parameters=[{
            'instance_id': LaunchConfiguration('instance_id'),
            'network_interface': LaunchConfiguration('network_interface'),
            'gz_world_name': LaunchConfiguration('gz_world_name'),
            'enable_on_startup': True,
            'max_latency_ms': 1000.0,
            'max_jitter_ms': 500.0,
            'max_packet_loss_rate': 0.99,
        }],
        output='screen',
    )

    return LaunchDescription([
        instance_id_arg,
        network_interface_arg,
        gz_world_name_arg,
        network_sim_node,
    ])
