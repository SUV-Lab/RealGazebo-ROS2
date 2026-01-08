from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Launch vehicle network simulator (V2V + TC controller)."""

    # Launch arguments
    instance_id_arg = DeclareLaunchArgument(
        'instance_id',
        default_value='0',
        description='Vehicle instance ID (container ID, not ROS2 topic ID)'
    )

    reference_vehicle_id_arg = DeclareLaunchArgument(
        'reference_vehicle_id',
        default_value='1',
        description='Reference vehicle ID for V2V (ROS2 topic ID, typically instance_id + 1)'
    )

    network_interface_arg = DeclareLaunchArgument(
        'network_interface',
        default_value='eth1',
        description='Network interface for TC control'
    )

    # Network simulator node
    network_sim_node = Node(
        package='network_sim',
        executable='network_sim_node',
        namespace=['network_sim_', LaunchConfiguration('instance_id')],
        parameters=[{
            # V2V parameters
            # Note: reference_vehicle_id should match ROS2 topic (/vehicle1, /vehicle2, ...)
            # which is typically instance_id + 1
            'reference_vehicle_id': LaunchConfiguration('reference_vehicle_id'),
            # TC controller parameters
            'instance_id': LaunchConfiguration('instance_id'),
            'network_interface': LaunchConfiguration('network_interface'),
            'enable_on_startup': True,
            'max_latency_ms': 1000.0,
            'max_jitter_ms': 500.0,
            'max_packet_loss_rate': 0.99,
        }],
        output='screen',
    )

    return LaunchDescription([
        instance_id_arg,
        reference_vehicle_id_arg,
        network_interface_arg,
        network_sim_node,
    ])
