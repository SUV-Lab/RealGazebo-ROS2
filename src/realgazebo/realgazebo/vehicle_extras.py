import os
import xml.etree.ElementTree as ET

# Per-vehicle extras shared by both execution modes. In docker mode
# vehicle.launch.py provides these inside each vehicle container; the
# subprocess backend uses the builders below to reach parity.

RTSP_PORT = 8554

# UE cameras streamed per vehicle type (RTSP path: {type}_{id}/{camera})
VEHICLE_CAMERAS = {
    'x500': ['front', 'bottom'],
    'x500_lidar_2d': ['front', 'bottom'],
    'x500_lidar_3d': ['front', 'bottom'],
    'lc_62': ['front', 'bottom'],
    'rover_ackermann': ['front', 'top'],
    'boat': ['front', 'top'],
}

SENSOR_BRIDGE_TYPES = {
    'gpu_lidar': [
        ('scan',        'sensor_msgs/msg/LaserScan',   'gz.msgs.LaserScan'),
        ('scan/points', 'sensor_msgs/msg/PointCloud2', 'gz.msgs.PointCloudPacked'),
    ],
}


def get_sensor_bridges(vehicle_type, vehicle_id, world, sdf_path,
                       model_search_paths=()):
    """Scan a rendered vehicle SDF for bridgeable sensors (gpu_lidar).

    Mirrors vehicle.launch.py's scan, but takes the world and paths
    explicitly (no hardcoded 'c-track'). Walks the model's own links plus
    one level of <include>d models resolved from model_search_paths.
    Returns ros_gz_bridge config entries.
    """
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
                'gz_topic_name': f'/world/{world}/model/{vehicle_type}_{vehicle_id}'
                                 f'/link/{link_name}/sensor/{sensor_name}/{suffix}',
                'ros_type_name': ros_type,
                'gz_type_name': gz_type,
                'direction': 'GZ_TO_ROS',
            })

    for link in model.findall('link'):
        for sensor in link.findall('sensor'):
            add_entries(link.get('name'), sensor.get('name'), sensor.get('type'))

    for include in model.findall('include'):
        uri = include.findtext('uri', '')
        model_name = uri.replace('model://', '')
        for search_path in model_search_paths:
            inc_sdf = os.path.join(search_path, model_name, 'model.sdf')
            if os.path.exists(inc_sdf):
                inc_model = ET.parse(inc_sdf).getroot().find('model')
                if inc_model is not None:
                    for link in inc_model.findall('link'):
                        for sensor in link.findall('sensor'):
                            add_entries(link.get('name'), sensor.get('name'),
                                        sensor.get('type'))
                break

    return bridges


def build_image_receiver_argv(vehicle_type, vehicle_id, camera_type,
                              unreal_ip, rtsp_port=RTSP_PORT):
    """argv for one UE camera receiver, started UNCONFIGURED (the
    image_viewer drives its lifecycle). The node name is the contract
    image_viewer uses to find it."""
    node_name = f'image_receiver_{vehicle_type}_{vehicle_id}_{camera_type}'
    return [
        'ros2', 'run', 'realgazebo', 'image_receiver_node', '--ros-args',
        '-r', f'__node:={node_name}',
        '-p', f'vehicle_type:={vehicle_type}',
        '-p', f'vehicle_id:={vehicle_id}',
        '-p', f'unreal_ip:={unreal_ip}',
        '-p', f'rtsp_port:={rtsp_port}',
        '-p', f'camera_type:={camera_type}',
    ]


def build_sensor_bridge_argv(config_path, node_name):
    return [
        'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '--ros-args',
        '-r', f'__node:={node_name}',
        '-p', f'config_file:={config_path}',
    ]
