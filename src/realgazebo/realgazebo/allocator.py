from dataclasses import dataclass

# Per-vehicle resource formulas, mirrored 1:1 from scripts/generate_compose.py
# (and docker-compose.yml network subnets). These are LOAD-BEARING:
# vehicle.launch.py independently derives its FastDDS interfaceWhiteList as
# 172.30.0.{10+instance_id}, so a vehicle container must be attached to
# vehicle-network at exactly the address computed here.
GAZEBO_NET_PREFIX = '172.20.0.'   # gazebo-network (GZ transport)
VEHICLE_NET_PREFIX = '172.30.0.'  # vehicle-network (isolated DDS, internal)
IP_OFFSET = 10                    # gazebo server itself sits at .2
MAVLINK_PORT_BASE = 18570         # host-published UDP port for QGC
MAX_VEHICLE_ID = 244              # keeps the last octet within 10..254


@dataclass
class VehicleResources:
    vehicle_id: int
    gz_ip: str            # address on gazebo-network
    vehicle_ip: str       # address on vehicle-network
    mavlink_port: int     # host port {18570+id}:{18570+id}/udp
    dds_profile_path: str
    ros_namespace: str    # PX4_UXRCE_DDS_NS (note the +1 convention)


def allocate(vehicle_id: int) -> VehicleResources:
    """Compute the resource set for one vehicle container."""
    if not 0 <= vehicle_id <= MAX_VEHICLE_ID:
        raise ValueError(
            f"vehicle_id {vehicle_id} out of range 0..{MAX_VEHICLE_ID}")
    octet = IP_OFFSET + vehicle_id
    return VehicleResources(
        vehicle_id=vehicle_id,
        gz_ip=f'{GAZEBO_NET_PREFIX}{octet}',
        vehicle_ip=f'{VEHICLE_NET_PREFIX}{octet}',
        mavlink_port=MAVLINK_PORT_BASE + vehicle_id,
        dds_profile_path=f'/tmp/dds_profiles/px4_participant_{vehicle_id}.xml',
        ros_namespace=f'vehicle{vehicle_id + 1}',
    )
