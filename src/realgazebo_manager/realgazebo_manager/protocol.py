import struct
from collections import namedtuple

# RealGazebo UDP wire protocol (must match the gz plugin / UE side).
# Header: vehicle_num(u8), vehicle_code(u8), message_id(u8).
HEADER_SIZE = 3
MSG_POSE = 1     # pose packet; reused as the inbound SPAWN command (UE -> manager)
MSG_DESTROY = 4  # destroy packet; reused as the inbound DESPAWN command

# Pose payload: 7 little-endian float32 = pos(x, y, z) + quat(x, y, z, w),
# expressed in the Gazebo frame (meters, right-handed).
_POSE = struct.Struct('<7f')

SpawnCommand = namedtuple(
    'SpawnCommand', ['vehicle_num', 'vehicle_code', 'position', 'quaternion'])
DespawnCommand = namedtuple('DespawnCommand', ['vehicle_num', 'vehicle_code'])


def parse_packet(data: bytes):
    """Parse a wire packet into a command.

    Returns a SpawnCommand (MessageID=1), a DespawnCommand (MessageID=4), or
    None for other message ids. Raises ValueError on a malformed packet.
    """
    if len(data) < HEADER_SIZE:
        raise ValueError("packet shorter than header")
    vehicle_num, vehicle_code, message_id = data[0], data[1], data[2]
    if message_id == MSG_POSE:
        payload = data[HEADER_SIZE:]
        if len(payload) < _POSE.size:
            raise ValueError("MessageID=1 payload too short for pose")
        x, y, z, qx, qy, qz, qw = _POSE.unpack(payload[:_POSE.size])
        return SpawnCommand(vehicle_num, vehicle_code, (x, y, z), (qx, qy, qz, qw))
    if message_id == MSG_DESTROY:
        return DespawnCommand(vehicle_num, vehicle_code)
    return None


def pack_pose(vehicle_num, vehicle_code, position, quaternion) -> bytes:
    """Build a MessageID=1 pose packet (used by tests and golden vectors)."""
    return bytes([vehicle_num, vehicle_code, MSG_POSE]) + _POSE.pack(
        *position, *quaternion)


def pack_destroy(vehicle_num, vehicle_code) -> bytes:
    """Build a MessageID=4 destroy packet (used by tests)."""
    return bytes([vehicle_num, vehicle_code, MSG_DESTROY])
