import struct
from collections import namedtuple

# RealGazebo UDP wire protocol (must match the gz plugin / UE side).
# Header: entity_id(u8), type_code(u8), message_id(u8).
# type_code 0..199 = PX4 vehicles, >= 200 = static props/obstacles
# (see type_codes.PROP_CODE_MIN).
HEADER_SIZE = 3
# Pose packet, reused inbound (UE -> manager) as an UPSERT:
#  - id unknown            -> SPAWN (vehicle or prop)
#  - id active, same code  -> prop: MOVE (set_pose); vehicle: ignored
#  - id active, other code -> dropped (sender bug; ids are global)
MSG_POSE = 1
# Destroy packet, reused as the inbound DESPAWN command. The same
# code-vs-holder validation applies: an id active under another code
# is dropped, never despawned.
MSG_DESTROY = 4

# Pose payload: 7 little-endian float32 = pos(x, y, z) + quat(x, y, z, w),
# expressed in the Gazebo frame (meters, right-handed).
_POSE = struct.Struct('<7f')

SpawnCommand = namedtuple(
    'SpawnCommand', ['entity_id', 'type_code', 'position', 'quaternion'])
DespawnCommand = namedtuple('DespawnCommand', ['entity_id', 'type_code'])


def parse_packet(data: bytes):
    """Parse a wire packet into a command.

    Returns a SpawnCommand (MessageID=1), a DespawnCommand (MessageID=4), or
    None for other message ids. Raises ValueError on a malformed packet.
    """
    if len(data) < HEADER_SIZE:
        raise ValueError("packet shorter than header")
    entity_id, type_code, message_id = data[0], data[1], data[2]
    if message_id == MSG_POSE:
        payload = data[HEADER_SIZE:]
        if len(payload) < _POSE.size:
            raise ValueError("MessageID=1 payload too short for pose")
        x, y, z, qx, qy, qz, qw = _POSE.unpack(payload[:_POSE.size])
        return SpawnCommand(entity_id, type_code, (x, y, z), (qx, qy, qz, qw))
    if message_id == MSG_DESTROY:
        return DespawnCommand(entity_id, type_code)
    return None


def pack_pose(entity_id, type_code, position, quaternion) -> bytes:
    """Build a MessageID=1 pose packet (used by tests and golden vectors)."""
    return bytes([entity_id, type_code, MSG_POSE]) + _POSE.pack(
        *position, *quaternion)


def pack_destroy(entity_id, type_code) -> bytes:
    """Build a MessageID=4 destroy packet (used by tests)."""
    return bytes([entity_id, type_code, MSG_DESTROY])
