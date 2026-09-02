import math
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
# World wind (UE <-> manager). A WORLD-level message: the header ids carry
# nothing and are ignored (the sender puts 0, 0). Payload: op(u8) +
# 3 little-endian float32 = wind linear velocity in the Gazebo world frame
# (m/s, right-handed, no UE-frame conversion) - 16 bytes in total.
#   UE -> manager: op 0 = wind OFF (vector ignored, parsed as zeros),
#                  1 = wind ON, 2 = QUERY the current state (vector ignored).
#   manager -> UE: the answer to a QUERY, same layout with op 0/1 = the
#                  current enable flag + vector, sent to the asker's IP on
#                  the UE port (unreal_port).
MSG_WIND = 6
WIND_OP_OFF = 0
WIND_OP_ON = 1
WIND_OP_QUERY = 2

# Pose payload: 7 little-endian float32 = pos(x, y, z) + quat(x, y, z, w),
# expressed in the Gazebo frame (meters, right-handed).
_POSE = struct.Struct('<7f')
# Wind payload: enable(u8) + velocity(x, y, z) as float32.
_WIND = struct.Struct('<B3f')

SpawnCommand = namedtuple(
    'SpawnCommand', ['entity_id', 'type_code', 'position', 'quaternion'])
DespawnCommand = namedtuple('DespawnCommand', ['entity_id', 'type_code'])
# Deliberately have NO entity_id/type_code: type_code 0 is a real vehicle
# (x500), so a wind message carrying its header bytes would be mistaken
# for x500_0 by any entity-keyed path. Dispatch on the class instead.
WindCommand = namedtuple('WindCommand', ['enable', 'velocity'])
# "Tell me the current world wind" - answered with a MSG_WIND packet.
WindQuery = namedtuple('WindQuery', [])


def parse_packet(data: bytes):
    """Parse a wire packet into a command.

    Returns a SpawnCommand (MessageID=1), a DespawnCommand (MessageID=4), a
    WindCommand or WindQuery (MessageID=6), or None for other message ids.
    Raises ValueError on a malformed packet.
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
    if message_id == MSG_WIND:
        payload = data[HEADER_SIZE:]
        if len(payload) < _WIND.size:
            raise ValueError("MessageID=6 payload too short for wind")
        op, vx, vy, vz = _WIND.unpack(payload[:_WIND.size])
        if op == WIND_OP_QUERY:
            return WindQuery()
        if op == WIND_OP_OFF:
            # The vector means nothing when the wind is off: normalise it,
            # so the state reported back is unambiguous and a garbage
            # vector can never stop the OFF command from landing.
            return WindCommand(False, (0.0, 0.0, 0.0))
        if op != WIND_OP_ON:
            raise ValueError(f"MessageID=6 unknown wind op {op}")
        # A NaN/inf wind would poison every enable_wind link in the world
        # (and UE text boxes are free-form), so refuse it here.
        if not all(math.isfinite(v) for v in (vx, vy, vz)):
            raise ValueError("MessageID=6 wind velocity is not finite")
        return WindCommand(True, (vx, vy, vz))
    return None


def pack_pose(entity_id, type_code, position, quaternion) -> bytes:
    """Build a MessageID=1 pose packet (used by tests and golden vectors)."""
    return bytes([entity_id, type_code, MSG_POSE]) + _POSE.pack(
        *position, *quaternion)


def pack_destroy(entity_id, type_code) -> bytes:
    """Build a MessageID=4 destroy packet (used by tests)."""
    return bytes([entity_id, type_code, MSG_DESTROY])


def pack_wind(enable, velocity) -> bytes:
    """Build a MessageID=6 wind packet: the UE->manager ON/OFF command and,
    with the same layout, the manager->UE state answer."""
    return bytes([0, 0, MSG_WIND]) + _WIND.pack(
        WIND_OP_ON if enable else WIND_OP_OFF, *velocity)


def pack_wind_query() -> bytes:
    """Build a MessageID=6 QUERY packet (UE -> manager, vector unused)."""
    return bytes([0, 0, MSG_WIND]) + _WIND.pack(WIND_OP_QUERY, 0.0, 0.0, 0.0)


def triage_batch(items):
    """Classify one receive batch of (command, sender_ip) pairs.

    Returns (wind, askers, rest):
      wind   - the NEWEST WindCommand, or None. A world-level setting is
               idempotent, so earlier ones in the same batch are superseded.
      askers - sender IPs that sent a WindQuery, deduplicated, arrival order.
      rest   - the entity commands in arrival order, for the entity
               supersede/dispatch logic, which keys on entity_id/type_code
               that the wind messages deliberately do not have.
    None commands (packets the manager does not act on) are skipped.
    """
    wind = None
    askers = []
    rest = []
    for cmd, sender_ip in items:
        if isinstance(cmd, WindCommand):
            wind = cmd
        elif isinstance(cmd, WindQuery):
            if sender_ip not in askers:
                askers.append(sender_ip)
        elif cmd is not None:
            rest.append(cmd)
    return wind, askers, rest
