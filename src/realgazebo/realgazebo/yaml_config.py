import ast
from dataclasses import dataclass


@dataclass
class VehicleSpec:
    vehicle_id: int
    vehicle_type: str
    build_target_path: str
    spawnpoint: tuple  # (x, y, z, yaw) floats
    # HITL fields (default => a normal SITL vehicle, unchanged behaviour):
    mode: str = 'sitl'          # 'sitl' (PX4 SITL process) or 'hitl' (real FC)
    fc_endpoint: dict = None    # HITL FC link: {'device','baud'} or {'udp','local_port'}
    sys_id: int = None          # HITL: MAVLink system id; None => vehicle_id + 1
    qgc_relay: bool = None      # HITL: relay FC<->QGC through the bridge;
                                # None => auto (serial yes, ethernet no)
    # Entity identity attached by the manager after code-map validation;
    # None straight out of parse_vehicles (the parser stays code-map-free).
    entity: object = None


def parse_vehicles(config: dict) -> list:
    """Convert YAML dict (px4_target + vehicles) into a sorted list[VehicleSpec].

    Entries without a build_target get build_target_path=None: static props
    (e.g. rock) never need one, HITL vehicles run a real FC (no SITL build),
    and a PX4 SITL vehicle missing it now fails its own spawn with a clear
    error instead of being silently skipped.

    HITL vehicles add `mode: hitl` plus an `fc:` block (device+baud or
    udp+local_port). PILS vehicles add `mode: pils` and nothing else: their
    PX4 SITL runs on a remote PC (scripts/run_pils_vehicle.sh) and attaches
    over gz-transport. Everything else (type, spawnpoint, the numeric id
    key) is identical to a SITL vehicle, so the id-derived identity (gz
    name, ROS namespace, UE num) is unchanged.
    """
    px4_targets = config.get('px4_target', {})
    vehicles = config.get('vehicles', {})
    specs = []
    for key in sorted(vehicles.keys()):
        v = vehicles[key]
        target = v.get('build_target')  # may legitimately be key 0
        raw = v['spawnpoint']
        point = ast.literal_eval(raw) if isinstance(raw, str) else raw
        specs.append(VehicleSpec(
            vehicle_id=int(key),
            vehicle_type=v['type'],
            build_target_path=px4_targets[target] if target is not None else None,
            spawnpoint=tuple(float(x) for x in point),
            mode=v.get('mode', 'sitl'),
            fc_endpoint=v.get('fc'),
            sys_id=v.get('sys_id'),
            qgc_relay=v.get('qgc_relay'),
        ))
    return specs
