import ast
from dataclasses import dataclass


@dataclass
class VehicleSpec:
    vehicle_id: int
    vehicle_type: str
    build_target_path: str
    spawnpoint: tuple  # (x, y, z, yaw) floats
    # Entity identity attached by the manager after code-map validation;
    # None straight out of parse_vehicles (the parser stays code-map-free).
    entity: object = None


def parse_vehicles(config: dict) -> list:
    """Convert YAML dict (px4_target + vehicles) into a sorted list[VehicleSpec].

    Entries without a build_target get build_target_path=None: static props
    (e.g. rock) never need one, and a PX4 vehicle missing it now fails its
    own spawn with a clear error instead of being silently skipped.
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
        ))
    return specs
