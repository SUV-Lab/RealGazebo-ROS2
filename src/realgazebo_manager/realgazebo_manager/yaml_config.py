import ast
from dataclasses import dataclass


@dataclass
class VehicleSpec:
    vehicle_id: int
    vehicle_type: str
    build_target_path: str
    spawnpoint: tuple  # (x, y, z, yaw) floats


def parse_vehicles(config: dict) -> list:
    """Convert YAML dict (px4_target + vehicles) into a sorted list[VehicleSpec].

    Obstacles (e.g. rock) are out of v1 scope, so entries without a
    build_target are skipped.
    """
    px4_targets = config.get('px4_target', {})
    vehicles = config.get('vehicles', {})
    specs = []
    for key in sorted(vehicles.keys()):
        v = vehicles[key]
        if 'build_target' not in v:
            continue
        raw = v['spawnpoint']
        point = ast.literal_eval(raw) if isinstance(raw, str) else raw
        specs.append(VehicleSpec(
            vehicle_id=int(key),
            vehicle_type=v['type'],
            build_target_path=px4_targets[v['build_target']],
            spawnpoint=tuple(float(x) for x in point),
        ))
    return specs
