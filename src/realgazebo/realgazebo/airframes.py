import os


def scan_airframes(px4_build_path: str) -> dict:
    """Scan {px4}/ROMFS/.../airframes for '{id}_gz_{type}' files -> {type: id}."""
    airframes_dir = os.path.join(
        px4_build_path, "ROMFS/px4fmu_common/init.d-posix/airframes")
    mapping = {}
    if not os.path.isdir(airframes_dir):
        return mapping
    for filename in os.listdir(airframes_dir):
        if filename[:1].isdigit() and '_gz_' in filename:
            autostart_id, _, vehicle_type = filename.partition('_gz_')
            mapping[vehicle_type] = autostart_id
    return mapping


def get_autostart_id(vehicle_type: str, px4_build_path: str) -> str:
    mapping = scan_airframes(px4_build_path)
    if vehicle_type not in mapping:
        raise ValueError(
            f"No airframe for vehicle type '{vehicle_type}' in {px4_build_path}")
    return mapping[vehicle_type]
