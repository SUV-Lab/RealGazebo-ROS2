# Canonical wire vehicle_code -> type map.
# MUST stay in sync with the gz plugin getVehicleCode() in
# src/realgazebo/plugins/realgazebo/RealGazebo.cpp and the UE
# FBridgeVehicleConfigRow DataTable. (code 0 covers both x500 and
# x500_lidar_2d on the sender side; the reverse maps to the base x500.)
CODE_TO_TYPE = {
    0: 'x500',
    1: 'rover_ackermann',
    2: 'boat',
    3: 'lc_62',
    4: 'ugv_kimm',
    201: 'rock',
}


def type_for_code(code: int) -> str:
    """Map a wire vehicle_code to a vehicle type string; raise on unknown."""
    if code not in CODE_TO_TYPE:
        raise ValueError(f"unknown vehicle_code {code}")
    return CODE_TO_TYPE[code]
