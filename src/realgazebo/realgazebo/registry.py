class VehicleRecord:
    def __init__(self, vehicle_type, vehicle_id):
        self.vehicle_type = vehicle_type
        self.vehicle_id = vehicle_id
        # backend-specific handle: subprocess.Popen in monolithic mode,
        # container id in docker mode; set right after backend.launch().
        # Stays None for static props (no autopilot stack to manage).
        self.handle = None
        # static prop/obstacle: skipped by the crash watcher and the
        # network_sim roster; movable at runtime via set_pose
        self.prop = False


class VehicleRegistry:
    """Single source of truth for active (type, id). v1: dedup + lookup only."""

    def __init__(self):
        self._records = {}

    def is_active(self, vehicle_type, vehicle_id):
        return (vehicle_type, vehicle_id) in self._records

    def add(self, vehicle_type, vehicle_id):
        key = (vehicle_type, vehicle_id)
        if key in self._records:
            raise ValueError(f"{vehicle_type}_{vehicle_id} already active")
        record = VehicleRecord(vehicle_type, vehicle_id)
        self._records[key] = record
        return record

    def remove(self, vehicle_type, vehicle_id):
        """Remove and return the record for (type, id), or None if not active."""
        return self._records.pop((vehicle_type, vehicle_id), None)

    def get(self, vehicle_type, vehicle_id):
        """Return the record for (type, id), or None if not active."""
        return self._records.get((vehicle_type, vehicle_id))

    def active_ids(self):
        return sorted(self._records.keys())

    def type_of(self, vehicle_id):
        """Type currently holding this numeric id, or None if the id is free.

        Numeric ids are globally unique on the wire (ROS namespace, MAVLink
        port, UE num all derive from it), so at most one type can hold one.
        Iterates a snapshot: callers read from the UDP listener thread while
        the crash watcher pops records on the executor thread, and a live
        dict iteration would raise if a reap lands mid-scan.
        """
        for vehicle_type, vid in list(self._records):
            if vid == vehicle_id:
                return vehicle_type
        return None
