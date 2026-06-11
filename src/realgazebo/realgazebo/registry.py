class VehicleRecord:
    def __init__(self, vehicle_type, vehicle_id):
        self.vehicle_type = vehicle_type
        self.vehicle_id = vehicle_id
        # backend-specific handle: subprocess.Popen in monolithic mode,
        # container id in docker mode; set right after backend.launch()
        self.handle = None


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
