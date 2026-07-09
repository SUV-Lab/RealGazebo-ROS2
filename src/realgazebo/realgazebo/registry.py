class EntityRecord:
    """Mutable runtime state for one active entity."""

    def __init__(self, entity):
        self.entity = entity
        # backend-specific handle: subprocess.Popen in monolithic mode,
        # container id in docker mode; set right after backend.launch().
        # Stays None for static props (no autopilot stack to manage).
        self.handle = None
        # which backend materialized this vehicle (SubprocessBackend /
        # DockerBackend for SITL, HitlBackend for HITL). The crash watcher
        # and despawn call alive()/kill() on THIS backend, so a mixed
        # HITL+SITL fleet reaps each vehicle through its own backend.
        # Stays None for props.
        self.backend = None

    @property
    def prop(self):
        # static prop/obstacle: skipped by the crash watcher and the
        # network_sim roster; movable at runtime via set_pose. Derived
        # from the entity's wire code so it cannot drift.
        return self.entity.is_prop


class EntityRegistry:
    """Single source of truth for active entities, keyed by (type, id)."""

    def __init__(self):
        self._records = {}

    def is_active(self, entity_type, entity_id):
        return (entity_type, entity_id) in self._records

    def add(self, entity):
        key = (entity.type, entity.id)
        if key in self._records:
            raise ValueError(f"{entity.name} already active")
        record = EntityRecord(entity)
        self._records[key] = record
        return record

    def remove(self, entity_type, entity_id):
        """Remove and return the record for (type, id), or None if not active."""
        return self._records.pop((entity_type, entity_id), None)

    def get(self, entity_type, entity_id):
        """Return the record for (type, id), or None if not active."""
        return self._records.get((entity_type, entity_id))

    def active_ids(self):
        return sorted(self._records.keys())

    def records(self):
        """Snapshot of active records, sorted by (type, id)."""
        return [self._records[k] for k in sorted(self._records)]

    def type_of(self, entity_id):
        """Type currently holding this numeric id, or None if the id is free.

        Numeric ids are globally unique on the wire (ROS namespace, MAVLink
        port, UE instance all derive from it), so at most one type can hold
        one. Iterates a snapshot: callers read from the UDP listener thread
        while the crash watcher pops records on the executor thread, and a
        live dict iteration would raise if a reap lands mid-scan.
        """
        for entity_type, vid in list(self._records):
            if vid == entity_id:
                return entity_type
        return None
