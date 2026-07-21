import os
import re
from glob import glob

# Wire convention: type codes 0..199 are PX4 vehicles (full autopilot
# stack), codes >= 200 are static props/obstacles - just a gz entity, no
# PX4, no per-vehicle container, and movable at runtime via repeated
# MSG_POSE. Matches the existing numbering (201 = rock).
PROP_CODE_MIN = 200

_CODE_RE = re.compile(r'<type_code>\s*(\d+)\s*</type_code>')


def scan_type_codes(models_dir=None) -> dict:
    """Build the code->type map by scanning <type_code> in *.sdf.jinja.

    The model template is the SINGLE source of truth: the gz plugin reads
    the same element at runtime, so adding an entity type needs no code
    changes on either side. The type name is the template filename stem.
    When two templates share a code (x500 / x500_lidar_2d both send 0),
    the shortest name wins — the base type is what a spawn request should
    produce. Raises RuntimeError when no template declares a code: a
    checkout with missing/stale templates must fail loudly at manager
    startup, not limp along on a silently divergent hardcoded map.
    """
    if models_dir is None:
        # lazy import so this module imports/unit-tests on a host without ROS
        from ament_index_python.packages import get_package_share_directory
        models_dir = os.path.join(get_package_share_directory('realgazebo'), 'models')
    mapping = {}
    pattern = os.path.join(models_dir, '**', '*.sdf.jinja')
    for path in sorted(glob(pattern, recursive=True)):
        with open(path) as f:
            match = _CODE_RE.search(f.read())
        if not match:
            continue
        code = int(match.group(1))
        name = os.path.basename(path)[:-len('.sdf.jinja')]
        if code not in mapping or len(name) < len(mapping[code]):
            mapping[code] = name
    if not mapping:
        raise RuntimeError(
            f"no <type_code> declarations found under {models_dir}; "
            f"the model templates are the single source of truth")
    return mapping


def type_for_code(code: int, mapping: dict) -> str:
    """Map a wire type_code to an entity type string; raise on unknown."""
    if code not in mapping:
        raise ValueError(f"unknown type_code {code}")
    return mapping[code]


def code_for_type(entity_type: str, mapping: dict) -> int:
    """Reverse lookup: wire code for a type string; raise on unknown."""
    for code, name in mapping.items():
        if name == entity_type:
            return code
    raise ValueError(f"unknown entity type '{entity_type}'")


def is_prop_code(code: int) -> bool:
    """True for static props/obstacles (no autopilot stack)."""
    return code >= PROP_CODE_MIN
