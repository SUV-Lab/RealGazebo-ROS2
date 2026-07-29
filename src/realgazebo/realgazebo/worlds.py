"""Resolving `world:=<name>` to a world SDF, and the one rule that holds it together.

The gz world NAME - not the file name - is what every gz topic and service is
namespaced under (/world/<name>/create, /world/<name>/remove, /world/<name>/
model/.../sensor/...). The manager, PX4 (PX4_GZ_WORLD), the sensor bridges and
network_sim all address that name as a bare string, and none of them reads it
back off the running server.

So the launch half and the manager half must derive the name the same way or
they address two different worlds and only some of the traffic lands. That is
not hypothetical: `world:=urban` used to load worlds/c-track.sdf (hardcoded)
while the manager addressed /world/urban/*, which left vehicles spawning fine
- the vehicle container hardcoded 'c-track' too - while every despawn timed
out and was swallowed by `check=False`, so containers died and their gz models
lived on, logged as "despawned".

The rule that prevents all of it: **a world's name equals its file name**.
worlds/<name>.sdf declares <world name='<name>'>. Checked once, at launch, by
resolve_world_file(). Adding a world is dropping a file in that directory -
nothing here has to be taught its name.

Not to be confused with `terrain:=`, which only picks which STL the shipped
c-track terrain model shows and never touches the world name.
"""

import glob
import os
import xml.etree.ElementTree as ET


def declared_world_name(world_file_path):
    """Return the <world name=...> declared inside an SDF, or None.

    None means "could not tell" (unparseable, unreadable, or no <world>
    element), which callers treat as "cannot check" rather than "mismatch".
    """
    try:
        root = ET.parse(world_file_path).getroot()
    except (ET.ParseError, OSError):
        return None
    node = root.find('world')
    if node is None:
        return None
    return node.get('name')


def available_worlds(package_share_path):
    """Names of every world SDF shipped/installed in <share>/worlds."""
    return sorted(
        os.path.basename(p)[:-len('.sdf')]
        for p in glob.glob(os.path.join(package_share_path, 'worlds', '*.sdf')))


def resolve_world_file(package_share_path, world):
    """Map `world:=<name>` to <share>/worlds/<name>.sdf.

    Raises RuntimeError if the file is missing, or if it declares a
    <world name=> other than <name> - see the module docstring for why that
    mismatch must not be allowed to reach a running simulation.
    """
    world_file_path = os.path.join(package_share_path, 'worlds', f'{world}.sdf')
    if not os.path.exists(world_file_path):
        available = ', '.join(available_worlds(package_share_path)) or '(none)'
        raise RuntimeError(
            f"world '{world}' not found: no {world_file_path}. "
            f'Available worlds: {available}. '
            f'(To show a different crop of the c-track terrain, pass '
            f'terrain:= instead - it does not select a world.)')

    declared = declared_world_name(world_file_path)
    if declared is not None and declared != world:
        raise RuntimeError(
            f"{world}.sdf declares <world name='{declared}'>, but a world's "
            f'name must equal its file name. Either rename the file to '
            f"{declared}.sdf or change the SDF to <world name='{world}'>.")
    return world_file_path
