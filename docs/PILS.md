# PILS: running a vehicle's PX4 on another PC

A vehicle with `mode: pils` in the fleet YAML has no local PX4 process and
no FC link: its **PX4 SITL runs on a separate PC** and attaches to the
shared gz world over gz-transport. It is the software twin of a HITL pair —
the remote PX4 plays the FC, and the same PC hosts the companion side
(uXRCE agent + mission), exactly like a Jetson next to a real FC:

```
sim PC                                    PILS PC
├─ gz world + manager                     ├─ docker: PX4 SITL (gz-transport attach)
│   mode: pils → spawns the model only    ├─ MicroXRCEAgent (same container)
└─ UE / QGC relay targets                 └─ mission (px4_ros2, local DDS)
```

SITL, HITL and PILS vehicles mix freely in one fleet; the numeric id keeps
its usual meaning (gz name `boat_3`, ROS namespace `/vehicle4`, UE
instance, sysid = key + 1).

## Sim PC side

1. Declare the vehicle with `mode: pils` and nothing else:

   ```yaml
     3 :
       type : boat
       mode : pils
       spawnpoint : (-281.27, -235.26, 8.5, -1.57)
   ```

2. Start the sim advertising a LAN address (the default `127.0.0.1` keeps
   the world invisible to other hosts):

   ```sh
   GZ_IP=10.41.10.1 scripts/run_realgazebo.sh fleet.yaml
   ```

The manager creates the gz model and its extras (camera receivers, lidar
bridge) and nothing else — like HITL, there is no local autopilot to
watch, and the remote peer may restart freely.

## PILS PC side

One-time prep: docker + the fleet image (`aware4docker/realgazebo:1.3-rc1`).
Then one line per vehicle, **after** the sim is up (the container waits up
to 30 s for the world, then gives up):

```sh
scripts/run_pils_vehicle.sh <type> <id> <sim_ip> [qgc_ip] [world]
# e.g. the boat above, QGC on the GCS PC:
./run_pils_vehicle.sh boat 3 10.41.10.1 10.41.10.2
```

The script runs one host-networked container that:
- joins the fleet's gz-transport graph (`GZ_PARTITION=realgazebo`,
  `GZ_IP` auto-detected from the route toward the sim PC),
- attaches PX4 to the existing model (`PX4_GZ_MODEL_NAME=<type>_<id>` —
  it never spawns its own),
- resolves the airframe from the `{id}_gz_{type}` SITL airframe files,
- starts its **own uXRCE agent** (docker-mode style, quiet `-v2`); the
  ROS namespace is `vehicle{id+1}`, matching SITL/HITL,
- beacons MAVLink at `qgc_ip:14550` (default: the sim PC).

## QGC

Nothing to configure: unlike a HITL FC (which listens and needs a per-FC
QGC link), SITL **actively sends** to `qgc_ip` (`-t` target in the rcS
GCS link). Run QGC on that host and the vehicle appears on the default
UDP link. Pick the host per deployment via the script's 4th argument.

## Verifying a run

1. Manager log: `spawned pils <type>_<id> (handle None)`.
2. PILS PC: `docker logs pils_<type>_<id>` shows
   `Gazebo world is ready` → `[gz_bridge] world: ..., model: ...` →
   `Ready for takeoff!`.
3. Actuator wiring (sim PC):
   `GZ_PARTITION=realgazebo GZ_IP=<sim ip> gz topic -i -t /<type>_<id>/command/motor_speed`
   must list a publisher on the PILS PC and subscribers on the sim PC.
   ⚠️ the similarly named `/model/<type>_<id>/command/motor_speed` is a
   PX4-internal twin — do not judge by that one.
4. `ros2 topic list` (either host, same LAN): `/vehicle{id+1}/...`.

## Pitfalls

- **Start order matters**: sim first, PILS second. The world-wait times
  out after ~30 s and the container exits (`--rm`, so it also disappears).
- **`GZ_IP` on BOTH sides.** The sim must advertise its LAN address and
  each PILS container its own (the script handles its side). Without it,
  gz-transport advertises unreachable addresses and discovery silently
  finds nothing.
- The shared partition is pinned to `realgazebo` by
  `manager_sim.launch.py`; host-side gz CLI debugging needs
  `GZ_PARTITION=realgazebo` exported to see anything.
- Multiple PILS vehicles on one PC share the single agent port 8888 —
  fine for one vehicle per PC; give additional agents distinct ports (and
  matching `PX4_UXRCE_DDS_PORT`) if you stack several.
