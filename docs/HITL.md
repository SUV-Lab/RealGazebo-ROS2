# HITL: driving a real flight controller from RealGazebo

A vehicle with `mode: hitl` in the fleet YAML has **no PX4 SITL process**.
The manager creates its gz model and starts `gz-hitl-bridge` (from
RealGazebo-PX4), which relays MAVLink HIL between the shared gz world and
a real flight controller over serial or Ethernet:

```
gz world ──sensors──▶ gz-hitl-bridge ──HIL_SENSOR/HIL_GPS──▶ real FC
gz motors ◀─commands── gz-hitl-bridge ◀─HIL_ACTUATOR_CONTROLS─ real FC
```

Example fleet: `src/realgazebo/yaml/hitl.yaml` (serial and Ethernet
side by side). HITL and SITL vehicles mix freely in one fleet; the
numeric id keeps its usual meaning (gz name `x500_0`, ROS namespace
`/vehicle1`, UE instance).

## FC preparation (once per FC)

1. **Firmware**: build and flash the HITL firmware:
   ```
   make px4_fmu-v6x_hitl          # in RealGazebo-PX4
   make px4_fmu-v6x_hitl upload   # flash over USB
   ```
2. **Airframe**: pick the one matching your gz model. Selecting an
   airframe sets `SYS_HITL=1` and the HIL channel layout.
   ⚠️ Changing the airframe RESETS parameters — do it BEFORE step 3.

   | gz model | airframe | note |
   |---|---|---|
   | `lc_62` | **HIL lc_62** (1003) | complete, nothing extra to set |
   | `rover_ackermann` | **HIL rover_ackermann** (1004) | complete |
   | `x500` (plain quad) | *HIL Quadcopter X* (1001, stock PX4) | also set the three bench params by hand: `COM_RCL_EXCEPT=31`, `COM_RC_IN_MODE=4`, `NAV_DLL_ACT=0` |

3. **MAVLink instances** (Ethernet only — see topology below).
4. Reflashing firmware later does NOT clear parameters; steps 2–3 are
   one-time unless you change airframes again.

## Serial FC (USB) — zero FC-side config

```yaml
fc :
  device : /dev/ttyACM0
  baud : 921600        # nominal; USB CDC ignores the baud value
```

The FC's USB MAVLink instance is automatic. The single serial pipe
carries HIL **and** telemetry, so the bridge relays FC↔QGC itself — QGC
must NOT hold the serial port (close its serial connection; also
`systemctl stop ModemManager` if the link flaps, and
`chmod 666 /dev/ttyACM0` on permission errors).

## Ethernet FC — canonical topology (14550 / 14560 split)

The FC's **default** ethernet MAVLink instance (UDP 14550) stays
reserved for QGC. HIL runs on a **dedicated second instance at 14560**,
so QGC and the bridge can never steal each other's traffic:

```
        ┌────────── FC 10.41.10.11 ─────────┐
QGC ◀──▶ instance @14550 (QGC)              │
bridge ◀▶ instance @14560 (HIL, dedicated)  │
        └────────────────────────────────────┘
```

FC side (nsh or QGC params; MAV_1 assumed free — check
`param show MAV_*_CONFIG` first):

```
param set MAV_1_CONFIG 1000     # Ethernet
param save
reboot
# after reboot the sub-params exist:
param set MAV_1_UDP_PRT 14560   # default is 0 = no port; REQUIRED
param set MAV_1_RATE 100000     # default is 1200 B/s (radio-sized) - the
                                # FC->bridge stream starves without this
param save
reboot
# verify: `mavlink status` now lists a UDP (14560, ...) instance with
# `tx rate max: 100000`. MAV_1_BROADCAST must stay 0.
```

Host side: put the sim PC on the FC's subnet and check reachability
(`ping <fc-ip>`). YAML:

```yaml
fc :
  udp : 10.41.10.11:14560
  # local_port : 14600    # optional; defaults to 14600 + the vehicle key
```

The bridge binds `14600 + key` and `connect()`s it to the FC, so stray
traffic from other hosts cannot steer the HIL stream. The 146xx range is
free in PX4's port map, unlike 145xx.

QGC connects **directly** to the FC over ethernet, independent of the
bridge, so the bridge's FC↔QGC relay is automatically **on for serial,
off for Ethernet**. Override per vehicle when needed:

```yaml
qgc_relay : true     # force the relay on (ethernet FC with no QGC instance)
qgc_relay : false    # force it off
```

## System ids

The bridge transmits with system id **vehicle key + 1** (PX4's own
instance+1 convention, matching the `/vehicle{key+1}` namespace). Key 0
transmits as system 1, which matches a stock FC's default `MAV_SYS_ID`.
In a fleet, set each FC's `MAV_SYS_ID` to its key+1.

If an FC deliberately uses another id, mirror it in the YAML:

```yaml
  0 :
    type : x500
    mode : hitl
    sys_id : 42       # this FC's MAV_SYS_ID is 42, not 1
```

**The bridge refuses to run on a mismatch** — it reads the FC's real id
from its heartbeat and exits with an error naming both values, so a
wrong `MAV_SYS_ID` is caught at startup instead of surfacing as two
vehicles in QGC or silently dropped HIL_GPS.

## Multi-vehicle HITL (fleet)

Several real FCs driving several gz models at once. One flat subnet
carries everything — there is no separate FC↔companion link. Address by
vehicle number N (N = YAML key + 1):

| role                             | IP           |
|----------------------------------|--------------|
| sim host / all bridges           | 10.41.10.1   |
| GCS (QGC)                        | 10.41.10.2   |
| vehicle N — FC                   | 10.41.10.N1  |
| vehicle N — companion (Jetson)   | 10.41.10.N2  |

Everything else is derived from the id: sysid `= key+1`, ROS namespace
`/vehicle{key+1}`, bridge `local_port = 14600 + key`. Nothing is
per-vehicle except the FC/companion IPs.

Per-FC parameters — only two values differ across the fleet:

```sh
param set MAV_SYS_ID <N>              # must equal the bridge's key+1
param set MAV_1_CONFIG 1000           # HIL instance -> sim :14560
param set MAV_1_UDP_PRT 14560
param set MAV_1_RATE 100000
param set MAV_2_BROADCAST 0           # see below
param set UXRCE_DDS_CFG 1000          # uXRCE over Ethernet
param set UXRCE_DDS_AG_IP <int .N2>   # this vehicle's OWN companion, NOT the sim
param set UXRCE_DDS_PTCFG 0           # plain agent (no participant profile)
param set UXRCE_DDS_SYNCT 0           # REQUIRED: QGC's SYSTEM_TIME and the
                                      # uXRCE timesync otherwise fight over
                                      # the clock and break the session
```

`UXRCE_DDS_AG_IP` is a 32-bit int `(A<<24)|(B<<16)|(C<<8)|D`:
`.12`=170461708, `.22`=170461718, `.32`=170461728.

**ROS2 namespacing is baked into the firmware.** The RealGazebo-PX4
`uxrce_dds_client` derives the namespace `vehicle{MAV_SYS_ID}` when no
`-n` is given, so hardware matches SITL automatically — flash this
firmware on every board. Verify from the sim host: `ros2 topic list`
shows `/vehicle1/... /vehicle2/...` with no bare `/fmu/...` set (a stray
`/fmu/...` = one board on old firmware). The firmware ships a trimmed
topic map (~23 topics: RealGazebo consumers, classic offboard, nav
basics, px4_ros2 external modes); the full 52-topic map makes fleet
reconnects slow enough to destabilise each other — don't restore it
without re-testing.

**Each FC's uXRCE agent lives on ITS OWN companion** (`UXRCE_DDS_AG_IP`
→ `.N2`, not the sim). Run the standalone `MicroXRCEAgent` (bundles
FastDDS; **ROS2 is NOT required on the companion**) from a systemd unit,
quiet, matching PX4's DDS line (v2.4.3 for DDS v2):

```ini
# /etc/systemd/system/uxrce-agent.service
[Unit]
After=network.target
[Service]
ExecStart=/usr/local/bin/MicroXRCEAgent udp4 -p 8888 -v2
Restart=always
[Install]
WantedBy=multi-user.target
```

`-v2` matters: the default verbosity logs every entity operation, and
during a fleet reconnect that log flood can stall the agent long enough
to drop sessions.

**Set `MAV_2_BROADCAST 0` on every FC** and add one QGC UDP link per FC
(`10.41.10.N1:14550`). With broadcast on, every FC hears the others'
14550 broadcasts and latches onto a *peer FC* as its "GCS partner".

**Expected behaviour:** brief uxrce disconnects during boot bursts or a
sim restart are normal — the session reconnects and re-creates its
topics within ~1 s. What should NOT happen is continuous flapping; if
topic counts keep oscillating for minutes, an agent is being overloaded
(check its verbosity and the topic-map size).

## Pitfalls (each of these was hit for real)

- **`local_port` must NEVER be 14550.** The sim container runs with
  `--network host`; the bridge would occupy the QGC port and receive its
  own relay output — a self-feeding packet storm.
- **Don't point the bridge at the FC's 14550 instance.** QGC's
  auto-connect and the bridge would fight over the instance's single UDP
  partner and the HIL stream gets stolen mid-run.
- **A one-time `lost` offset (0–255) after a bridge restart or FC reboot
  is normal** — the FC cannot distinguish a sender restart from packet
  loss. It must stop growing afterwards.
- **`ping` proving the link does not prove HIL**: an instance must be
  LISTENING on the port the YAML names. `mavlink status` on the FC is
  the ground truth — the 14560 instance's `rx` rate is nonzero when
  bridge packets arrive.
- **A reflash + airframe re-select RESETS parameters** — re-apply the
  per-FC block afterwards (MAV_1, uXRCE, MAV_2_BROADCAST).

## Verifying a run

1. Manager log: `spawned hitl <model>_<id>`.
2. FC `mavlink status`: the HIL instance shows `rx` ≈ 20 KB/s
   (HIL_SENSOR ~250 Hz, HIL_GPS ~30 Hz) and single-digit `rx loss`.
3. QGC shows the vehicle; preflight clean after a reboot-and-settle
   (reboot the FC once with the bridge already streaming so the EKF
   initializes on sim data).
4. Arm (QGC slider, or `commander arm` in nsh) → rotors spin in gz/UE.
   The bridge zeroes motor velocities while disarmed.
