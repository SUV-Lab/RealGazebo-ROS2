# HITL: driving a real flight controller from RealGazebo

A vehicle with `mode: hitl` in the fleet YAML has **no PX4 SITL process**.
The manager creates its gz model and starts `gz-hitl-bridge` (from
RealGazebo-PX4) as a manager-local subprocess. The bridge relays MAVLink
HIL between the shared gz world and a real flight controller over serial
or Ethernet, and forwards FC telemetry to QGC:

```
gz world ──sensors──▶ gz-hitl-bridge ──HIL_SENSOR/HIL_GPS──▶ real FC
gz motors ◀─commands── gz-hitl-bridge ◀─HIL_ACTUATOR_CONTROLS─ real FC
                          │
                          └──GcsRelay──▶ QGC (mavlink_gcs_ip:14550)
```

Example fleet: `src/realgazebo/yaml/hitl_one_drone.yaml`. HITL and SITL
vehicles mix freely in one fleet; the numeric id keeps its usual meaning
(gz name `x500_0`, ROS namespace `/vehicle1`, UE instance).

## FC preparation (once per FC)

1. **Firmware**: build and flash the HITL firmware — it bakes in
   `pwm_out_sim` so the HIL airframe works on real hardware:
   ```
   make px4_fmu-v6x_hitl          # in RealGazebo-PX4
   make px4_fmu-v6x_hitl upload   # flash over USB
   ```
   The `px4_fmu-v6x_hitl` board target exists on the RealGazebo-PX4
   branch that ships `gz_hitl_bridge` (`boards/px4/fmu-v6x/hitl.px4board`).
2. **Airframe**: select *HIL Quadcopter X* (1001). This sets `SYS_HITL=1`.
   ⚠️ Changing the airframe RESETS parameters — do it BEFORE step 3.
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
carries HIL **and** telemetry, so the bridge's GcsRelay is the only way
QGC sees the vehicle — QGC must NOT hold the serial port itself (close
its serial connection; also `systemctl stop ModemManager` if the link
flaps, and `chmod 666 /dev/ttyACM0` on permission errors).

## Ethernet FC — canonical topology (14550 / 14560 split)

The FC's **default** ethernet MAVLink instance (UDP 14550, broadcast on)
stays reserved for QGC. HIL runs on a **dedicated second instance at
14560**, so QGC and the bridge can never steal each other's traffic:

```
        ┌────────── FC 10.41.10.2 ──────────┐
QGC ◀──▶ instance @14550 (broadcast, QGC)   │
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
param set MAV_1_UDP_PRT 14560
param set MAV_1_RATE 100000     # default is 1200 B/s (radio-sized) - the
                                # FC->bridge stream starves without this
param save
reboot
# verify: `mavlink status` now lists a UDP (14560, ...) instance with
# `tx rate max: 100000`; MAV_1_BROADCAST must stay 0 (a broadcasting HIL
# instance would be discovered and stolen by any QGC on the subnet)
```

Host side: put the sim PC on the FC's subnet (PX4's DHCP-fallback
static address is `10.41.10.2/24`):

```bash
sudo ip addr add 10.41.10.1/24 dev <iface>    # persist via NetworkManager
ping 10.41.10.2
```

YAML:

```yaml
fc :
  udp : 10.41.10.2:14560
  local_port : 14540      # unique per HITL vehicle (14540 + id)
```

QGC connects **directly** to the FC over ethernet (broadcast
auto-connect), independent of the bridge, so the bridge's FC↔QGC relay
is **off** for Ethernet links: `build_hitl_command` only passes `--qgc`
when it is the only path to QGC (serial), and the bridge opens the relay
only when that flag is present. Override per vehicle with `qgc_relay:`:

```yaml
qgc_relay : true     # force the relay on (ethernet FC with no GCS instance)
qgc_relay : false    # force it off
```

Without this the relay duplicated every FC message onto loopback — 1219
pps of redundant traffic next to the FC's own 244 pps direct feed, since
the HIL instance runs at a much higher rate than the QGC one.

The rule predates us — Gazebo Classic's `gazebo_mavlink_interface`
reached the same conclusion (`sitl_gazebo-classic/src/mavlink_interface.cpp`):

|                | Gazebo Classic                      | RealGazebo                       |
| -------------- | ----------------------------------- | -------------------------------- |
| decided in     | the plugin (`hil_mode_ && serial_enabled_`) | the manager (passes `--qgc` or not) |
| serial FC      | relay ON                            | relay ON                         |
| Ethernet FC    | relay OFF                           | relay OFF                        |
| SITL           | sockets never opened (`if (hil_mode_)`) | n/a (no bridge)              |
| override       | none (hardcoded)                    | `qgc_relay:`                     |

## System ids

The bridge stamps a MAVLink system id on everything it sends to the FC
(HIL_SENSOR, HIL_GPS, its 1 Hz heartbeat), all under component id 200.
That id defaults to **the vehicle's YAML key + 1** — PX4's own
convention, since SITL's `rcS` sets `MAV_SYS_ID = instance + 1` and the
ROS namespace is `/vehicle{key+1}`. So YAML key `0` transmits as system
`1`, matching a stock FC's default `MAV_SYS_ID`.

Set `sys_id:` on the vehicle only when the FC's `MAV_SYS_ID` is
something else:

```yaml
  0 :
    type : x500
    mode : hitl
    sys_id : 42       # this FC's MAV_SYS_ID is 42, not 1
```

**The bridge refuses to run on a mismatch.** It reads the FC's id from
the FC's own heartbeat and exits with an error naming both values, so a
wrong `MAV_SYS_ID` is caught at startup instead of surfacing later as
odd behaviour: QGC would list the bridge and the FC as two systems, and
on the `MAV_USEHILGPS` path `mavlink_receiver.cpp` requires
`msg->sysid == mavlink_system.sysid` and silently drops HIL_GPS
otherwise. (Only autopilot heartbeats are checked, so a GCS heartbeat
forwarded by an instance with `MAV_x_FORWARD` on is not mistaken for
the FC.)

In a multi-HITL fleet each vehicle needs its own sysid, which the
`key + 1` default already provides; without it every bridge would
transmit as system 1 and a GCS would merge them into one vehicle.

## Pitfalls (each of these was hit for real)

- **`local_port` must NEVER be 14550.** The sim container runs with
  `--network host`, so the bridge's FC socket would occupy the host's
  QGC port and receive its own GcsRelay output — a self-feeding packet
  storm: bridge CPU pegged, seq-gap floods in the FC's `mavlink status`,
  QGC never connects.
- **Don't point the bridge at the FC's 14550 instance** (the shortcut of
  reusing the default instance works, but then QGC's broadcast
  auto-connect and the bridge fight over the instance's single UDP
  partner — the HIL stream gets stolen mid-run the moment any QGC on
  the subnet answers a broadcast). If you must use it, disable that
  instance's broadcast (`MAV_x_BROADCAST 0`).
- **A one-time `lost` offset (0–255) after a bridge restart or FC reboot
  is normal**: the FC's per-component stats cannot distinguish a sender
  restart from packet loss. It must stop growing afterwards; steady
  growth of exactly ±255 per second was a bridge seq bug, fixed in
  RealGazebo-PX4 (`fix(gz_hitl_bridge): single tx seq counter`).
- **`ping` proving the link does not prove HIL**: an instance must be
  LISTENING on the port the YAML names, or the packets are silently
  dropped. `mavlink status` on the FC is the ground truth — the 14560
  instance's `rx` rate is nonzero when bridge packets arrive.

## Verifying a run

1. Manager log: `spawned hitl x500_0`.
2. FC `mavlink status`: the HIL instance shows `rx` ≈ 20 KB/s
   (HIL_SENSOR ~250 Hz, HIL_GPS ~30 Hz) and single-digit `rx loss`.
3. QGC shows the vehicle; preflight clean after a reboot-and-settle
   (reboot the FC once with the bridge already streaming so the EKF
   initializes on sim data).
4. Arm (QGC slider, or `commander arm` in nsh) → rotors spin in gz/UE.
   The bridge zeroes motor velocities while disarmed.
