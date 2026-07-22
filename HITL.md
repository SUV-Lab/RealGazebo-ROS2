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
auto-connect), independent of the bridge. The GcsRelay still forwards a
second copy of the telemetry; QGC treats it as an extra link to the same
vehicle (harmless; a relay on/off switch is planned).

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
