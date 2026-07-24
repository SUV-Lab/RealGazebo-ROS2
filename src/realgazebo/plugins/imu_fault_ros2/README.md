# ImuFaultROS2

Injects per-channel faults into a vehicle's IMU so PX4's EKF2 consumes corrupted
accelerometer and gyro data. Built for reproducing published fault scenarios
(bias / drift / oscillation over a time window) against x500 and lc_62.

## How it works

Physics (system priority 0) fills the IMU **sensor entity's**
`components::LinearAcceleration` and `components::AngularVelocity` each step.
This plugin runs at priority 100 — after Physics, before the stock
`gz-sim-imu-system` reads those components in PostUpdate — and adds the fault.
The sensor then publishes on the topic PX4's `gz_bridge` hardcodes
(`/world/$W/model/$M/link/base_link/sensor/imu_sensor/imu`, GZBridge.cpp:81),
so **PX4 needs no modification at all**.

The `<sensor>` block is never touched. That matters for x500, whose IMU is
inherited from PX4's `x500_base` via `<include merge='true'>` — mutating runtime
ECM components works identically for an inherited sensor and a locally declared
one (lc_62).

### Failing open

If this plugin is missing, fails to load, or is disabled, the vehicle gets clean
IMU data and behaves exactly as it did before the plugin existed. Verified: with
the `.so` absent, Gazebo logs

```
[Err] [SystemLoader.cc:92] Failed to load system plugin [ImuFaultROS2] : Could not find shared library.
```

and continues loading the world; the IMU topic publishes normally. This is why
the ECM-mutation approach was chosen over a topic relay — a relay renames the
sensor in the SDF, so a stale or missing plugin would leave PX4's topic empty and
the whole fleet unable to arm.

When no fault is active the plugin **skips the component write entirely** rather
than adding 0.0, so a disabled plugin is bit-identical to no plugin.

## SDF

```xml
<plugin filename="ImuFaultROS2" name="custom::ImuFaultROS2System">
  <gz:system_priority>100</gz:system_priority>   <!-- required: must beat Physics -->
  <sensorName>imu_sensor</sensorName>            <!-- optional: first IMU child if omitted -->
  <frame>frd</frame>                             <!-- optional: frd (default) | flu -->
</plugin>
```

`<gz:system_priority>` is load-bearing. Dropping it, or setting it below
Physics, silently produces a fault-free run — the plugin still evaluates and
reports the fault on `applied`, but Physics overwrites the components before the
sensor reads them. That asymmetry is the counter-probe used to verify the
ordering actually works.

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/<model>/imu_fault/spec` | `realgazebo_msgs/ImuFaultSpec` | in — the real interface |
| `/<model>/imu_fault/commands` | `std_msgs/Float32MultiArray` | in — debug smoke test |
| `/<model>/imu_fault/applied` | `std_msgs/Float32MultiArray` | out — `[ax,ay,az,p,q,r]` FRD, ~10 Hz |

**Record `applied` on every experiment run.** ECM mutation leaves no clean/faulted
topic pair to diff, so this is the only evidence that a fault was actually live.
A silently fault-free run otherwise looks exactly like a successful baseline.

### Where to run the client

The plugin's ROS node lives in the **gzserver** process, not with PX4. Publish
from the gazebo service container in multi-container mode (the vehicle
containers sit on a different DDS partition and will not reach it — with no
error). In monolithic mode `--network host` makes the host work.

## Usage

```bash
# inject and hold — this process must stay alive
python3 scripts/inject_imu_fault.py --vehicle x500_0 \
    --scenario scripts/scenarios/imu_table2.yaml

# release
python3 scripts/inject_imu_fault.py --vehicle x500_0 --clear
```

### transient_local is not what it looks like

DDS keeps a latched sample in the **publishing process**. `ros2 topic pub --once`
exits immediately and therefore latches nothing, and a *volatile* publisher will
not even QoS-match the plugin's transient_local subscription — the message is
dropped with no error. So "publish before spawn, delivered on spawn" and
"re-inherited on respawn" only hold while `inject_imu_fault.py` is still running.

## Fault model

`tau = simTime - t_ref`; a channel is active for `t_start < tau < t_end`
(`t_end <= t_start` means no end). With `d = tau - t_start`:

| type | value |
|---|---|
| `bias` | `p0` |
| `drift` | `p0 * d` |
| `oscillation` | `p0 * sin(2*pi*p1*d + p2)` |

scaled by a linear fade-in `min(d/ramp_in, 1)` when `ramp_in > 0`.

`EvalFaultChannel` is a **pure function** of (channel, tau) — no state carried
between calls, so results do not depend on step size, dropped steps or resets.

### Window exit

- `snap` (default) — drops to zero instantly at `t_end`, matching a bounded
  fault window read literally (e.g. TABLE II's `10<t<30`).
- `hold_last` — latches the analytic value the waveform had at `t_end`, fade-in
  included, forever (permanent-degradation model). Deterministic: it does not
  depend on where the step boundary happened to land.

### Time reference

- `on_command` (default) — `t_ref` is the first tick after a spec is accepted.
  Use this: runtime spawn plus PX4 boot (~15 s) makes world-start timing
  unreproducible. Trigger it in stable hover and `10<t<30` reads as "10 s clean,
  then 20 s faulted".
- `on_spawn` — the plugin's first Update tick. That instant is recorded
  unconditionally, so a spec that arrives seconds later still anchors to the real
  spawn rather than to its own arrival. Careful: a `t_start` of 10 lands during
  PX4 boot, before EKF2 has converged.
- `sim_time_abs` — `t_ref = 0`.

`t_ref` is re-latched **only when `spec_seq` changes**, so a duplicate delivery
(transient_local resend) cannot silently restart the fault clock mid-experiment.
A repeated sequence updates the parameters and logs a `gzwarn` saying the clock
was *not* restarted. `inject_imu_fault.py` picks a fresh `spec_seq` per run by
default, so re-running it always restarts the window; pass `--seq` explicitly
only when you deliberately want to retune parameters without restarting.

On world reset the clock is cleared. `on_spawn` and `sim_time_abs` can both
re-derive their reference, so they re-latch on the next tick; `on_command` has no
way to know when the fault was meant to start and stays dormant until a new
`spec_seq` arrives. The reset log line says which of the two happened.

## Frames — the failure you will not see

Channels are specified in **FRD** by default (paper and PX4 convention). The ECM
components are in the sensor's FLU frame, and PX4's gz_bridge maps FLU→FRD with
`(x, y, z) -> (x, -y, -z)` (GZBridge.cpp:303), so the plugin applies the same
flip on the way in and the value arrives at EKF2 with the sign you asked for.

Get this wrong and `f_Ay`/`f_r` come out negated — visible — but `f_Az`/`f_q`
come out **phase-shifted by 180°**, which RMSE and FFT magnitude cannot detect.
Always run the sign check:

```bash
# FRD ay = +3.0 must show as y: -3 in the raw Gazebo IMU message
ros2 topic pub --once /<model>/imu_fault/commands std_msgs/msg/Float32MultiArray \
    "{data: [1,1,3.0,0,0,0,0]}"
```

## Debug channel

`/<model>/imu_fault/commands` takes rows of
`[channel, type, p0, p1, p2, t_start, t_end]` — a one-line smoke test that needs
no custom message build. The whole payload is validated into a staging buffer
before any state is touched, so a bad length, an out-of-range channel or an
unimplemented fault type is genuinely rejected whole and leaves a running
experiment untouched.

`frame` and `time_ref` cannot be set here: the path *forces* FRD + `on_command`
+ `snap` on every message. Forcing the frame matters — the sign check above
runs through this topic, and a leftover FLU from an earlier spec would silently
invert what that check proves. This path also leaves `spec_seq` alone, so a smoke
test can never make a later real spec look like a duplicate.

## Extending

`plugins/fault_core/FaultChannelModel.hpp` is header-only and free of ROS and
Gazebo types, so it unit-tests without a simulator
(`test/test_fault_channel_model.cpp`) and ports unchanged to a different
injection backend.

Non-additive faults (stuck, scale, dropout, quantization) cannot be expressed as
an additive term on a pre-noise component and need the message-relay backend —
see section 8 of `docs/imu_fault_injection-plan.md`. Type codes 4..11 are
reserved for them.
