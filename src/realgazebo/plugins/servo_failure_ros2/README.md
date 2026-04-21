# Servo Failure System Plugin

This Gazebo plugin simulates control-surface (servo) failures on fixed-wing
and VTOL vehicles by intercepting the force command that
`JointPositionController` writes each simulation step.

## Overview

The plugin runs AFTER `gz-sim-joint-position-controller-system` in PreUpdate
and overrides `JointForceCmd` (and optionally `JointVelocityCmd` for the
LOCK mode) on configured servo joints. This is the only safe intercept
layer in the presence of PX4 SITL, because PX4 continuously publishes to
the servo gz-transport topic — intercepts at the topic layer race with PX4.

## Failure Modes

A single Float32MultiArray topic drives all servos. Payload layout:

```
data = [mode_0, param_0, mode_1, param_1, ...]
```

One `(mode, param)` pair per servo, in the order declared in the SDF
`<servoJointList>`. Missing pairs default to NORMAL.

| ID | Mode   | Behavior                                          | `param`          |
|----|--------|---------------------------------------------------|------------------|
| 0  | NORMAL | Passthrough (no modification)                     | unused           |
| 1  | FLOPPY | `JointForceCmd = 0` — free-swinging servo         | unused           |
| 2  | LOCK   | Force = 0 AND velocity = 0 — frozen               | unused           |
| 3  | SCALE  | `force *= clamp(param, 0, 1)` — weaker            | 0.0~1.0          |
| 4  | LIMIT  | Range clamped to `ratio × SDF limits` (one-way)   | 0.0~1.0 (ratio)  |

### Physical mapping

- **FLOPPY** — broken linkage: no torque, aerodynamic forces swing the
  surface freely (realistic flutter).
- **LOCK** — stuck gearbox or seized servo: the joint freezes at whatever
  angle it was at when the failure started, regardless of command.
- **SCALE** — weakened actuator (low battery, worn gears): reduced control
  authority, commands still work but produce less deflection. Note: in a
  static setup with no aerodynamic load the joint still converges to the
  commanded target (just more slowly); deflection is only capped in flight.
- **LIMIT** — reduced range of motion (partial linkage jam, debris in
  mechanism): the joint's travel is capped to `ratio × SDF joint limits`.
  `param` is a ratio in `[0, 1]`; for a joint whose SDF defines
  `<lower>-0.53</lower><upper>0.53</upper>`, `param=0.5` yields an
  effective range of ±0.265 rad. Implemented as a one-way kinematic block:
  when the joint is outside the allowed range and the commanded force
  would push it further out, force and velocity are both zeroed for that
  tick; inward motion is never blocked. If the joint has no usable SDF
  limits, LIMIT is a no-op (warning logged at Configure time).

## SDF Configuration

**IMPORTANT**: declare this plugin AFTER all
`gz-sim-joint-position-controller-system` plugins in your SDF, so the
override runs after JPC writes its force command.

```xml
<!-- ... JointPositionController plugins above ... -->

<plugin filename="ServoFailureROS2" name="custom::ServoFailureROS2System">
  <servoJointList>
    <servoJoint name="left_ail_joint"/>
    <servoJoint name="right_ail_joint"/>
    <servoJoint name="left_elevator_joint"/>
    <servoJoint name="right_elevator_joint"/>
    <servoJoint name="rudder_joint"/>
  </servoJointList>
</plugin>
```

### SDF Parameters

- `<ServoFailureTopic>` (optional): ROS2 topic. Default
  `/<model_name>/servo_failure/commands`.
- `<servoJointList>` (required): ordered list of `<servoJoint name="..."/>`.
  Declaration order defines the index used in the message payload.

## Usage

For a model named `lc_62_0` with the 5-servo list above:

```bash
# Rudder (index 4) stuck
ros2 topic pub --once /lc_62_0/servo_failure/commands \
  std_msgs/msg/Float32MultiArray \
  "data: [0,0, 0,0, 0,0, 0,0, 2,0]"

# Left aileron (index 0) floppy, right elevator (index 3) at 50% authority
ros2 topic pub --once /lc_62_0/servo_failure/commands \
  std_msgs/msg/Float32MultiArray \
  "data: [1,0, 0,0, 0,0, 3,0.5, 0,0]"

# Rudder (index 4) range-limited to 30% of SDF limits
# (for lc_62 rudder with SDF ±0.53 rad → effective ±0.159 rad)
ros2 topic pub --once /lc_62_0/servo_failure/commands \
  std_msgs/msg/Float32MultiArray \
  "data: [0,0, 0,0, 0,0, 0,0, 4,0.3]"

# Restore everything
ros2 topic pub --once /lc_62_0/servo_failure/commands \
  std_msgs/msg/Float32MultiArray \
  "data: [0,0, 0,0, 0,0, 0,0, 0,0]"
```

### Encoding notes

- `data[2*i]` is the mode, cast from float to `uint8_t` (use whole
  numbers: `0.0`, `1.0`, `2.0`, `3.0`, `4.0`). Values outside `[0, 4]` are
  logged as warnings and treated as NORMAL.
- `data[2*i + 1]` is the param (only SCALE uses it).
- Payload length must be even. Shorter-than-servo-count arrays leave the
  trailing servos at NORMAL.

## Dependencies

- Gazebo Harmonic (gz-sim8)
- ROS2 Jazzy
- rclcpp, std_msgs

## Notes

- Plugin order matters: **after** all `JointPositionController` plugins,
  before or after `MotorFailureROS2`/`RealGazebo` does not matter.
- LOCK freezes the joint via `JointVelocityCmd = 0`, which gz-sim physics
  enforces as a hard kinematic constraint.
- FLOPPY leaves the joint subject to `<damping>` and aerodynamic forces
  from `gz-sim-lift-drag-system`.
- The plugin creates a ROS2 node named `servo_failure` (shared with other
  models if multiple instances run in the same process).
