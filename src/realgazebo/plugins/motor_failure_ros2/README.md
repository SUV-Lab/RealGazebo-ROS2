# Motor Failure System Plugin

This Gazebo plugin enables motor failure simulation for multirotor vehicles in RealGazebo.

## Overview

The Motor Failure System plugin subscribes to a ROS2 topic to receive motor failure commands and directly controls motor joints to simulate failures. This allows simulating motor failures during flight testing.

## Features

- ROS2 integration for receiving motor failure commands (ratio-based partial failures)
- Automatic model name detection (no manual namespace configuration needed)
- Automatic detection of motor joints (rotor_0_joint, rotor_1_joint, etc.)
- Direct joint velocity degradation in PreUpdate cycle
- Thread-safe motor failure ratio handling
- Configurable topic names via SDF

## Configuration

### SDF Parameters

- `<MotorFailureTopic>` (optional): ROS2 topic for receiving motor failure commands
  - Default: `/<model_name>/motor_failure/ratios`
  - The model name is automatically detected from the entity name

### Example SDF Usage

**IMPORTANT**: The MotorFailureROS2System plugin must be declared **AFTER** the `gz-sim-multicopter-motor-model-system` plugin in your SDF file. This ensures the motor failure override runs after the motor model sets its velocity commands.

```xml
<!-- First: Motor model plugin -->
<plugin
    filename="gz-sim-multicopter-motor-model-system"
    name="gz::sim::systems::MulticopterMotorModel">
    <!-- motor model configuration -->
</plugin>

<!-- Second: Motor failure plugin (must come after motor model) -->

<!-- Minimal configuration (automatically uses model name) -->
<plugin
    filename="MotorFailureROS2"
    name="custom::MotorFailureROS2System">
</plugin>

<!-- Custom topic configuration -->
<plugin
    filename="MotorFailureROS2"
    name="custom::MotorFailureROS2System">
    <MotorFailureTopic>/custom/topic/name</MotorFailureTopic>
</plugin>
```

## Usage

### Publishing Motor Failure Commands

To trigger a motor failure, publish a `Float32MultiArray` to the ROS2 topic.
Each element `data[i]` is the failure ratio for motor `i` (0-indexed):
- `0.0` = normal operation
- `0.5` = 50% output reduction
- `1.0` = complete failure

For a vehicle with model name `x500_0`:

```bash
# Fail motor 0 by 50% (partial failure)
ros2 topic pub --once /x500_0/motor_failure/ratios \
  std_msgs/msg/Float32MultiArray "data: [0.5, 0.0, 0.0, 0.0]"

# Completely fail motor 0
ros2 topic pub --once /x500_0/motor_failure/ratios \
  std_msgs/msg/Float32MultiArray "data: [1.0, 0.0, 0.0, 0.0]"

# Restore all motors to normal
ros2 topic pub --once /x500_0/motor_failure/ratios \
  std_msgs/msg/Float32MultiArray "data: [0.0, 0.0, 0.0, 0.0]"
```

**Note**: Replace `x500_0` with your model's name (automatically detected from the entity name).

**Motor Indexing**:
- Motors are **0-indexed** matching `rotor_0_joint`, `rotor_1_joint`, etc.
- Arrays shorter than the motor count leave remaining motors at normal (0.0)
- Ratios are clamped to `[0.0, 1.0]`

### Monitoring Motor Failure Status

You can monitor the motor failure messages in the Gazebo console output.

## Dependencies

- Gazebo Harmonic (gz-sim8)
- ROS2 Jazzy or later
- rclcpp
- std_msgs

## Building

This plugin is built as part of the RealGazebo-ROS2 project.

### Building in Docker (Recommended)

1. Use the provided Docker container with all dependencies:
```bash
# Enter the Docker container
./scripts/edit_realgazebo.sh

# Inside the container, build the project
cd /home/user/vm/realgazebo/RealGazebo-ROS2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select realgazebo
```

### Building Locally

1. Source your ROS2 installation:
```bash
source /opt/ros/jazzy/setup.bash
```

2. Build the RealGazebo package:
```bash
cd RealGazebo-ROS2
colcon build --packages-select realgazebo
```

3. Source the workspace:
```bash
source install/setup.bash
```

## Notes

- The plugin automatically initializes ROS2 if not already initialized
- Motor failure ratios are thread-safe protected with a mutex
- The plugin applies motor failure in the PreUpdate cycle by scaling the current joint velocity command
- **Plugin Declaration Order**: This plugin must be declared AFTER the MulticopterMotorModel plugin in the SDF file to ensure proper execution order
- The plugin automatically detects the model name from the entity, eliminating the need for manual namespace configuration
