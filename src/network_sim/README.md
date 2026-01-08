# network_sim

## Overview

The `network_sim` package is a ROS2-based vehicle network simulator that provides realistic V2V (Vehicle-to-Vehicle) communication modeling and network traffic control. It calculates communication quality metrics based on inter-vehicle distances and applies real network impairments using Linux Traffic Control (TC) to simulate realistic wireless communication conditions in multi-vehicle systems.

## Features

- **Distance-based Communication Modeling**: Calculates RSSI, packet loss rate, latency, and jitter based on vehicle positions using log-distance path loss models
- **Real Network Impairment**: Applies calculated network conditions to actual network interfaces using Linux TC qdisc
- **Dynamic Vehicle Discovery**: Automatically discovers and tracks vehicles through ROS2 topic scanning
- **Congestion-aware Model**: Optionally accounts for network congestion effects based on the number of vehicles in communication range
- **Real-time Adjustment**: Continuously updates network impairments based on changing vehicle positions

## Building

Navigate to your ROS2 workspace and build the package:

```bash
cd ~/realgazebo/RealGazebo-ROS2
colcon build --packages-select network_sim
source install/setup.bash
```

## Usage

Launch the network simulator using the provided launch file:

```bash
ros2 launch network_sim network_sim.launch.py instance_id:=0 reference_vehicle_id:=1 network_interface:=eth1
```

### Launch Parameters

- `instance_id` (default: `0`): Vehicle instance ID (container ID, not ROS2 topic ID)
- `reference_vehicle_id` (default: `1`): Reference vehicle ID for V2V communication (ROS2 topic ID, typically instance_id + 1)
- `network_interface` (default: `eth1`): Network interface name for TC control

### Runtime Parameters

The node accepts the following parameters:

- `enable_on_startup` (default: `True`): Enable TC impairments on node startup
- `max_latency_ms` (default: `1000.0`): Maximum latency cap in milliseconds
- `max_jitter_ms` (default: `500.0`): Maximum jitter cap in milliseconds
- `max_packet_loss_rate` (default: `0.99`): Maximum packet loss rate (0.0-1.0)

### Services

- `~/enable` (std_srvs/SetBool): Enable or disable TC impairments at runtime

## Architecture

The package consists of two integrated components:

1. **V2V Communication Model Node**: Monitors vehicle positions from `vehicle<N>/fmu/out/vehicle_global_position` topics, calculates inter-vehicle distances using Haversine formula, and computes communication quality metrics
2. **TC Controller Node**: Receives quality metrics and applies corresponding network impairments to the specified network interface using Linux TC qdisc rules

## Dependencies

- `rclcpp`: ROS2 C++ client library
- `px4_msgs`: PX4 message definitions (for vehicle position data)
- `std_srvs`: Standard ROS2 service definitions
- `std_msgs`: Standard ROS2 message definitions
- `diagnostic_msgs`: Diagnostic message definitions

## Requirements

- Linux system with Traffic Control (tc) utilities installed
- Root or CAP_NET_ADMIN capabilities for TC operations
- ROS2 (tested with ROS2 Humble or later)