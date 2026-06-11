#!/bin/bash
# Keyboard control demo for one drone.
# Default: single-container simulation ('realgazebo', run_realgazebo.sh).
# Manager multi-container mode: exec into 'gazebo', which sits on
# vehicle-network and reaches every vehicle's isolated DDS.

if docker ps --format '{{.Names}}' | grep -qx "realgazebo"; then
    CONTAINER="realgazebo"
elif docker ps --format '{{.Names}}' | grep -qx "gazebo"; then
    CONTAINER="gazebo"
else
    echo "No simulation container found (looked for 'realgazebo' and 'gazebo')."
    exit 1
fi
echo "Using container: $CONTAINER"

docker exec -it -u user "$CONTAINER" bash -c "source /opt/ros/jazzy/setup.bash && source /home/user/realgazebo/RealGazebo-ROS2/install/setup.bash && ros2 launch realgazebo control_one_drone.launch.py"
