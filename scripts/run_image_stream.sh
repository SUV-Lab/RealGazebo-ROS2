#!/bin/bash
# Demo script for the image streaming viewer.
#
# Runs image_viewer next to the camera receiver. The viewer drives the
# receiver's lifecycle (configure/activate) on demand, which opens the UE
# RTSP stream and republishes it as a ROS image topic.
#
# Default is the single-container simulation (container "realgazebo",
# started by run_realgazebo.sh). In manager multi-container mode the
# receiver and its isolated DDS live in the per-vehicle container, so we
# fall back to vehicle_<id> when "realgazebo" is not running.
#
# Usage: ./run_image_stream.sh [vehicle_id] [vehicle_type] [camera_type]
#   vehicle_id   : vehicle instance id, 0-based (default: 0)
#   vehicle_type : x500 | x500_lidar_2d | lc_62 | rover_ackermann | boat (default: x500)
#   camera_type  : front | bottom | top (default: front)

VEHICLE_ID=${1:-0}
VEHICLE_TYPE=${2:-x500}
CAMERA_TYPE=${3:-front}

if docker ps --format '{{.Names}}' | grep -qx "realgazebo"; then
    CONTAINER="realgazebo"               # single-container simulation
elif docker ps --format '{{.Names}}' | grep -qx "vehicle_${VEHICLE_ID}"; then
    CONTAINER="vehicle_${VEHICLE_ID}"   # manager multi-container mode
else
    echo "No simulation container found (looked for 'realgazebo' and 'vehicle_${VEHICLE_ID}')."
    exit 1
fi
echo "Using container: $CONTAINER"

docker exec -it -u user -e DISPLAY="${DISPLAY:-:0}" "$CONTAINER" bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   source /home/user/realgazebo/RealGazebo-ROS2/install/setup.bash && \
   ros2 run image_viewer image_viewer \
     --ros-args \
     -p vehicle_id:=${VEHICLE_ID} \
     -p vehicle_type:=${VEHICLE_TYPE} \
     -p camera_type:=${CAMERA_TYPE}"
