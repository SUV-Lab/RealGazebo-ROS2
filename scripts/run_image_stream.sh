#!/bin/bash
# Demo script for image streaming viewer
# Usage: ./run_image_stream.sh [vehicle_num] [vehicle_type] [camera_type]
#   vehicle_num  : vehicle index (default: 0)
#   vehicle_type : x500 | lc_62 (default: x500)
#   camera_type  : front | down | ... (default: front)

VEHICLE_NUM=${1:-0}
VEHICLE_TYPE=${2:-x500}
CAMERA_TYPE=${3:-front}

docker exec -it -u user realgazebo bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   source /home/user/realgazebo/RealGazebo-ROS2/install/setup.bash && \
   ros2 run image_viewer image_viewer \
     --ros-args \
     -p vehicle_num:=${VEHICLE_NUM} \
     -p vehicle_type:=${VEHICLE_TYPE} \
     -p camera_type:=${CAMERA_TYPE}"
