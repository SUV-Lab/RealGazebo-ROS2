#!/bin/bash
# Camera stream session for a vehicle's UE image receiver.
#
# The receiver is a lifecycle node (spawned UNCONFIGURED next to every
# vehicle); configure+activate opens the UE RTSP stream and republishes
# it as a ROS image topic. Stream lifetime == this script's lifetime,
# with or without a window:
#
#   ./run_image_stream.sh [id] [type] [camera]          headless session:
#                                                        stream runs until
#                                                        Ctrl-C
#   ./run_image_stream.sh --gui [id] [type] [camera]    same session with
#                                                        the image_viewer
#                                                        window
#
# Exit always drives the receiver back to UNCONFIGURED (regardless of
# who activated it), so a stray stream is recovered by simply running
# this again and pressing Ctrl-C. Startup is state-aware: only the
# transitions still needed are sent.
#
#   vehicle_id   : vehicle instance id, 0-based (default: 0)
#   vehicle_type : x500 | x500_lidar_2d | lc_62 | rover_ackermann | boat (default: x500)
#   camera_type  : front | bottom | top (default: front)
#
# Default is the single-container simulation (container "realgazebo",
# started by run_realgazebo.sh). In manager multi-container mode the
# receiver and its isolated DDS live in the per-vehicle container, so we
# fall back to vehicle_<id> when "realgazebo" is not running.

MODE=headless
case "$1" in
    --gui) MODE=gui; shift ;;
esac

VEHICLE_ID=${1:-0}
VEHICLE_TYPE=${2:-x500}
CAMERA_TYPE=${3:-front}
NODE="/image_receiver_${VEHICLE_TYPE}_${VEHICLE_ID}_${CAMERA_TYPE}"

if docker ps --format '{{.Names}}' | grep -qx "realgazebo"; then
    CONTAINER="realgazebo"               # single-container simulation
elif docker ps --format '{{.Names}}' | grep -qx "vehicle_${VEHICLE_ID}"; then
    CONTAINER="vehicle_${VEHICLE_ID}"   # manager multi-container mode
else
    echo "No simulation container found (looked for 'realgazebo' and 'vehicle_${VEHICLE_ID}')."
    exit 1
fi
echo "Using container: $CONTAINER"

ROS_SETUP="source /opt/ros/jazzy/setup.bash && source /home/user/realgazebo/RealGazebo-ROS2/install/setup.bash"

# Keep stdin open always (-i): the in-container session holds on stdin, so
# it tears down on EOF when this client dies even without a TTY. Add a TTY
# (Ctrl-C passthrough) only when we actually have one.
TTY_FLAG="-i"
if [ -t 0 ]; then TTY_FLAG="-it"; fi

if [ "$MODE" = gui ]; then
    # The viewer performs the same session semantics itself: state-aware
    # startup, teardown to UNCONFIGURED on exit.
    exec docker exec $TTY_FLAG -u user -e DISPLAY="${DISPLAY:-:0}" "$CONTAINER" bash -c \
      "$ROS_SETUP && \
       ros2 run image_viewer image_viewer \
         --ros-args \
         -p vehicle_id:=${VEHICLE_ID} \
         -p vehicle_type:=${VEHICLE_TYPE} \
         -p camera_type:=${CAMERA_TYPE}"
fi

# Headless session: bring the stream up, hold until Ctrl-C/TERM, tear down.
exec docker exec $TTY_FLAG -u user "$CONTAINER" bash -c "$ROS_SETUP
STATE=\$(timeout 10 ros2 lifecycle get '$NODE' 2>/dev/null | awk '{print \$1}')
if [ -z \"\$STATE\" ]; then
    echo 'receiver node $NODE not reachable (is the vehicle spawned?)'
    exit 1
fi
echo \"$NODE state: \$STATE\"
case \"\$STATE\" in
    unconfigured) ros2 lifecycle set '$NODE' configure && ros2 lifecycle set '$NODE' activate ;;
    inactive)     ros2 lifecycle set '$NODE' activate ;;
    active)       echo 'already active - adopting the stream' ;;
    *)            echo \"unexpected state '\$STATE'\"; exit 1 ;;
esac
echo 'streaming - press Ctrl-C to stop'
teardown() {
    echo 'stopping stream...'
    ros2 lifecycle set '$NODE' deactivate >/dev/null 2>&1
    ros2 lifecycle set '$NODE' cleanup >/dev/null 2>&1
    echo 'stream stopped'
    exit 0
}
trap teardown INT TERM
# holds until Ctrl-C (trap) or until this docker exec's client dies and
# stdin hits EOF (signals do NOT cross docker exec without a TTY)
cat > /dev/null
teardown"
