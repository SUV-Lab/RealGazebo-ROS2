#!/bin/bash
# Run one PILS vehicle on THIS (remote) PC: PX4 SITL in a container that
# attaches to the RealGazebo world served by the sim PC over gz-transport.
#
# Prerequisites:
#   - the sim PC runs the fleet with this vehicle declared `mode: pils`
#     (the manager spawns the gz model and nothing else), and advertises
#     its LAN address:  GZ_IP=<sim pc ip> scripts/run_realgazebo.sh ...
#   - this PC has the fleet image (PILS_IMAGE, default mdeagewt/realgazebo:ue5.7)
#
# The container uses host networking, advertises this host's own address
# on the route toward the sim PC, and runs ONLY PX4 (the model already
# exists in the shared world — PX4_GZ_MODEL_NAME attaches, never spawns).
# The uXRCE client targets a local agent (companion pattern: run
# MicroXRCEAgent on this PC), and MAVLink beacons to qgc_ip.
#
# Usage: run_pils_vehicle.sh <vehicle_type> <id> <sim_pc_ip> [qgc_ip] [world]
set -e
TYPE=${1:?usage: run_pils_vehicle.sh <vehicle_type> <id> <sim_pc_ip> [qgc_ip] [world]}
ID=${2:?vehicle id (the fleet YAML key)}
SIM_IP=${3:?sim pc ip}
QGC_IP=${4:-$SIM_IP}
WORLD=${5:-c-track}
IMAGE=${PILS_IMAGE:-mdeagewt/realgazebo:ue5.7}

# advertise the address this host uses to reach the sim PC
MY_IP=$(ip -4 route get "$SIM_IP" | grep -oP 'src \K\S+')

exec docker run --rm -d --network host --name "pils_${TYPE}_${ID}" \
  -e GZ_PARTITION=realgazebo \
  -e GZ_IP="$MY_IP" \
  -e PX4_GZ_STANDALONE=1 \
  -e PX4_GZ_MODEL_NAME="${TYPE}_${ID}" \
  -e PX4_GZ_WORLD="$WORLD" \
  -e PX4_UXRCE_DDS_NS="vehicle$((ID + 1))" \
  -e PX4_PARAM_UXRCE_DDS_SYNCT=0 \
  -e MAVLINK_GCS_IP="$QGC_IP" \
  -e VEHICLE_TYPE="$TYPE" \
  -e VEHICLE_ID="$ID" \
  "$IMAGE" bash -c '
    # own uXRCE agent, docker-mode style (the SITL uxrce client targets
    # localhost:8888 by default). MUST start BEFORE sourcing ROS: jazzy'"'"'s
    # LD_LIBRARY_PATH shadows the standalone agent'"'"'s bundled FastDDS and
    # every entity creation then fails with error 255.
    /usr/local/bin/MicroXRCEAgent udp4 -p 8888 -v2 &
    source /opt/ros/jazzy/setup.bash
    PX4=/home/user/realgazebo/RealGazebo-PX4
    # airframe id from the {id}_gz_{type} filename convention (same rule as
    # realgazebo.airframes.scan_airframes, shell-side so the image needs no
    # particular realgazebo install state)
    AF=$(ls "$PX4/ROMFS/px4fmu_common/init.d-posix/airframes" | grep -E "^[0-9]+_gz_${VEHICLE_TYPE}$" | head -1)
    if [ -z "$AF" ]; then echo "no gz airframe for type ${VEHICLE_TYPE}" >&2; exit 1; fi
    export PX4_SYS_AUTOSTART=${AF%%_*}
    cd "$PX4/build/px4_sitl_default"
    exec bin/px4 -d -i "$VEHICLE_ID"
  '
