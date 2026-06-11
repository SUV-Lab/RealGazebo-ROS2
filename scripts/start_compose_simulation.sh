#!/bin/bash
# Start the RealGazebo multi-container simulation (manager-driven).
#
# Brings up the gazebo container (Gazebo server + realgazebo manager) via
# docker compose. Vehicles are created at runtime by UDP commands on :5006
# (wire contract: src/realgazebo/realgazebo/protocol.py) and/or spawned at
# boot from a vehicle YAML. Replaces the retired generate_compose.py flow.
#
# Usage (positionals match the legacy script):
#   ./scripts/start_compose_simulation.sh [options] [vehicle_yaml] [unreal_ip] [world]
#
# Arguments:
#   [vehicle_yaml]   Host path to a vehicle YAML (same format as
#                    src/realgazebo/yaml/example.yaml) to spawn at boot;
#                    relative paths resolve against your current directory.
#                    Omit to start with an empty world: vehicles are then
#                    spawned at runtime over UDP only.
#   [unreal_ip]      Same as --unreal-ip (legacy positional form)
#   [world]          Same as --world (legacy positional form)
#
# Options:
#   --gui            Run Gazebo with GUI (default: headless; runs `xhost +local:`)
#   --dev            Dev mode: mount the working tree and build at startup
#   --unreal-ip IP   Unreal Engine host (default: host.docker.internal;
#                    localhost/127.0.0.1 is rewritten automatically)
#   --unreal-port P  Unreal Engine UDP port (default: 5005)
#   --world W        World: c-track | urban | vils (default: c-track)
#   --image IMG      Manager/vehicle image (default: aware4docker/realgazebo:1.2-manager)
set -euo pipefail
ORIG_PWD="$(pwd)"
cd "$(dirname "$0")/.."

UNREAL_IP="host.docker.internal"
UNREAL_PORT="5005"
WORLD="c-track"
HEADLESS="true"
IMAGE="aware4docker/realgazebo:1.2-manager"
DEV_MODE=false
DRY_RUN=false
VEHICLE_YAML=""
POS=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --gui)         HEADLESS="false"; shift ;;
        --dev)         DEV_MODE=true; shift ;;
        --unreal-ip)   UNREAL_IP="$2"; shift 2 ;;
        --unreal-port) UNREAL_PORT="$2"; shift 2 ;;
        --world)       WORLD="$2"; shift 2 ;;
        --image)       IMAGE="$2"; shift 2 ;;
        --dry-run)     DRY_RUN=true; shift ;;
        -h|--help)     grep '^#' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        -*)            echo "Unknown option: $1 (see --help)"; exit 1 ;;
        *)  # legacy positional order: vehicle_yaml, unreal_ip, world
            POS=$((POS + 1))
            case $POS in
                1) VEHICLE_YAML="$1" ;;
                2) UNREAL_IP="$1" ;;
                3) WORLD="$1" ;;
                *) echo "Too many positional arguments: $1 (see --help)"; exit 1 ;;
            esac
            shift ;;
    esac
done

# Resolve a relative yaml path against the caller's directory (the script
# itself runs from the repo root)
if [[ -n "$VEHICLE_YAML" && "$VEHICLE_YAML" != /* ]]; then
    VEHICLE_YAML="$ORIG_PWD/$VEHICLE_YAML"
fi

# Docker cannot reach the host via loopback; rewrite to the host gateway alias
if [[ "$UNREAL_IP" == "127.0.0.1" || "$UNREAL_IP" == "localhost" ]]; then
    echo "Note: rewriting $UNREAL_IP -> host.docker.internal (Docker compatibility)"
    UNREAL_IP="host.docker.internal"
fi

# Auto-detect the GCS (QGC on this host) address from the default docker
# bridge gateway, like the legacy flow did; fall back to the usual default.
MAVLINK_GCS_IP=$(docker network inspect bridge \
    --format '{{(index .IPAM.Config 0).Gateway}}' 2>/dev/null || true)
MAVLINK_GCS_IP="${MAVLINK_GCS_IP:-172.17.0.1}"
echo "MAVLink GCS IP: $MAVLINK_GCS_IP"

# Vehicle YAML: copy into ./config (mounted read-only at /config in the container)
MANAGER_YAML=""
if [[ -n "$VEHICLE_YAML" ]]; then
    [[ -f "$VEHICLE_YAML" ]] || { echo "Vehicle yaml not found: $VEHICLE_YAML"; exit 1; }
    mkdir -p config
    cp "$VEHICLE_YAML" config/vehicles.yaml
    MANAGER_YAML="/config/vehicles.yaml"
    echo "Boot vehicles from: $VEHICLE_YAML"
else
    echo "No vehicle YAML given: empty world, vehicles spawn at runtime via UDP :5006"
fi

COMPOSE=(docker compose -f docker-compose.yml)
if $DEV_MODE; then
    COMPOSE+=(-f docker-compose.dev.yml)
    echo "Mode: dev (working tree mounted, builds at startup)"
else
    if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
        echo "Image $IMAGE not found. Build it once with:"
        echo "  docker build -f docker/Dockerfile.update -t $IMAGE ."
        exit 1
    fi
    echo "Mode: baked image ($IMAGE)"
fi

if [[ "$HEADLESS" == "false" ]]; then
    xhost +local: >/dev/null 2>&1 || echo "Warning: xhost failed (no X server?)"
fi

export DISPLAY="${DISPLAY:-:0}" HEADLESS WORLD UNREAL_IP UNREAL_PORT \
       MAVLINK_GCS_IP MANAGER_YAML MANAGER_IMAGE="$IMAGE" VEHICLE_IMAGE="$IMAGE"

if $DRY_RUN; then
    echo "[dry-run] HEADLESS=$HEADLESS WORLD=$WORLD UNREAL_IP=$UNREAL_IP:$UNREAL_PORT"
    echo "[dry-run] MANAGER_YAML=${MANAGER_YAML:-<none>} IMAGE=$IMAGE DEV=$DEV_MODE"
    exit 0
fi

"${COMPOSE[@]}" down --timeout 15 >/dev/null 2>&1 || true
"${COMPOSE[@]}" up -d

echo -n "Waiting for the manager"
for _ in $(seq 1 90); do
    if docker logs gazebo 2>&1 | grep -qa "listening for UDP"; then
        echo; echo "Ready: manager is listening for spawn/despawn commands on UDP :5006"
        echo "(wire contract: src/realgazebo/realgazebo/protocol.py)"
        exit 0
    fi
    echo -n "."; sleep 2
done
echo; echo "Manager did not come up in time; check: docker logs gazebo"
exit 1
