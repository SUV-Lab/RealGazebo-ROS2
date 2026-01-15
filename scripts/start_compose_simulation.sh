#!/bin/bash
#
# Start RealGazebo Multi-Container Simulation
#
# Usage:
#   ./start_compose_simulation.sh [options] <config_file> [unreal_ip] [world_type]
#
# Options:
#   --gui           Enable Gazebo GUI (default: headless)
#   --verbose       Enable verbose logging (level 4)
#   --no-gpu        Disable GPU acceleration
#   --unreal-ip IP  Unreal Engine server IP (default: host.docker.internal)
#   --unreal-port P Unreal Engine server port (default: 5005)
#   --world TYPE    World type: c-track, urban, vils (default: c-track)
#   --follow        Follow logs after starting
#
# Examples:
#   ./start_compose_simulation.sh config.yaml                                  # Config only (required)
#   ./start_compose_simulation.sh config.yaml 10.255.70.74                     # Config + IP
#   ./start_compose_simulation.sh config.yaml 10.255.70.74 urban               # Config + IP + world
#   ./start_compose_simulation.sh --gui config.yaml                            # Options before config
#   ./start_compose_simulation.sh --unreal-ip 10.0.0.1 --world urban config.yaml
#

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Default values
CONFIG_FILE=""
HEADLESS=true
VERBOSE=false
USE_GPU=true
UNREAL_IP="host.docker.internal"
UNREAL_IP_SET=false
UNREAL_PORT="5005"
WORLD_TYPE="c-track"
WORLD_TYPE_SET=false
FOLLOW_LOGS=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --gui)
            HEADLESS=false
            echo "GUI mode enabled"
            shift
            ;;
        --verbose|-v)
            VERBOSE=true
            echo "Verbose mode enabled"
            shift
            ;;
        --no-gpu)
            USE_GPU=false
            echo "GPU disabled"
            shift
            ;;
        --unreal-ip)
            UNREAL_IP="$2"
            UNREAL_IP_SET=true
            echo "Unreal IP: $UNREAL_IP"
            shift 2
            ;;
        --unreal-port)
            UNREAL_PORT="$2"
            echo "Unreal Port: $UNREAL_PORT"
            shift 2
            ;;
        --world)
            case $2 in
                c-track|urban|vils)
                    WORLD_TYPE="$2"
                    WORLD_TYPE_SET=true
                    echo "World type: $WORLD_TYPE"
                    ;;
                *)
                    echo "Error: Invalid world type '$2'. Valid options: c-track, urban, vils"
                    exit 1
                    ;;
            esac
            shift 2
            ;;
        --follow|-f)
            FOLLOW_LOGS=true
            shift
            ;;
        --help|-h)
            head -30 "$0" | tail -25
            exit 0
            ;;
        *)
            if [[ -z "$CONFIG_FILE" ]]; then
                # First positional arg: config file
                if [[ -f "$1" ]]; then
                    CONFIG_FILE="$1"
                elif [[ -f "${PROJECT_DIR}/$1" ]]; then
                    CONFIG_FILE="${PROJECT_DIR}/$1"
                else
                    echo "Error: Config file not found: $1"
                    exit 1
                fi
            elif [[ "$UNREAL_IP_SET" == "false" ]]; then
                # Second positional arg: unreal IP
                UNREAL_IP="$1"
                UNREAL_IP_SET=true
                echo "Unreal IP: $UNREAL_IP"
            elif [[ "$WORLD_TYPE_SET" == "false" ]]; then
                # Third positional arg: world type
                case $1 in
                    c-track|urban|vils)
                        WORLD_TYPE="$1"
                        WORLD_TYPE_SET=true
                        echo "World type: $WORLD_TYPE"
                        ;;
                    *)
                        echo "Error: Invalid world type '$1'. Valid options: c-track, urban, vils"
                        exit 1
                        ;;
                esac
            else
                echo "Error: Unknown argument: $1"
                exit 1
            fi
            shift
            ;;
    esac
done

# Validate config file (required)
if [[ -z "$CONFIG_FILE" ]]; then
    echo "Error: Config file is required"
    echo "Usage: $0 [options] <config_file> [unreal_ip] [world_type]"
    exit 1
fi

# Convert localhost to Docker-compatible host
if [[ "$UNREAL_IP" == "127.0.0.1" ]] || [[ "$UNREAL_IP" == "localhost" ]]; then
    echo "Note: Converting $UNREAL_IP → host.docker.internal (Docker compatibility)"
    UNREAL_IP="host.docker.internal"
fi

echo "Using config: $CONFIG_FILE"

# Allow X11 connections
xhost + 2>/dev/null || true

# Generate docker-compose.override.yml from config
echo "Generating docker-compose configuration..."
python3 "${SCRIPT_DIR}/generate_compose.py" "$CONFIG_FILE" \
    --unreal-ip "$UNREAL_IP" \
    --unreal-port "$UNREAL_PORT" \
    --world "$WORLD_TYPE"

# Set environment variables
export LOCAL_USER_ID=$(id -u)
export DISPLAY=${DISPLAY:-:0}
export HEADLESS=$HEADLESS
export VERBOSE=$VERBOSE
export UNREAL_IP=$UNREAL_IP
export UNREAL_PORT=$UNREAL_PORT
export WORLD=$WORLD_TYPE

# Get Docker host gateway IP for MAVLink GCS connection
DOCKER_HOST_IP=$(docker network inspect bridge --format '{{range .IPAM.Config}}{{.Gateway}}{{end}}' 2>/dev/null)
if [ -z "$DOCKER_HOST_IP" ]; then
    DOCKER_HOST_IP=$(ip route | grep docker0 | awk '{print $9}' 2>/dev/null)
fi
if [ -z "$DOCKER_HOST_IP" ]; then
    DOCKER_HOST_IP="172.17.0.1"
    echo "Warning: Could not detect Docker host IP, using default $DOCKER_HOST_IP"
fi
export MAVLINK_GCS_IP=$DOCKER_HOST_IP
echo "MAVLink GCS IP: $MAVLINK_GCS_IP"

# Check for host PX4 path and set if available
if [[ -d "/home/kmk/ws/realgazebo/RealGazebo-PX4" ]]; then
    export PX4_PATH="/home/kmk/ws/realgazebo/RealGazebo-PX4"
fi

# GPU configuration
COMPOSE_PROFILES=""
if [[ "$USE_GPU" == "true" ]] && command -v nvidia-smi &>/dev/null && nvidia-smi &>/dev/null; then
    if docker info 2>/dev/null | grep -q nvidia; then
        echo "NVIDIA GPU detected and enabled"
        # Note: GPU support requires nvidia-docker runtime
        # Add to docker-compose.yml if needed
    else
        echo "Warning: nvidia-docker not available, running without GPU"
    fi
else
    echo "Running without GPU acceleration"
fi

# Start containers
cd "$PROJECT_DIR"
echo ""
echo "Starting simulation..."
echo "========================================"

docker compose down --remove-orphans 2>/dev/null || true
docker compose up -d

echo ""
echo "Containers started. Waiting for Gazebo to be ready..."
echo "(This may take 30-60 seconds for map loading)"

# Wait for gazebo to be healthy
MAX_WAIT=120
WAIT_TIME=0
while [[ $WAIT_TIME -lt $MAX_WAIT ]]; do
    STATUS=$(docker inspect --format='{{.State.Health.Status}}' gazebo 2>/dev/null || echo "unknown")
    if [[ "$STATUS" == "healthy" ]]; then
        echo ""
        echo "Gazebo is ready!"
        break
    fi
    echo -n "."
    sleep 5
    WAIT_TIME=$((WAIT_TIME + 5))
done

if [[ "$STATUS" != "healthy" ]]; then
    echo ""
    echo "Warning: Gazebo may not be fully ready (status: $STATUS)"
fi

echo ""
echo "========================================"
echo "Simulation is running!"
echo ""
echo "Useful commands:"
echo "  docker compose logs -f           # View all logs"
echo "  docker compose logs -f gazebo    # View Gazebo logs"
echo "  docker compose logs -f vehicle_0 # View vehicle_0 logs"
echo "  docker compose ps                # List containers"
echo "  ./scripts/stop_compose_simulation.sh     # Stop simulation"
echo ""

if [[ "$FOLLOW_LOGS" == "true" ]]; then
    docker compose logs -f
fi
