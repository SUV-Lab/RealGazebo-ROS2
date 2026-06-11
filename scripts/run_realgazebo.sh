#!/bin/bash

xhost +

# Show git branch of this repo (works from any cwd; silent if not a git repo)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GIT_BRANCH="$(git -C "$SCRIPT_DIR" rev-parse --abbrev-ref HEAD 2>/dev/null)"
if [[ -n "$GIT_BRANCH" ]]; then
    echo "Git branch: $GIT_BRANCH"
else
    echo "Git branch: (not a git repository)"
fi

# Default values
USE_GPU=true
USE_GUI=false
CONFIG_FILE=""
UNREAL_IP="127.0.0.1"
UNREAL_IP_SET=false
WORLD_TYPE="c-track"
WORLD_TYPE_SET=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --no-gpu)
            USE_GPU=false
            echo "GPU option disabled."
            shift
            ;;
        --gui)
            USE_GUI=true
            echo "GUI mode enabled."
            shift
            ;;
        --unreal-ip)
            UNREAL_IP="$2"
            UNREAL_IP_SET=true
            echo "Unreal IP: $UNREAL_IP"
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
        --help|-h)
            echo "Usage: $0 [options] [config_file] [unreal_ip] [world_type]"
            echo ""
            echo "Monolithic (single-container) simulation, manager-driven."
            echo "Without a config file the world starts empty and vehicles"
            echo "spawn/despawn at runtime over UDP :5006."
            echo ""
            echo "Options:"
            echo "  --no-gpu          Disable GPU acceleration"
            echo "  --gui             Enable Gazebo GUI (default: headless)"
            echo "  --unreal-ip IP    Unreal Engine server IP (default: 127.0.0.1)"
            echo "  --world TYPE      World type: c-track, urban, vils (default: c-track)"
            exit 0
            ;;
        *)
            if [[ -z "$CONFIG_FILE" ]]; then
                # First positional arg: config file
                if [[ -f "$1" ]]; then
                    CONFIG_FILE="$1"
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

# Validate and configure GPU options
GPU_OPTION=""
GPU_RUNTIME=""
GPU_ENV=""
if [[ "$USE_GPU" == "true" ]] && command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
    # Check nvidia-docker installation
    if docker info 2>/dev/null | grep -q nvidia; then
        echo "NVIDIA GPU detected. Enabling GPU options."
        GPU_OPTION="--gpus all"
        GPU_RUNTIME="--runtime=nvidia"
        GPU_ENV="-e NVIDIA_DRIVER_CAPABILITIES=all"
    else
        echo "nvidia-docker not installed. Running without GPU support."
    fi
else
    if [[ "$USE_GPU" == "false" ]]; then
        echo "Running without GPU support."
    else
        echo "GPU not found. Running without GPU support."
    fi
fi

# A config file is optional since the manager flow: without one the world
# starts empty and vehicles arrive at runtime over UDP :5006
if [[ -z "$CONFIG_FILE" ]]; then
    echo "No vehicle YAML given: empty world, vehicles spawn at runtime via UDP :5006"
fi

container_name="realgazebo"

docker stop "$container_name" 2>/dev/null
docker rm "$container_name" 2>/dev/null

docker run ${GPU_OPTION} ${GPU_RUNTIME} -d -it --privileged \
    -e LOCAL_USER_ID="$(id -u)" \
    -e DISPLAY=$DISPLAY \
    --env="QT_X11_NO_MITSHM=1" \
    ${GPU_ENV} \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev:/dev:rw \
    --hostname $(hostname) \
    --network host \
    --name "$container_name" mdeagewt/realgazebo:ue5.7

HEADLESS_ARG="true"
if [[ "$USE_GUI" == "true" ]]; then
    HEADLESS_ARG="false"
fi

# Manager-driven monolithic launch (vehicles run as subprocesses in this
# container). A YAML only adds a boot fleet; UDP spawn works either way.
LAUNCH_ARGS="unreal_ip:=$UNREAL_IP headless:=$HEADLESS_ARG world:=$WORLD_TYPE"
if [[ -n "$CONFIG_FILE" ]]; then
    docker cp "$CONFIG_FILE" "$container_name":/home/user/
    LAUNCH_ARGS="yaml_path:=/home/user/$(basename "$CONFIG_FILE") $LAUNCH_ARGS"
fi

# Allocate a TTY only when we have one (keeps the script usable from CI)
TTY_FLAG=""
if [ -t 0 ]; then TTY_FLAG="-it"; fi

docker exec -u user $TTY_FLAG "$container_name" bash -c "source /opt/ros/jazzy/setup.bash && source /home/user/realgazebo/RealGazebo-ROS2/install/setup.bash && ros2 launch realgazebo manager_sim.launch.py $LAUNCH_ARGS"

docker stop "$container_name" 2>/dev/null
docker rm "$container_name" 2>/dev/null
