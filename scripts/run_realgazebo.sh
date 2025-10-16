#!/bin/bash

xhost +

# Determine GPU usage and GUI mode
USE_GPU=true
USE_GUI=false
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
        *)
            break
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

if [ "$#" -lt 2 ] || [ "$#" -gt 3 ]; then
    echo "Usage: $0 [--no-gpu] [--gui] <file_path> <server_ip> [world_type]"
    echo "world_type options: default, urban, vils (default: default)"
    exit 1
fi

if [ ! -f "$1" ]; then
    echo "Error: The specified file does not exist."
    exit 1
fi

# Set world_type (default: "c-track")
WORLD_TYPE="c-track"
if [ "$#" -eq 3 ]; then
    case $3 in
        c-track|urban|vils)
            WORLD_TYPE="$3"
            echo "Using world type: $WORLD_TYPE"
            ;;
        *)
            echo "Error: Invalid world_type '$3'. Valid options: c-track, urban, vils"
            exit 1
            ;;
    esac
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
    --name "$container_name" aware4docker/realgazebo:1.1

docker cp $1 "$container_name":/home/user/

HEADLESS_ARG="true"
if [[ "$USE_GUI" == "true" ]]; then
    HEADLESS_ARG="false"
fi

docker exec -u user -it "$container_name" bash -c "source /opt/ros/jazzy/setup.bash && source /home/user/realgazebo/RealGazebo-ROS2/install/setup.bash && ros2 launch realgazebo realgazebo.launch.py vehicle:=/home/user/$(basename "$1") unreal_ip:=$2 headless:=$HEADLESS_ARG world:=$WORLD_TYPE"

docker stop "$container_name" 2>/dev/null
docker rm "$container_name" 2>/dev/null
