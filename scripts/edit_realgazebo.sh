#!/bin/bash

# Determine GPU usage
USE_GPU=true
if [[ "$1" == "--no-gpu" ]]; then
    USE_GPU=false
    echo "GPU option disabled."
    shift # Remove --no-gpu argument
fi

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

container_name="edit_realgazebo"

docker stop "$container_name" 2>/dev/null
docker rm "$container_name" 2>/dev/null

docker run ${GPU_OPTION} ${GPU_RUNTIME} -it --privileged \
    -e LOCAL_USER_ID="$(id -u)" \
    -e DISPLAY=$DISPLAY \
    --env="QT_X11_NO_MITSHM=1" \
    ${GPU_ENV} \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev:/dev:rw \
    --hostname $(hostname) \
    --network host \
    --name "$container_name" mdeagewt/realgazebo-dev:0.2 bash

