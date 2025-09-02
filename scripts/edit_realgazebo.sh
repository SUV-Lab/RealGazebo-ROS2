container_name="edit_realgazebo"

docker stop "$container_name" 2>/dev/null
docker rm "$container_name" 2>/dev/null

docker run --gpus all --runtime=nvidia -it --privileged \
-e DISPLAY=$DISPLAY \
--env="QT_X11_NO_MITSHM=1" \
-e NVIDIA_DRIVER_CAPABILITIES=all \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v /dev:/dev:rw \
-u user \
--hostname $(hostname) \
--network host \
--name "$container_name" mdeagewt/realgazebo-dev:0.2 bash

