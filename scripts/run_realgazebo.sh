if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <file_path> <server_ip>"
    exit 1
fi

if [ ! -f "$1" ]; then
    echo "Error: The specified file does not exist."
    exit 1
fi

container_name="realgazebo"

docker stop "$container_name" 2>/dev/null
docker rm "$container_name" 2>/dev/null

docker run --gpus all --runtime=nvidia -d -it --privileged \
-e DISPLAY=$DISPLAY \
--env="QT_X11_NO_MITSHM=1" \
-e NVIDIA_DRIVER_CAPABILITIES=all \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v /dev:/dev:rw \
--hostname $(hostname) \
--network host \
--name "$container_name" mdeagewt/realgazebo:0.2

docker cp $1 "$container_name":/home/user/

docker exec -u user -it "$container_name" bash -c "source /opt/ros/jazzy/setup.bash && source /home/user/realgazebo/RealGazebo-ROS2/install/setup.bash && ros2 launch realgazebo realgazebo.launch.py vehicle:=/home/user/$(basename "$1") server_ip:=$2"

docker stop "$container_name" 2>/dev/null
docker rm "$container_name" 2>/dev/null
