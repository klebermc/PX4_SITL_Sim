# http://wiki.ros.org/docker/Tutorials/GUI
myloc=$(realpath $0)
myloc2=$(dirname "$myloc")
hostpath="$myloc2/host"
hostws="$myloc2/catkin_ws"
mkdir -p "$hostpath"
mkdir -p "$hostws"
docker run -it \
    --net=host \
    --user=px4devel:px4devel \
    --env="DISPLAY" \
    --workdir="/home/px4devel" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$hostpath:/home/px4devel/host:rw" \
    --volume="$hostws:/home/px4devel/catkin_ws:rw" \
    --device=/dev/dri:/dev/dri \
    ros-test #\
    #--volume="/home/$USER:/home/$USER" \
    #--volume="/etc/group:/etc/group:ro" \
    #--volume="/etc/passwd:/etc/passwd:ro" \
    #--volume="/etc/shadow:/etc/shadow:ro" \
    #--volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
    #--gpus=all \

