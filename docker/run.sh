#!/bin/bash

xhost +local:root
mkdir -p ws/src
docker run \
    -it \
    --rm \
    --net=host \
    --privileged \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --device=/dev/dri \
    --device=/dev/video0 \
    -v $HOME/.ws/vinsfusion/:/root/catkin_ws/ \
    -v $(pwd)/:/root/catkin_ws/src/VINS-Fusion/ \
    -v /run/media/tdemirdal/tb/:/datasets/ \
    ros:vins-fusion \
    /bin/bash -c \
    "cd /root/catkin_ws/; source devel/setup.bash; bash"