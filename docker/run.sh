#!/bin/bash

xhost +local:root
mkdir -p $HOME/.ws/vinsfusion
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
    -v /home/shady/Others/:/datasets/ \
    ros:vins-fusion \
    /bin/bash -c \
    "cd /root/catkin_ws/; source devel/setup.bash; bash"