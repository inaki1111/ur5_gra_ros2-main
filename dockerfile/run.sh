#!/usr/bin/env bash

# ROS2 version
ROSVERSION=jazzy

# User volume
CURRENT_DIR=$(realpath .)
USERVOLUME=${CURRENT_DIR}/../example

xhost +

docker run \
--rm \
--tty \
--interactive \
--privileged \
--network host \
--ipc=host  \
--pid=host  \
--env DISPLAY=$DISPLAY \
--volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
--volume /dev:/dev \
--volume $USERVOLUME:/root/workspace/example:rw \
--device-cgroup-rule='c 81:* rmw' \
--name ros2-ur5-$ROSVERSION \
ros2-ur5-$ROSVERSION:latest \
bash
