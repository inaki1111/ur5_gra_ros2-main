#!/usr/bin/env bash

# ROS2 version
ROSVERSION=jazzy

docker build -f Dockerfile.$ROSVERSION -t ros2-ur5-$ROSVERSION .