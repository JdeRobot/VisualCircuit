#!/bin/bash

echo "Starting..."

ls .

source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

echo "$ROS_DISTRO"

source /.env

python3 manager.py