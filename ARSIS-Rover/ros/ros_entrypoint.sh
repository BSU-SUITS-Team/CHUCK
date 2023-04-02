#!/usr/bin/env bash
set -x
source /opt/ros/noetic/setup.bash
cd /catkin_ws/
catkin_make
source ./devel/setup.bash
roslaunch vision_kit vision_kit.launch
