#!/usr/bin/env bash
set -x
source /opt/ros/noetic/setup.bash
cd /catkin_ws/
catkin_make
source ./devel/setup.bash
echo $VISION_KIT_MODE
if [[ "${VISION_KIT_MODE}" == "PROTOTYPE" ]]; then
    roslaunch vision_kit vision_kit_prototype.launch
elif [[ "${VISION_KIT_MODE}" == "VISION" ]]; then
    roslaunch vision_kit vision_kit_vision.launch
elif [[ "${VISION_KIT_MODE}" == "ROVER" ]]; then
    roslaunch vision_kit vision_kit_rover.launch
fi
