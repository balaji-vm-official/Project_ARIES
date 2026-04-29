#!/bin/bash

#source the ros2 file and setup files
source /opt/ros/jazzy/setup.bash
source /home/comrade/Documents/Project_ARIES/ros-ws/install/setup.bash

#Using Cyclone DDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

#move to ros2 workspace directory
cd /home/comrade/Documents/Project_ARIES/ros-ws

#build the workspace
colcon build --symlink-install

#use the following command to run the this file "~/.ros2.sh"