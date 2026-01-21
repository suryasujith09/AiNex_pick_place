#!/bin/bash
cd /home/ubuntu/ros_ws
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
rosrun pick_place interactive_game.py
