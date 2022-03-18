#!/usr/bin/env bash
export ROS_MASTER_URI=http://jonas-ThinkPad-P1-Gen-2:11311
source /opt/ros/noetic/setup.sh
source /home/msr/install/setup.bash
exec "$@"
