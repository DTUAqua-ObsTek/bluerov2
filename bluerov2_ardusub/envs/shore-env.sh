#!/usr/bin/env bash
source /opt/ros/melodic/setup.bash
source /home/fft/ros_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.2.1:11311
export ROS_IP=192.168.2.1
export DISPLAY=:0
exec "$@"
