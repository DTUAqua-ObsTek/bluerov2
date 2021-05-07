#!/usr/bin/env bash
source /opt/ros/kinetic/setup.bash
source /home/pi/mavros_catkin_ws/devel/setup.bash
source /home/pi/aqua_catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.2.1:11311
export ROS_IP=192.168.2.2
exec "$@"
