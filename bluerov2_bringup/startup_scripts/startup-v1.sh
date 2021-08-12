#!/usr/bin/env bash

source ~/.bashrc
export ROS_IP=192.168.2.1
source /home/obstec/ardupilot/Tools/completion/completion.bash
source /opt/ros/melodic/setup.bash
source /home/obstec/ros_ws/devel/setup.bash

source ~/ros_ws/src/bluerov2/bluerov2_bringup/startup_scripts/bringup_rov-v2.sh
echo "Startup_rov: done"