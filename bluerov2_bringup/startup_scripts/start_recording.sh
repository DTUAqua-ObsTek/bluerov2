#!/usr/bin/env bash
TS=$(date -u '+%Y%m%dT%H%M%S')
FP=$HOME/Desktop/ROS_Recordings/$TS
mkdir -p $FP
mkdir -p $(rospack find bluerov2_bringup)/logs
LOGFILE=$(rospack find bluerov2_bringup)/logs/rosrecord.log
screen -L -Logfile $LOGFILE -dmS rosrecord roslaunch bluerov2_bringup record.launch bag_dir:=$FP