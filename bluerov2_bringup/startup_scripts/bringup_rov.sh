#!/usr/bin/env bash
mkdir -p $(rospack find bluerov2_bringup)/logs
LOGFILE=$(rospack find bluerov2_bringup)/logs/rosrov.log
screen -L -Logfile $LOGFILE -dmS rosrov roslaunch bluerov2_bringup bringup_ardusub_vehi.launch joystick_name:=f710