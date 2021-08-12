#!/usr/bin/env bash
mkdir -p $(rospack find bluerov2_bringup)/logs
LOGFILE=$(rospack find bluerov2_bringup)/logs/rosrov.log
screen -L -Logfile $LOGFILE -dmS rosrov roslaunch --wait bluerov2_bringup bringup_ardusub_vehi.launch joystick_name:=f710
#ip_rov:=blue2rov.clients.wireless.dtu.dk ip_gcs:=blue2shore.clients.wireless.dtu.dk

# read -p 'Start recording? [y/n] ' record
# if [ $record == 'y' ]
# then
# 	bash ~/ros_ws/src/bluerov2/bluerov2_bringup/startup_scripts/start_recording.sh
# fi
#sleep 5
source ~/ros_ws/src/bluerov2/bluerov2_bringup/startup_scripts/start_recording.sh