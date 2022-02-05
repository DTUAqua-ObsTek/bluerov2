# Autonomous Marine Systems Software Setup Guide #

The following steps are a checklist for ensuring you are able to use the open-source software that is used within this course.

## Install Ubuntu ##

You may skip this step if Ubuntu 20.04 is already installed on your machine.

- OS: Ubuntu 20.04, see [Download Page](https://releases.ubuntu.com/20.04/) and [This Installation Guide for Dual Boot](https://help.ubuntu.com/community/WindowsDualBoot)
- Configure Wi-Fi: If using DTU's DTUSecure or Eduroam WiFi networks, then follow the configuration instructions [Here](https://itswiki.compute.dtu.dk/index.php/DTUsecure_WiFi)

## Quick Install ##

Run the script [convenient_install.sh](convenient_install.sh):

`. convenient_install.sh`

## Slow Install (If you want to install each component yourself) ##

### Install Robot Operating System ###

- Noetic Ninjemys: Follow the installation instructions [Here](http://wiki.ros.org/noetic/Installation/Ubuntu), make sure to specify ros-noetic-desktop-full when installing.

### ROS Package Installation ###

Create a ROS workspace:

```
mkdir -p $HOME/ros_ws/src
cd $HOME/ros_ws/src
catkin_init_workspace
```

Clone the bluerov2 repository to the workspace:

```
git clone https://github.com/DTUAqua-ObsTek/bluerov2.git
cd bluerov2
```

Checkout the submodules:

`git submodule update --init --recursive`

Then install dependencies:

```
cd ..
rosdep install --from-paths src -i
sudo apt install python3-catkin-tools
```

Then build the workspace:

`catkin build`

Next, make sure to install the support libraries for MAVROS:

`sudo /opt/ros/noetic/lib/mavros/install_geogrpahiclib_datasets.sh`

## Install Ardusub Software-In-The-Loop ##

- SITL: Follow the installation instructions [Starting Here, and Be Sure to Follow Links](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html).
	- NOTE: Follow "Cloning with the command line" section for minimum fuss. When moving on to the [BUILD.md](https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md) section, you do not have to clone ardupilot again. 
	
- ArduSub Configuration: 

	```
	./waf configure --board sitl
	./waf sub
 	```

## Install QGroundControl ##

- APP: Install the application [Here](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html), follow instructions for Ubuntu Linux.

## Test ArduSub SITL and QGroundControl ##

NOTE: For this you will need access to a gamepad joystick (XBox, Logitech, Nintendo Switch Pro, Playstation etc.), or a joystick-keyboard emulator for linux such as [wejoy](https://github.com/Vantskruv/wejoy).

- ArduSub SITL: Follow the instructions [Here](https://www.ardusub.com/developers/sitl.html) and launch a SITL simulation.
- QGroundControl: Double Click on APP, configure joystick.

## Test ArduSub SITL and MAVROS ##

** NOTE, this is still a work in progress, the base_link -> thruster TFs are a little strange for some reason but this is purely cosmetic.**

- Launch an instance of ArduSub simulation

	`sim_vehicle.py -v ArduSub -l 55.60304,12.808937,0,0 --map --console`

- Launch MAVROS 

	- with joystick: `roslaunch bluerov2_bringup bringup_ardusub_sitl.launch`

	- with keyboard: `roslaunch bluerov2_bringup bringup_ardusub_sitl.launch use_joystick:=false`

- With Gazebo Camera Simulation

	- with joystick: `roslaunch bluerov2_bringup bringup_ardusub_sitl.launch gazebo:=true`

	- with keyboard: `roslaunch bluerov2_gazebo start_ardusub_sitl_demo.launch gazebo:=true use_joystick:=false`
