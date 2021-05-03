# Autonomous Marine Systems Software Setup Guide #

The following steps are a checklist for ensuring you are able to use the open-source software that is used within this course.

## Install Ubuntu ##

You may skip this step if Ubuntu 18.04 is already installed on your machine.

- OS: Ubuntu 18.04 (not Ubuntu 20.04), see [Download Page](https://releases.ubuntu.com/18.04/) and [This Installation Guide for Dual Boot](https://help.ubuntu.com/community/WindowsDualBoot)
- Configure Wi-Fi: If using DTU's DTUSecure or Eduroam WiFi networks, then follow the configuration instructions [Here](https://itswiki.compute.dtu.dk/index.php/DTUsecure_WiFi)

## Install Robot Operating System ##

- Melodic Morenia: Follow the installation instructions [Here](http://wiki.ros.org/melodic/installation/Ubuntu), make sure to specify ros-melodic-desktop-full when installing.

### ROS Packages ###

- UUV Simulator: Follow the installation instructions [Here](https://uuvsimulator.github.io/installation/)
- MAVROS: Follow the installation instructions [Here](https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation)
	- NOTE: replace "kinetic" with "melodic" in the apt-get install commands. 
	- NOTE: Make sure to install the GeographicLib datasets after installing mavros! You will have to execute ´´´sudo ./install_geographiclib_datasets.sh"´´´
- BlueROV2 Gazebo and ArduSub Modelling
	1. Follow instructions to create a new ROS workspace [Here (choose Melodic)](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).
	2. Clone this repository into your workspace's src directory (called `catkin_ws/src` in this guide) `git clone https://github.com/FletcherFT/bluerov2.git`
	3. Execute `catkin_make` in the top-level directory of `catkin_ws`.

## Install Ardusub Software-In-The-Loop ##

- SITL: Follow the installation instructions [Starting Here, and Be Sure to Follow Links](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html).
	- NOTE: Follow "Cloning with the command line" section for minimum fuss. When moving on to the [BUILD.md](https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md) section, you do not have to clone ardupilot again. 
	
- ArduSub Configuration: run 

`./waf configure --board sitl`

`./waf sub`

## Install QGroundControl ##

- APP: Install the application [Here](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html), follow instructions for Ubuntu Linux.

## Test ArduSub SITL and QGroundControl ##

NOTE: For this you will need access to a gamepad joystick (XBox, Logitech, Nintendo Switch Pro, Playstation etc.), or a joystick-keyboard emulator for linux such as [QJoyPad](http://qjoypad.sourceforge.net/)

- ArduSub SITL: Follow the instructions [Here](https://www.ardusub.com/developers/sitl.html) and launch a SITL simulation.
- QGroundControl: Double Click on APP, configure joystick.

## Test UUV Simulator ##

- Open ~/.ignition/fuel/config.yaml, replace "api.ignitionfuel.org" with "fuel.ignitionrobotics.org"
- Follow instructions [Here](https://uuvsimulator.github.io/quick_start)

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
