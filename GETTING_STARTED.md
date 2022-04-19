# Autonomous Marine Systems Software Setup Guide #

The following steps are a checklist for ensuring you are able to use the open-source software that is used within this course.

## Install Ubuntu ##

You may skip this step if Ubuntu 20.04 is already installed on your machine.

- OS: Ubuntu 20.04, see [Download Page](https://releases.ubuntu.com/20.04/) and [This Installation Guide for Dual Boot](https://help.ubuntu.com/community/WindowsDualBoot)
- Configure Wi-Fi: If using DTU's DTUSecure or Eduroam WiFi networks, then follow the configuration instructions [Here](https://itswiki.compute.dtu.dk/index.php/DTUsecure_WiFi)

## Quick Install ##

Run the script [convenient_installer.sh](convenient_installer.sh):

`./convenient_installer.sh`

**Note**: If the installer fails for some reason (like a missing dependency), please let me know.

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
python -m pip install requests-futures bluerobotics-ping
```

Then build the workspace:

```
catkin init
catkin config --install
catkin build
```

**Note**: If you want to develop, then it is more convenient to not use the install space,
so change to `catkin config --no-install` to prevent installation step.

Make sure to source your workspace in your `~/.bashrc file`.

```
echo "source /your/workspace/path/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Next, make sure to install the support libraries for MAVROS:

`sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh`

## Install Ardusub Software-In-The-Loop ##

- SITL: Follow the installation instructions [Starting Here, and Be Sure to Follow Links](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html).
	- NOTE: Follow "Cloning with the command line" section for minimum fuss. I suggest cloning the [Ardusub 4.0.3 tag](https://github.com/ArduPilot/ardupilot/releases/tag/ArduSub-4.0.3).
	  When moving on to the [BUILD.md](https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md) section, you do not have to clone ardupilot again. 
	
- ArduSub Configuration: 

	```
	cd ardupilot
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

See [Tutorial 1](documentation/TUTORIAL-01.md).
