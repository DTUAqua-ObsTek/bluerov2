# Mission Planning Tutorial #

In [tutorial 1](TUTORIAL-01.md) you were introduced to the UUV Simulator plugins for Gazebo and the BlueROV2.
 In this tutorial we'll integrate the simulator with an industry standard mission planning framework: [Neptus](https://github.com/LSTS/neptus).

## Pre-requisites ##

Please make sure you are running Ubuntu 18.04 (VM or otherwise) with ROS Melodic Morena installed. 
Please note that the packages are still only compatible with Python 2.7, just make sure you have it installed and make sure python routes to python2.7:

```
sudo apt install ros-melodic-ros-base --reinstall
```

**Note**: You will probably need the following as well:

```
sudo apt install ros-melodic-geographic-info ros-melodic-mavros
python -m pip install --upgrade scipy
```

### Installing IMC to ROS Bridge ###

The Inter Module Communication (IMC) API is a messaging protocol used by Neptus, this needs to be bridged to the ROS messaging protocol. 
There is one good bridge available from [SMarC](https://smarc.se/), please clone the fork into your ROS workspace

Navigate to the ROS workspace (called **catkin_ws** here) where you want to install the bluerov2 stack, then clone another ROS package into `catkin_ws/src` and build the packages.

```
git clone -b noetic-devel https://github.com/FletcherFT/imc_ros_bridge.git
cd ..
catkin_make
```

There may also be some other missing packages, pay attention to errors in catkin_make or during run time.

### Updating your bluerov2 repository ###

The bluerov2 repository has been updated substantially to run with neptus. Please pull changes as follows

```
roscd bluerov2_control/..
git pull origin master
```

### Installing and Starting Neptus ###

First, install dependencies: `sudo apt install openjdk-11-jdk`.
Second, make sure there are no other Javas in your system, you might have a `jre-default` installed by default. Use `apt remove ...` to remove it (See Troubleshooting for more details).

**Neptus is not a ROS package, so you clone into your home directory (or wherever you have write access)**

Clone and build Neptus at a known-working commit:
```
git clone https://github.com/LSTS/neptus.git
cd neptus
git checkout 38c7f41a9885c6b059f79b38861edb4b7b67511b
./gradlew
```

The built Neptus executable is now available and can be executed via ```./neptus``` from the neptus root directory.

Next you should include the bluerov vehicle definition file into Neptus so that it knows what the vehicle is capable of.

```
roscp bluerov2_neptus 00-bluerov2-1.nvcl ~/neptus/vehicles-defs/.
```

If you are already running neptus, restart the program to load the new vehicle profle.

[Neptus Startup](doc/imgs/neptus_001.png)

Adjust the map overlay by right-clicking somewhere on the map and choosing "Choose Visible World Map":

[World Map](doc/imgs/neptus_002.png)

Load in something fairly light-weight like OpenStreetMap:

[OpenStreetMap](doc/imgs/neptus_003.png)

The default mission is based in Portugal, first adjust the home point by right-clicking the GPS reference on the right panel 
and choosing "View/Edit Home Reference". Change latitude and longitude to 55.603036 and 12.8089368 respectively.

[Edit Home Point](docs/imgs/neptus_004.png)

Then right-click somewhere on the map and choose "Center map in -> Home Reference"

## Checking Your Setup ##

### ROS Check ###

You can launch the stack from the bluerov2_bringup package:

```
roslaunch bluerov2_bringup bringup_neptus_gazebo.launch rviz_on:=True gui_on:=True
```

Any errors should hopefully turn up at this point, you may have to install some additional ros packages:

- ros-melodic-geodesy
- ros-melodic-geographic-msgs

Gazebo will warn you that pymap3d is not installed, please **do not** install this package as it messes with the geodetic libraries used by uuv_simulator.

### Neptus Integration Check ###

Run neptus: `~/neptus/neptus`

Next, with the bluerov2 ROS stack running, check that ROS is receiving heartbeats from Neptus via IMC:

```
rostopic echo /bluerov2/imc/imc_heartbeat
```

Then, check the bluerov2 is visible on neptus near the home point and facing East:

[bluerov2 visible](doc/imgs/neptus_005.png)

You should then be good for the tutorial!

## Troubleshooting ##

If Neptus does not seem to work, try these:
On Ubuntu 18.04 and 20.04, you might have `openjdk-11-jre-headless` and `default-jre` remove them with `apt remove ...` manually and make sure that `apt list --installed | grep jre` only shows one jre, it should look like this:
```
$ apt list --installed | grep jre

WARNING: apt does not have a stable CLI interface. Use with caution in scripts.

openjdk-8-jre-headless/focal-updates,focal-security,now 8u282-b08-0ubuntu1~20.04 amd64 [installed,automatic]
openjdk-8-jre/focal-updates,focal-security,now 8u282-b08-0ubuntu1~20.04 amd64 [installed,automatic]
```
This is a known-working setup as of Mar 2 2021.


Additionally, if you get a "No VTK Java Packages Found" message in Neptus's console AND things are not working, do this(might need to change the java version to the version you have):

`sudo vim /etc/java-8-openjdk/accessibility.properties`
Comment out the following line:
`assistive_technologies=org.GNOME.Accessibility.AtkWrapper`

If you get an error that involves `iced-tea` in the Neptus terminal window when you try to open a console, check that you have openjdk-8-jre. Not _just_ the headless version of it, Neptus needs the head.