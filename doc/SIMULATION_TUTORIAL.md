# Getting Started With Gazebo Simulation of BlueROV2 (with Neptus Support) #

In [tutorial 1](TUTORIAL-01.md) you were introduced to the UUV Simulator plugins for Gazebo and the BlueROV2.
 In this tutorial we'll integrate the simulator with an industry standard mission planning framework: [Neptus](https://github.com/LSTS/neptus).

## Pre-requisites ##

Please follow the setup instructions in [tutorial 8](TUTORIAL-08.md)

## Gazebo Quick Start ##

From a terminal, run the following:

```
roslaunch bluerov2_bringup bringup_neptus_gazebo.launch rviz_on:=true mode:=altimeter
```

This will launch a gazebo simulation of the peberholm world with a BlueROV2 equipped with a downwards facing altimeter.

## ArduSub Quick Start ##

From a terminal, start the ardusub sitl simulation:

```
sim_vehicle.py -v ArduSub -l 55.60304,12.808937,0,0 --map --console
```

In another terminal, run the following:

```
roslaunch bluerov2_bringup bringup_ardusub_sitl.launch
```