# BlueROV2 ROS Simulation

## What is this Repository? ##

This repository contains the robot description and necessary launch files to simulate the BlueROV2 (unmanned underwater vehicle) on [Unmanned Underwater Vehicle Simulator (UUV Simulator)](https://github.com/uuvsimulator/uuv_simulator). It's also possible to simulate the BlueROV2 using [ArduSub Software-In-The-Loop](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html) with [mavros](http://wiki.ros.org/mavros) and other ROS packages. joystick interaction and video streaming capture with opencv based on  package from BlueRobotics.

This work consolidates the contributions from [Ingeniarius, Lda.](http://ingeniarius.pt/) and [Instituite of Systems and Robotics University of Coimbra](https://www.isr.uc.pt/) within the scope of MS thesis "Localization of an unmanned underwater vehicle using multiple water surface robots, multilateration, and sensor data fusion". Source code is derived from [bluerov_ros_playground](https://github.com/patrickelectric/bluerov_ros_playground).

The [Unmanned Underwater Vehicle Simulator](https://uuvsimulator.github.io/) (UUV-Sim) is also supported in this repository. The vehicle model generated at [bluerov2](https://github.com/fredvaz/bluerov2), has been adapted to work with Melodic relase of UUV-Sim

This repository is intended as introductory marine vehicle simulation software for the 2021 Spring Special Course "Autonomous Marine Robotics" hosted at the Technical University of Denmark.

<p align="center">
  <img src="doc/imgs/bluerov2_uuv_simulator.png">
</p>


## Getting Started ## 

The repository was built and tested on a fresh installation of Ubuntu 18.04. The [Getting Started Guide](GETTING_STARTED.md) will assist with installation of dependencies and testing. 
Tutorials [01](TUTORIAL-01.md) and [08](TUTORIAL-08.md) are intended for students enrolled in Autonomous Marine Robotics special course at DTU. The [simulation tutorial](SIMULATION_TUTORIAL.md) gives a quick start guide for people interested in using the simulations.

## Additional Links ##

- [MAVROS Documentation](https://github.com/mavlink/mavros/blob/master/mavros/README.md)
- [UUV Simulator Documentation](https://uuvsimulator.github.io/packages/uuv_simulator/intro/)


## Topics

If you need more information about the topics and what you can access, take a look [here](doc/topics_and_data.md).


## License

BlueROV2 package is open-sourced under the Apache-2.0 license. See the
[LICENSE](LICENSE) file for details.
