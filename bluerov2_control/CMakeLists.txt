cmake_minimum_required(VERSION 3.0.2)
project(bluerov2_control)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bluerov2_control
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY
  launch
  params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  nodes/controller_4dof
  nodes/keyboard_joy
  nodes/mavros_interface
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
