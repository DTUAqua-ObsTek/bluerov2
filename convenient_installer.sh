#!/usr/bin/env bash

INSTALL_PATH=$HOME/rov_ros_ws
sudo apt update
echo "Installing ROS..."
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
grep -qxF 'source /opt/ros/noetic/setup.bash' ~/.bashrc || echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source /opt/ros/noetic/setup.bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-catkin-tools python3-pip python3-future python3-pymavlink
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3 10
sudo rosdep init
rosdep update
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh 
echo "ROS Installed."

echo "Setting up bluerov2 ROS workspace..."
mkdir -p $INSTALL_PATH/src
cd $INSTALL_PATH/src
catkin_init_workspace
git clone https://github.com/DTUAqua-ObsTek/bluerov2.git
cd bluerov2
git submodule init
git submodule update

cd $INSTALL_PATH
catkin init
rosdep install --from-paths src -i
catkin config --merge-devel
catkin build
grep -qxF "source $INSTALL_PATH/devel/setup.bash" ~/.bashrc || echo "source $INSTALL_PATH/devel/setup.bash" >> ~/.bashrc

echo "bluerov2 workspace ready."

echo "Installing QGroundControl..."

sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
. ~/.profile

cd ~/Desktop/
if [ ! -e "~/Desktop//QGroundControl.AppImage" ]; then
       wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
       chmod +x ./QGroundControl.AppImage
fi

echo "QGroundControl Installed."

cd ~
if [ ! -d "~/ardupilot" ]; then
  echo "Installing ArduSub SITL"
  git clone https://github.com/ArduPilot/ardupilot.git -b ArduSub-stable
  cd ardupilot
  git submodule update --init --recursive
  sudo apt install python3-wxgtk4.0
  ./waf configure --board sitl
  ./waf sub
  grep -qxF "export PATH=$PATH:$HOME/ardupilot/Tools/autotest" ~/.bashrc || echo "export PATH=$PATH:$HOME/ardupilot/Tools/autotest" >> ~/.bashrc
  echo "Ardusub Installed."
fi

# Unlisted 3rd party python libraries
python3 -m pip install requests-futures bluerobotics-ping pymavlink mavproxy
python3 -m pip install --upgrade scipy

echo "BlueROV2 stack installed"
