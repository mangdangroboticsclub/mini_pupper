#!/bin/bash
######################################################################################
# ROS1
#
# This will install ROS1 amd mini pupper packages for use on a PC or VM
#
# To install
#    ./setup_pc.sh
######################################################################################

set -e

### Get directory where this script is installed
BASEDIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# check Ubuntu version
source /etc/os-release

if [[ $UBUNTU_CODENAME != 'focal' ]]
then
    echo "Ubuntu 20.04 LTS (Focal Fossa) is required"
    echo "You are using $VERSION"
    exit 1
fi

cd ~
git clone https://github.com/mangdangroboticsclub/mini_pupper_bsp.git
git clone https://github.com/Tiryoh/ros_setup_scripts_ubuntu.git
sudo apt-get update
sudo apt-get -y install python3 python3-pip python-is-python3 python3-venv python3-virtualenv
sudo pip install -e ~/mini_pupper_bsp/mock_api
~/ros_setup_scripts_ubuntu/ros-noetic-desktop-main.sh
source /opt/ros/noetic/setup.bash
source /usr/lib/python3/dist-packages/catkin_tools/verbs/catkin_shell_verbs.bash
rosdep update

sudo apt-get install -y ninja-build stow
mkdir -p ~/carto_ws/src
cd ~/carto_ws
vcs import src --input https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall
# Hot fix for Ubuntu 20.04. See https://github.com/cartographer-project/cartographer_ros/pull/1745
sed -i -e "s%<depend>libabsl-dev</depend>%<\!--<depend>libabsl-dev</depend>-->%g" src/cartographer/package.xml
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
src/cartographer/scripts/install_abseil.sh
dpkg -l | grep ros-noetic-abseil-cpp && sudo apt-get remove ros-noetic-abseil-cpp
catkin_make_isolated --install --use-ninja
source install_isolated/setup.bash

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone -b ros1 https://github.com/mangdangroboticsclub/minipupper_ros.git
vcs import < minipupper_ros/.minipupper.repos --recursive

# install dependencies
rosdep install --from-paths . --ignore-src -r -y --skip-keys=joint_state_publisher_gui --skip-keys=octomap_server
cd ~/catkin_ws
catkin_make

#TODO move this to minipupper_ros repo
# install service
cd ~
sudo cp $BASEDIR/robot.service /etc/systemd/system/
sudo mkdir -p /var/lib/minipupper/
sudo cp $BASEDIR/run.sh /var/lib/minipupper/
sudo systemctl daemon-reload
sudo systemctl enable robot

echo "setup_pc.sh executed."
