#!/bin/bash
######################################################################################
# ROS2
#
# This stack will consist of board support package (mini_pupper_bsp) and ROS2
#
# To install
#    ./setup.sh <SSID> "<your Wifi password>"
######################################################################################

set -e
echo "setup.sh started at $(date)"

###### work in progress #######

### Get directory where this script is installed
BASEDIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# check Ubuntu version
source /etc/os-release

if [[ $UBUNTU_CODENAME != 'jammy' ]]
then
    echo "Ubuntu 22.04 LTS (Jammy Jellyfish) is required"
    echo "You are using $VERSION"
    exit 1
fi

############################################
# wait until unattended-upgrade has finished
############################################
tmp=$(ps aux | grep unattended-upgrade | grep -v unattended-upgrade-shutdown | grep python | wc -l)
[ $tmp == "0" ] || echo "waiting for unattended-upgrade to finish"
while [ $tmp != "0" ];do
sleep 10;
echo -n "."
tmp=$(ps aux | grep unattended-upgrade | grep -v unattended-upgrade-shutdown | grep python | wc -l)
done

cd ~
git clone https://github.com/mangdangroboticsclub/mini_pupper_bsp.git
sudo apt-get update
sudo apt-get -y install python3 python3-pip python-is-python3 python3-venv python3-virtualenv
./mini_pupper_bsp/install.sh

# sudo PBR_VERSION=$(cd ~/mini_pupper_bsp; ./get-version.sh) pip install ~/mini_pupper_bsp/Python_Module

git clone https://github.com/Tiryoh/ros2_setup_scripts_ubuntu.git
~/ros2_setup_scripts_ubuntu/ros2-humble-ros-base-main.sh
source /opt/ros/humble/setup.bash

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/mangdangroboticsclub/mini_pupper_ros.git -b ros2
vcs import < mini_pupper_ros/.minipupper.repos --recursive
# compiling gazebo and cartographer on Raspberry Pi is not recommended
touch champ/champ/champ_gazebo/AMENT_IGNORE
touch champ/champ/champ_navigation/AMENT_IGNORE
touch mini_pupper_ros/mini_pupper_gazebo/AMENT_IGNORE
touch mini_pupper_ros/mini_pupper_navigation/AMENT_IGNORE

# install dependencies without unused heavy packages
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y --skip-keys=joint_state_publisher_gui --skip-keys=rviz2 --skip-keys=gazebo_plugins --skip-keys=velodyne_gazebo_plugins
sudo apt-get install ros-humble-teleop-twist-keyboard
MAKEFLAGS=-j1 colcon build --executor sequential --symlink-install

# install service
cd ~
sudo cp $BASEDIR/robot.service /etc/systemd/system/
sudo mkdir -p /var/lib/minipupper/
sudo cp $BASEDIR/run.sh /var/lib/minipupper/
sudo cp $BASEDIR/show_ip.py /var/lib/minipupper/
sudo systemctl daemon-reload
sudo systemctl enable robot

echo "setup.sh finished at $(date)"
sudo reboot
