#!/bin/bash
######################################################################################
# ROS2 (for testing)
#
# This stack will consist of mock_api and ROS2
#
# To install
#    ./setup_testing.sh
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
sudo PBR_VERSION=$(cd ~/mini_pupper_bsp; ./get-version.sh) pip install ~/mini_pupper_bsp/mock_api

git clone https://github.com/Tiryoh/ros2_setup_scripts_ubuntu.git
~/ros2_setup_scripts_ubuntu/ros2-humble-ros-base-main.sh
source /opt/ros/humble/setup.bash

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/mangdangroboticsclub/mini_pupper_ros.git -b ros2
git clone --recursive https://github.com/chvmp/champ.git -b ros2

cd ~/ros2_ws
touch src/champ/champ_description/AMENT_IGNORE
touch src/champ/champ_gazebo/AMENT_IGNORE
touch src/champ/champ_navigation/AMENT_IGNORE
touch src/mini_pupper_ros/mini_pupper_gazebo/AMENT_IGNORE
touch src/mini_pupper_ros/mini_pupper_navigation/AMENT_IGNORE

rosdep install --from-paths src --ignore-src -r -y --skip-keys=joint_state_publisher_gui --skip-keys=rviz2
sudo apt-get -y install ros-humble-teleop-twist-keyboard
MAKEFLAGS=-j1 colcon build --executor sequential

# install service
cd ~
sudo cp $BASEDIR/robot.service /etc/systemd/system/
sudo mkdir -p /var/lib/minipupper/
sudo cp $BASEDIR/run.sh /var/lib/minipupper/
sudo cp $BASEDIR/show_ip.py /var/lib/minipupper/
sudo systemctl daemon-reload
sudo systemctl enable robot

echo "setup.sh finished at $(date)"
