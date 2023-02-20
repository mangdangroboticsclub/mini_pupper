#!/bin/bash
######################################################################################
# ROS1
#
# This stack will consist of board support package (mini_pupper_bsp),
#    ROS1 and champ
#
# To install
#    ./setup.sh
######################################################################################

set -e
echo "setup.sh started at $(date)"

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

git clone -b mini_pupper_2 https://github.com/mangdangroboticsclub/mini_pupper_bsp.git
git clone https://github.com/Tiryoh/ros_setup_scripts_ubuntu.git
sudo apt-get update
sudo apt-get -y install python3 python3-pip python-is-python3 python3-venv python3-virtualenv
./mini_pupper_bsp/install.sh
~/ros_setup_scripts_ubuntu/ros-noetic-ros-base-main.sh
source /opt/ros/noetic/setup.bash
source /usr/lib/python3/dist-packages/catkin_tools/verbs/catkin_shell_verbs.bash
rosdep update

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone -b ros1 https://github.com/mangdangroboticsclub/minipupper_ros.git
vcs import < minipupper_ros/.minipupper.repos --recursive

# it's not recommend to compile gazebo and cartographer on raspberry pi
touch champ/champ/champ_description/CATKIN_IGNORE
touch champ/champ/champ_gazebo/CATKIN_IGNORE
touch champ/champ/champ_navigation/CATKIN_IGNORE
touch minipupper_ros/mini_pupper_gazebo/CATKIN_IGNORE
touch minipupper_ros/mini_pupper_navigation/CATKIN_IGNORE

# install dependencies without unused heavy packages
rosdep install --from-paths . --ignore-src -r -y --skip-keys=joint_state_publisher_gui --skip-keys=octomap_server
cd ~/catkin_ws
catkin_make

#TODO move this to minipupper_ros repo
# install service
cd ~
sudo cp $BASEDIR/robot.service /etc/systemd/system/
sudo mkdir -p /var/lib/minipupper/
sudo cp $BASEDIR/run.sh /var/lib/minipupper/
sudo cp $BASEDIR/show_ip.py /var/lib/minipupper/
sudo cp $BASEDIR/edit_bashrc.sh /var/lib/minipupper/
sudo systemctl daemon-reload
sudo systemctl enable robot

echo "setup.sh finished at $(date)"
sudo reboot
