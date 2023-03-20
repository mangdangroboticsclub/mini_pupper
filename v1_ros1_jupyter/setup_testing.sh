#!/bin/bash
######################################################################################
# ROS1 and Jupyter notebooks
#
# This stack will consist of mock_api
#     ROS1 and Jupyter notebook
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

[ -d "./mini_pupper_bsp" ] || git clone https://github.com/mangdangroboticsclub/mini_pupper_bsp.git
[ -d "./ros_setup_scripts_ubuntu" ] || git clone https://github.com/Tiryoh/ros_setup_scripts_ubuntu.git
sudo apt-get update
sudo apt-get -y install python3 python3-pip python-is-python3 python3-venv python3-virtualenv
sudo PBR_VERSION=$(cd ~/mini_pupper_bsp; ./get-version.sh) pip install ~/mini_pupper_bsp/mock_api
~/ros_setup_scripts_ubuntu/ros-noetic-ros-base-main.sh
sudo apt install -y ros-noetic-rosbridge-server ros-noetic-tf2-web-republisher
sudo apt install -y ros-noetic-depthai-bridge ros-noetic-depthai-examples
source /opt/ros/noetic/setup.bash
source /usr/lib/python3/dist-packages/catkin_tools/verbs/catkin_shell_verbs.bash
rosdep update

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
[ -d "./minipupper_ros" ] || git clone -b ros1 https://github.com/mangdangroboticsclub/minipupper_ros.git
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

sudo apt install -y docker.io
sudo usermod -aG docker ubuntu
mkdir -p ~/dev
cd ~/dev/
[ -d "./mini-pupper-jupyter-notebooks" ] || git clone https://github.com/Tiryoh/mini-pupper-jupyter-notebooks.git

# patch to allow to run in Multipass VM
sed -i "s/wlan0/enp0s2/" mini-pupper-jupyter-notebooks/run.sh
sed -i "s/wlan0/enp0s2/" mini-pupper-jupyter-notebooks/notebook/*.ipynb

# Build Docker image
cd ~/dev/mini-pupper-jupyter-notebooks/docker/conda-jupyter-ros
sed -i "s/aarch/x86_/" Dockerfile
sudo ./build.sh

# call as root as docker group not activates before logout/login
#sudo docker pull ghcr.io/tiryoh/conda-jupyter-ros:noetic

# udev rules for OAL-D
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules

cd ~
sudo cp $BASEDIR/robot.service /etc/systemd/system/
sudo cp $BASEDIR/jupyter.service /etc/systemd/system/
sudo mkdir -p /var/lib/minipupper/
sudo cp $BASEDIR/run.sh /var/lib/minipupper/
sudo cp $BASEDIR/run_jupyter.sh /var/lib/minipupper/
sudo cp $BASEDIR/show_ip.py /var/lib/minipupper/
sudo cp $BASEDIR/show_jupyter_url.sh /var/lib/minipupper/
sudo cp $BASEDIR/edit_bashrc.sh /var/lib/minipupper/
# patch to allow to run in Multipass VM
sudo sed -i "s/wlan0/enp0s2/" /var/lib/minipupper/*
sudo systemctl daemon-reload
sudo systemctl enable robot
sudo systemctl enable jupyter

echo "setup.sh finished at $(date)"
sudo reboot
