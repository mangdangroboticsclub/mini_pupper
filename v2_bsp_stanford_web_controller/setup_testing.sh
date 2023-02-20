#!/bin/bash
######################################################################################
# Stanford (test environment)
#
# This stack will consist of mock_api
#    the StanfordQuadruped controller and the mini_pupper_web_controller
#
# After installation you can either pair a supported PS4 joystick or
# point your web browser to 
#   http://x.x.x.x:8080
# where x.x.x.x is the IP address of your Mini Pupper as displayed on the LCD screen
#
# To install
#    ./setup.sh <SSID> "<your Wifi password>"
######################################################################################

set -e

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

cd ~
git clone -b mini_pupper_2 https://github.com/mangdangroboticsclub/mini_pupper_bsp.git
git clone https://github.com/mangdangroboticsclub/StanfordQuadruped.git
git clone https://github.com/mangdangroboticsclub/mini_pupper_web_controller.git
sudo apt-get update
sudo apt-get -y install python3 python3-pip python-is-python3 python3-venv python3-virtualenv
sudo pip install -e ~/mini_pupper_bsp/mock_api

cd StanfordQuadruped
./install.sh
cp /etc/netplan/50-cloud-init.yaml /tmp/mini-pupper.yaml
sudo rm /etc/netplan/50-cloud-init.yaml
sed -i "/version/d" /tmp/mini-pupper.yaml
cat >> /tmp/mini-pupper.yaml << EOF
    bridges:
        br0:
            addresses: [10.0.0.10/24]
            parameters:
                stp: true
                forward-delay: 4
            dhcp4: false
            optional: true
    version: 2
EOF
sudo cp /tmp/mini-pupper.yaml /etc/netplan/

cd ~
./mini_pupper_web_controller/webserver/install.sh

# prepare for testing
sudo systemctl stop robot
sudo rm /tmp/Display.log
sudo pip install coverage
# we are running in a virtual machine and are not using a Python virtual environment
sed -i "/source  ~\/mini_pupper_venv\/bin\/activate/d" ~/StanfordQuadruped/tests/run_all_tests.sh
