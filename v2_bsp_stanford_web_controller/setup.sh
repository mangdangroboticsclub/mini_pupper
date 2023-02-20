#!/bin/bash
######################################################################################
# Stanford
#
# This stack will consist of board support package (mini_pupper_bsp),
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
echo "setup.sh started at $(date)"

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

# check parameters
if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <ssid> <wifi password>"
    exit 1
fi

cd ~
git clone -b mini_pupper_2 https://github.com/mangdangroboticsclub/mini_pupper_bsp.git
git clone https://github.com/mangdangroboticsclub/StanfordQuadruped.git
git clone https://github.com/mangdangroboticsclub/mini_pupper_web_controller.git
./mini_pupper_bsp/install.sh

cd StanfordQuadruped
./install.sh
./configure_network.sh $1 "$2"

cd ~
./mini_pupper_web_controller/webserver/install.sh
echo "setup.sh finished at $(date)"
sudo reboot
