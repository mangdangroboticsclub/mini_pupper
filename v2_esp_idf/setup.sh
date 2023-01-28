#!/bin/bash
######################################################################################
# ESP-IDF
#
# This stack will install the ESP32 development environment
#
# To install
#    ./setup.sh
######################################################################################

set -e
echo "setup.sh started at $(date)"

###### work in progress #######
cd ~
git clone https://github.com/hdumcke/multipass-orchestrator-configurations.git
cd ~/multipass-orchestrator-configurations/esp-idf
./build.sh

echo "setup.sh finished at $(date)"
sudo reboot
