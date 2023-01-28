#!/bin/bash

if test=$(ip a show dev wlan0 | grep -v inet6 | grep inet); then
    IP=$(echo $test | awk -F' ' '{print $2}' | awk -F'/' '{print $1}')
elif test=$(ip a show dev eth0 | grep -v inet6 | grep inet); then
    IP=$(echo $test | awk -F' ' '{print $2}' | awk -F'/' '{print $1}')
else
    IP=127.0.0.1
fi

JUPYTER_TOKEN=$(sed -e "s/://g" /sys/class/net/wlan0/address)

echo "To access the notebook use:" | sudo tee /etc/motd
echo "##################################################" | sudo tee -a /etc/motd
echo "http://$IP:8888/lab?token=$JUPYTER_TOKEN" | sudo tee -a /etc/motd
echo "##################################################" | sudo tee -a /etc/motd
