#!/bin/bash

if test=$(ip a show dev wlan0 | grep -v inet6 | grep inet); then
    IP=$(echo $test | awk -F' ' '{print $2}' | awk -F'/' '{print $1}')
elif test=$(ip a show dev eth0 | grep -v inet6 | grep inet); then
    IP=$(echo $test | awk -F' ' '{print $2}' | awk -F'/' '{print $1}')
else
    IP=127.0.0.1
fi

sed -i "s/export ROS_IP=.*$/export ROS_IP=$IP/"  ~/.bashrc
