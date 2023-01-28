#!/bin/bash

source ~/.bashrc
source ~/catkin_ws/devel/setup.bash
roslaunch mini_pupper_bringup bringup.launch lidar_connected:=false
