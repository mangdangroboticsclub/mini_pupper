#!/bin/bash

/usr/bin/python3 /var/lib/minipupper/show_ip.py
/bin/bash /var/lib/minipupper/edit_bashrc.sh
source ~/catkin_ws/devel/setup.bash
roslaunch mini_pupper_bringup bringup.launch
