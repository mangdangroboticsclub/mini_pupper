#/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 terminal<1> where n=1..3>"
    exit 1
fi

# Terminal 1
if [ "$1" == "terminal1" ]; then
    source ~/ros2_ws/install/setup.bash
    ros2 launch mini_pupper_gazebo gazebo.launch.py
fi

# Terminal 2
if [ "$1" == "terminal2" ]; then
    source ~/ros2_ws/install/setup.bash
    rviz2 -d src/champ/champ_description/rviz/urdf_viewer.rviz
fi

# Terminal 3
if [ "$1" == "terminal3" ]; then
    source ~/ros2_ws/install/setup.bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    # Then control robot dog with your keyboard
fi
