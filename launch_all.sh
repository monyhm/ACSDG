#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/acsdg_ws/install/setup.bash
ros2 launch acsdg_sensors sensors.launch.py &
sleep 2
ros2 launch acsdg_c2 c2.launch.py &
sleep 2
ros2 launch acsdg_ai ai.launch.py &
sleep 3
ros2 launch acsdg_dashboard dashboard.launch.py
