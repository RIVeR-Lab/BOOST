#!/bin/bash

source /opt/ros/noetic/setup.bash

# Rosserial USB port
sudo chmod uga=rw /dev/ttyACM0

roslaunch ./ros1_keyboard_teleop.launch
