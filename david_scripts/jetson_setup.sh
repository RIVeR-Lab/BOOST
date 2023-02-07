#!/usr/bin/env bash
echo "Running setup..."
echo "Installing ROS1 Noetic Bare Bones..."

# From here: http://wiki.ros.org/noetic/Installation/Ubuntu
# Setup sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# Set Keys
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
# Install ROS
sudo apt update
sudo apt install ros-noetic-ros-base

# Install Other ROS Packages
sudo apt satisfy ros-noetic-rosserial
sudo apt install ros-noetic-rosserial
