#!/usr/bin/env bash
echo "Running setup..."
echo "Installing ROS1 Noetic Bare Bones..."

# From here: http://wiki.ros.org/noetic/Installation/Ubuntu
# Setup sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# Set Keys
sudo apt -y install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
# Install ROS
sudo apt update
sudo apt -y install ros-noetic-ros-base

# Install Other ROS1 Packages
sudo apt -y satisfy ros-noetic-rosserial
sudo apt -y install ros-noetic-rosserial
sudo apt -y install ros-noetic-teleop-twist-keyboard

echo "Installing ROS2 galactic Bare Bones..."
# Update time locale
sudo apt update && sudo apt -y install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# First ensure that the Ubuntu Universe repository is enabled.
sudo apt -y install software-properties-common
sudo add-apt-repository universe

# Now add the ROS 2 GPG key with apt.
sudo apt update && sudo apt -y install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Then add the repository to your sources list.
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade

# Bares Bones
sudo apt -y install ros-galactic-ros-base python3-argcomplete

# Install Other ROS2 Packages
sudo apt -y install ros-galactic-ros1-bridge






