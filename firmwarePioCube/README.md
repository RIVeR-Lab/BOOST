## rosserial with ROS1
- http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup
- Installed ROS1 with WSL Ubuntu on windows
  - http://wiki.ros.org/noetic/Installation/Ubuntu
  - Installed ROS-Base: sudo apt install ros-noetic-ros-base
- Followed the below to build the rosserial/ros_lib library files and msgs.
  - http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

### To find the USB Port and get Nucleo board USB into WSL
  - https://learn.microsoft.com/en-us/windows/wsl/connect-usb
  - PS> usbipd wsl list
  - PS> usbipd wsl attach -i 0483:374b
  - wsl> lsusb
  - wsl> sudo chmod 777 /dev/ttyACM0

### Getting Data into ROS1 from MCU
- http://wiki.ros.org/rosserial_arduino/Tutorials/Hello%20World
- run roscore in one terminal
  - wsl> roscore
- Run serial_node.py in another terminal
  - wsl> rosrun rosserial_python serial_node.py /dev/ttyACM0
  - Can also set baudrate in this command
  - This will get all topics from the MCU.
- Should now see the topic in ROS1 in another terminal
  - wsl> rostopic list
  - Can echo with wsl> rostopic echo <topic>

## Using ros1_bridge
- https://github.com/ros2/ros1_bridge
- Start ROS1 core wsl> roscore
  - If not done already.
- Need to source both ROS1 and then ROS2 setup.bash's
- wsl> ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
  - This will bring in all topics from ROS1 to ROS2 and all topics from ROS2 to ROS1.