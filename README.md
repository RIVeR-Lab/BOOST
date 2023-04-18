# Autonomous Navigation & Docking for Distributed Robots
* Aka Swarm Crawler
* Aka BOOST: Battery Optimized On-site Swapping Technology
* Making swarms crawl

# Where Software Is

`/hubbot_software/`
* The software running on the NVidia Jetson Nano on the HubBot
* Includes the main ROS2 node running on the HubBot in `hubbot_software\ros2_ws\src\py_hubbot_main_controller_node\py_hubbot_main_controller_node`

`/minibot_hubbot_firmware/charge_controller_firmware/`
* Code running on the Arduino Nano on the HubBot that controls the LEDs and reads battery module continuity and voltage and sends to Jetson Nano over UART.

`/minibot_hubbot_firmware/firmwarePioCube/`
* Code running on the STM32F466RE NUCLEO on our Custom PCB.
* This board is identical on the 2x minibots and on the hubbot.
* Controls IMU, GPS, motor control, encoders, battery voltage.

# David Useful Commands
> ros2 topic pub /chatter std_msgs/String "data: Hello ROS Developers"

>ros2 topic pub /hub_stat std_msgs/Int32 "data: 4"

#### rosserial with ROS1
- http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup
- Installed ROS1 with WSL Ubuntu on windows
  - http://wiki.ros.org/noetic/Installation/Ubuntu
  - Installed ROS-Base: sudo apt install ros-noetic-ros-base
- Followed the below to build the rosserial/ros_lib library files and msgs.
  - http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

##### To find the USB Port and get Nucleo board USB into WSL
  - https://learn.microsoft.com/en-us/windows/wsl/connect-usb
  - PS> usbipd wsl list
  - PS> usbipd wsl attach -i 0483:374b
  - wsl> lsusb
  - wsl> sudo chmod 777 /dev/ttyACM0

##### To give user permission to /dev/ttyUSBx, ttyACMx, and ttySx devices
- These devices are in the dialout group, so add your user to that group
  > $ sudo adduser myusername dialout

##### Getting Data into ROS1 from MCU
- http://wiki.ros.org/rosserial_arduino/Tutorials/Hello%20World
- run roscore in one terminal
  - wsl> roscore
- Run serial_node.py in another terminal
  - wsl> rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600
  - This will get all topics from the MCU.
- Should now see the topic in ROS1 in another terminal
  - wsl> rostopic list
  - Can echo with wsl> rostopic echo <topic>

#### Using ros1_bridge
- https://github.com/ros2/ros1_bridge
- Start ROS1 core wsl> roscore
  - If not done already.
- Need to source both ROS1 and then ROS2 setup.bash's
- wsl> ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
  - This will bring in all topics from ROS1 to ROS2 and all topics from ROS2 to ROS1.

#### teleop twist keyboard
- http://wiki.ros.org/stdr_simulator/Tutorials/Teleop%20with%20teleop_twist_keyboard
- ROS1: >rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=cmd_vel
- ROS2: >ros2 run teleop_twist_keyboard teleop_twist_keyboard


#### ROSSerial TroubleShooting
#### [ERROR] [1677681154.605087]: Unable to sync with device; possible link problem or link software version mismatch such as hydro rosserial_python with groovy Arduino
  * Make sure Baud rates are matching
  * Make sure that LOGGING is off on the nucleo so that ONLY ROSserial stuff is is sent out over the serial port connected to the Jetson.

#### If serial isn't getting to jetson, but is getting to laptop
* Make sure that the power configuration is correct on the NUCLEO board.
* i.e. since we are using the STLink as a serial to USB adapter, the STLINK must be powered when the MCU is booted.
* Make sure that we are powering USB through NUCLEO from the jetson.
  * And if we are using external 5v to power NUCLEO that the correct jumpers are set.


### For running ROSserial and ros1_bridge
>source ros1
>roscore
>rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600

>source ros2
>source ros1
>ros2 run ros1_bridge dynamic_bridge --bridge-all-topics