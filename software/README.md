# **Swarm Crawler Software Stack Guide**

#### After plugging in the wifi module into power, wait until you see the swarm_crawler wifi network appear.

WIFI name: **swarm_crawler**

Wifi Password: **capstone**

After connecting to the swarm_crawler, go to 

**http://192.168.8.1,** the password at the dashboard is **capstone,**

Go to the section that allows you to use the wifi module as a reapeter, and select the local wifi network you are on. This will allow you, and all the minibots to be on the same ROS network, and also access the internet. you will also be able to see the IP addresses of all the other devices connected to the network

#### **Add some ease of use to .bashrc. You only need to do this once.**

* *`echo "export ros2=/opt/ros/galactic/setup.bash" >>  ~/.bashrc`*
* *`echo "export ros1=/opt/ros/noetic/setup.bash" >>  ~/.bashrc`*

And from from now on, before you run anything in a terminal in ros1, or ros2 you only need to say  :

* *`source $ros1`*
* *`source $ros2`*

Adding rqt plugins:

* `sudo apt-get install ros-galactic-rqt-*`
* `rqt --force-discover`

## **Runnings Software Stack:**

#### Build Realsense package ( If its your first time setup be sure build the realsense package)

* *`cd software/realsense-ros`*
* *`colcon buildSS`*

#### Boot Up Cameras on Jetson

*In order to run commands on the jetson, you have to be on the same network*

* *`cd software/scripts`*
* In a terminal run :
  * *`./t265.launch.sh & ./d435.launch.sh`*

    * You can also run these independently.
    * *If you go into these scripts, you can edit the id numbers of each device. I will soon add a command to automatically get these IDs.*
* Killing the Camera Nodes:
  * `sudo pkill realsense2_camera_node`

#### Use Bluetooth Controller. Note: this publishes a message to /cmd_vel.

Note: May need to install the following: sudo apt-get iinstall ros-galactic-joy
sudo apt-get install ros-galactic-teleop-twist-joy

* `cd software/scripts`
* `./controller.sh`

#### Launch nav2 stack. (will perform obstacle avoidance, driving to a pose)

*Launch both cameras using the commands above*

* `cd software/scripts`
* `./jetsonnav2.sh`

#### Launch aruco-detection and traversal (will navigate to aruco on the message that the battery is getting low)

* `cd software/scripts`
* `./jetson_aruco.sh`

##### **As of right now, the minibot will start navigating to the aruco tag if it sees it, and the battery is low.**

To indicate Low battery run the following comand in a different terminal:

> `ros2 topic pub /battery_status sensor_msgs/BatteryState '{voltage: 2.16, percentage: 0.24, power_supply_status: 3}'`

## Installing Software Stack:

* `cd software/scripts`
* `./jetson_setup.sh`

## Code Structure -- Ignore everything Below This For Now

TODO

We are currently using ROS Galactic Geochrome.

Installing Realsense Dependencies:

Installing ROS dependencies:

sudo apt-get install ros-galactic-(missing dependency)
example:
sudo apt-get install ros-galactic-nav2

After you do colcon build to build the repo, you can then do
ros2 launch swarm_crawler minibot.launch.py

the new folders such as build, install, and log will be ignored if you decide to push.

**CONTROLLER OPERATION:**
sudo apt-get iinstall ros-galactic-joy
sudo apt-get install ros-galactic-teleop-twist-joy

Joy Node:
ros2 run joy node
nodes being published:
/joy
/joy/set_feedback

ros2 launch teleop_twist_joy teleop-launch.py joy_config:='ps3'
https://index.ros.org/p/teleop_twist_joy/github-ros2-teleop_twist_joy/

Published Topics

    cmd_vel (geometry_msgs/msg/Twist)

additional RQT stuff:
 sudo apt install ros-galactic-rqt*
Today:
lanch joy node to publish twist messages
drive robto in rviz
launch realsense node from within repo.
get jetson networking done

ros2 launch swarm_crawler controller-teleop.launch.py joy_config:='ps3'

ros2 launch realsense2_camera rs_launch.py

D435 Serial number: 830112071549
ros2 launch realsense2_camera rs_launch.py camera:=cam_1 "serial_no:='830112071549'"

T265 Serial Number: 845412111433
ros2 launch realsense2_camera rs_launch.py camera:=cam_2 "serial_no:='845412111433'"

ros2 launch realsense2_camera rs_launch.py camera_name:=minibot_a_t265 camera:=cam_1 "serial_no:='845412111433'"

RQT:

sudo apt-get install ros-galactic-rqt-*

rqt --force-discover
