Code Structure

Steps to this to work on your own computer... 
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

CONTROLLER OPERATION:
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
