# this script will laaunch the ros1_bridge and rosserial_python on the JETSON
echo "Launching roscore, ROS1 Bridge and rosserial on Jetson"
source /opt/ros/galactic/setup.bash
source /opt/ros/noetic/setup.bash
roscore & rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600 & ros2 run ros1_bridge dynamic_bridge --bridge-all-topics & rostopic list & ros2 topic list