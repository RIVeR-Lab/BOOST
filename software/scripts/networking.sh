# this script will laaunch the ros1_bridge and rosserial_python on the JETSON
echo "Launching roscore, ROS1 Bridge and rosserial on Jetson"
source /opt/ros/galactic/setup.bash
source /opt/ros/noetic/setup.bash
roscore & sleep 2 && rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600 & sleep 2 && ros2 run ros1_bridge dynamic_bridge --bridge-all-1to2-topics

# start roscore.. Sleep 2 seconds .. start rosserial ... sleep 2 seconds ... run ros bridge