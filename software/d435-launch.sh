sleep 1
source /opt/ros/galactic/setup.bash
source realsense-ros/install/setup.bash
# initial_reset:=true 
ros2 launch realsense2_camera rs_launch.py pointcloud.stream_index_filter:=-1 enable_sync:=true camera_name:=minibot_a_d435 "serial_no:='830112071549'"
