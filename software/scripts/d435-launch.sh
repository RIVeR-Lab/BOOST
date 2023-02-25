sleep 1
cd ..
source /opt/ros/galactic/setup.bash
source realsense-ros/install/setup.bash
# initial_reset:=true 
# ros2 launch realsense2_camera rs_launch.py pointcloud.stream_index_filter:=-1 enable_sync:=true camera_name:=minibot_a_d435 "serial_no:='830112071549'"
#add stuff for the robot id


# if statement that checks if the current path contains the word "jetson" if it does, execute the code at line 8, else execute the code at line 6
if [[ $PWD == *"jetson"* ]]; then
    echo 123
    ros2 run realsense2_camera realsense2_camera_node --ros-args -p spatial_filter.enable:=true -p temporal_filter.enable:=true -p pointcloud.enable:=true -r __ns:=/minibot_a_d435 
else
    echo 'RUNNing '
    ros2 launch realsense2_camera rs_launch.py pointcloud.stream_index_filter:=-1 enable_sync:=true camera_name:=minibot_a_d435 "serial_no:='830112071549'"
fi


