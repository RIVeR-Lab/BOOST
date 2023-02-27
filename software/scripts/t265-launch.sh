sleep 1
cd ..
source /opt/ros/galactic/setup.bash
source realsense-ros/install/setup.bash
#launch t265 first
# topic_odom_in:='/test_odom' 
# initial_reset:=true 

# create a command line argument named "reset", and include it in the launch file
    # ros2 launch realsense2_camera rs_launch.py  initial_reset:=true  camera_name:=minibot_a_t265 "serial_no:='845412111433'" enable_gyro:=True enable_acel:=True enable_pose:=True 
    # ros2 launch realsense2_camera rs_launch.py enable_sync:=true  unite_imu_method:=2 camera_name:=minibot_a_t265 "serial_no:='845412111433'"  enable_gyro:=True enable_acel:=True enable_pose:=True # "unite_imu_method:='2'"
ros2 launch realsense2_camera rs_launch.py  camera_name:=minibot_a_t265 serial_no:="'845412111433'"   enable_gyro:=True enable_accel:=True enable_pose:=True "unite_imu_method:='1'"
     # "unite_imu_method:='2'"
    # ros2 run realsense2_camera realsense2_camera_node --ros-args -p serial_no:="'845412111433'" -p camera_name:=minibot_a_t265 -p spatial_filter.enable:=false -p temporal_filter.enable:=false -p pointcloud.enable:=false -r  __ns:=/minibot_a_t265 

