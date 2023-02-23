sleep 1
source /opt/ros/galactic/setup.bash
source realsense-ros/install/setup.bash
#launch t265 first
# topic_odom_in:='/test_odom' 
# initial_reset:=true 

# create a command line argument named "reset", and include it in the launch file
    # ros2 launch realsense2_camera rs_launch.py  initial_reset:=true  camera_name:=minibot_a_t265 "serial_no:='845412111433'" enable_gyro:=True enable_acel:=True enable_pose:=True 
    ros2 launch realsense2_camera rs_launch.py  camera_name:=minibot_a_t265 "serial_no:='845412111433'" enable_gyro:=True enable_acel:=True enable_pose:=True # "unite_imu_method:='2'"
