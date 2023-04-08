# cleanup all ros2 affiliated processes
programs_to_kill="ekf_node  ros_2 joy_node joy_node teleop_node ros2 static_transform_publisher ros2 rviz2 joint_state_publisher_gui robot_state_publisher"
sudo killall -e sync_slam_toolbox_node
sudo killall -e lifecycle_manager
sudo killall -e map_server
sudo killall -e  amcl


sudo killall -e  ekf_node
sudo killall -e joy_node 
sudo killall -e teleop_node  
sudo killall -e ros2 
sudo pkill static_transform_publisher 
sudo killall -e ros2
sudo killall -e rviz2  
sudo pkill robot_state_publisher
sudo killall -e joint_state_publisher_gui 
sudo killall -e robot_state_publisher 
sudo killall -e depthimage_to_laserscan_node 
sudo killall -e static_transform_publisher 
sudo killall -e sync_slam_toolbox_node
sudo killall -e lifecycle_manager
sudo killall -e map_saver_server
sudo killall -e bt_navigator
sudo killall -e waypoint_follower
sudo killall -e lifecycle_manager
sudo killall -e planner_server
sudo killall -e controller_server
sudo killall -e recoveries_server
sudo killall -e rviz2
sudo pkill python3
# sudo pkill _ros2_daemon
sudo pkill ros 
sudo pkill dynamic_bdirge
sudo pkill roscore


 


# sudo killall -e python3 # gotta do it, sorry 
sudo killall -e rqt 



# sudo killall -e realsense2_camera_node


