sleep 1
cd ..
source /opt/ros/foxy/setup.bash
source swarm_crawler/install/setup.bash
cd swarm_crawler
colcon build && ros2 launch swarm_crawler minibot_aruco.launch.py 
