sleep 1
cd ..
source /opt/ros/galactic/setup.bash
source swarm_crawler/install/setup.bash
cd swarm_crawler
colcon build && ros2 launch swarm_crawler minibot_nav2.launch.py use_rviz:=False autostart:=False slam:=False
