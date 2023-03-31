sleep 1
cd ..
# source /opt/ros/galactic/setup.bash
source swarm_crawler/install/setup.bash
cd swarm_crawler

# colcon build && ros2 launch swarm_crawler controller-teleop.launch.py
colcon build &&  ros2 launch swarm_crawler base-station.launch.py 