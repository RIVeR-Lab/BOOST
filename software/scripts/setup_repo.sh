cd ~/swarm_crawler
#source ros 
source /opt/ros/foxy/setup.bash

#build the swarm_crawler package
cd ~/swarm_crawler
cd software
git checkout software 
cd swarm_crawler

colcon build # --symlink-install

# build realsense-ros library
cd ~/swarm_crawler/realsense-ros
colcon build # --symlink-install




