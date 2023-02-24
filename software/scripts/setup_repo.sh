cd ..
#source ros 
source /opt/ros/foxy/setup.bash

# build realsense-ros library
cd realsense-ros
colcon build --symlink-install


#build the swarm_crawler package
cd ..
cd swarm_crawler
colcon build --symlink-install

