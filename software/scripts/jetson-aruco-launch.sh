sleep 1
cd ..
source /opt/ros/galactic/setup.bash
source swarm_crawler/install/setup.bash
cd swarm_crawler

#if we are on the jetson, we want to launch headless
if [[ $PWD == *"jetson"* ]]; then
    echo 'Running Headless '
    colcon build && ros2 launch swarm_crawler minibot_dock.launch.py use_rviz:=False run_headless:=True autostart:=false
else
    echo "Running on Desktop."
    colcon build && ros2 launch swarm_crawler minibot_dock.launch.py autostart:=False
fi

# colcon build && ros2 launch swarm_crawler minibot_aruco.launch.py 
