import os
import time
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

robot_name = 'minibot_a' # TODO: make this a exec bash command that retrieves the name of the robot

def generate_launch_description():
    package_name = 'swarm_crawler'
    
    # Set the path to this package.
    pkg_share = FindPackageShare(package='swarm_crawler').find('swarm_crawler')

    #path to the localization ekf file 
    robot_localization_file_path = 'config/ekf.yaml'
    robot_localization_file_path = os.path.join(pkg_share, robot_localization_file_path) 

    use_sim_time = LaunchConfiguration('use_sim_time')


    #ROBOT LOCALIZATION   using an Extended Kalman filter
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[robot_localization_file_path, {'use_sim_time': False}]
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(robot_localization_node)
    # time.sleep(10)

   



    return ld
