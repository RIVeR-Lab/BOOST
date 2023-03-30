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

robot_name = 'minibot_a'

def generate_launch_description():
    package_name = 'swarm_crawler'
    
    robot_localization_file_path = 'config/ekf.yaml'
    # default_rviz_config_path = os.path.join(pkg_share,)
    # Set the path to this package.
    pkg_share = FindPackageShare(package='swarm_crawler').find('swarm_crawler')

    # Set the path to the RViz configuration settings
    default_rviz_config_path = os.path.join(
        pkg_share,  'rviz/swarm_crawler.rviz')

    # Set the path to the URDF file
    default_urdf_model_path = os.path.join(pkg_share, 'urdf/minibot.urdf')

    #path to the localization ekf file 
    robot_localization_file_path = os.path.join(pkg_share, robot_localization_file_path) 

    ########### YOU DO NOT NEED TO CHANGE ANYTHING BELOW THIS LINE ##############
    # Launch configuration variables specific to simulation
    gui = LaunchConfiguration('gui')
    urdf_model = LaunchConfiguration('urdf_model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace') #creating a namespace that can easily be changed from minibot to minibot. 
    use_namespace = LaunchConfiguration('use_namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')

     # Launch RViz
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file])

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)

    teleop = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join( 'launch', 'controller-teleop.launch.py'))
    ,launch_arguments={'namespace':namespace}.items())

    ld.add_action(teleop)
    ld.add_action(start_rviz_cmd)

    

    return ld


