import os
import time
from launch import LaunchDescription
# from launch.actions import PushRosNamespace
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)


robot_name = 'minibot_a' # TODO: make this a exec bash command that retrieves the name of the robot
#  TODO: fix the namespace issue
def generate_launch_description():
    package_name = 'swarm_crawler'
    nav_to_charging_dock_script = 'navigate_to_aruco_v1.py'
    
    # Set the path to this package.
    pkg_share = FindPackageShare(package='swarm_crawler').find('swarm_crawler')
   
    # Set the path to the RViz configuration settings
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/swarm_crawler.rviz')

    # Set the path to the URDF file
    default_urdf_model_path = os.path.join(pkg_share, 'urdf/minibot.urdf')
    
    # Navigation 2 parameters
    nav2_params_path = 'params/nav2_aruco_traversal.yaml'
    nav2_params_path = os.path.join(pkg_share, nav2_params_path)
    nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup') 
    nav2_launch_dir = os.path.join(nav2_dir, 'launch') 
    depthimage_to_laserscan_yaml_path =  os.path.join('/home/ben/Desktop/swarm_crawler/software/swarm_crawler/config/depthimage_to_laserscan_params.yaml')

    slam = LaunchConfiguration('slam')

    #path to the localization ekf file 
    robot_localization_file_path = 'config/ekf.yaml'
    robot_localization_file_path = os.path.join(pkg_share, robot_localization_file_path) 
    
    # Launch configuration variables specific to simulation

    map_file_path = 'maps/hospital_world/hospital_world.yaml'
    static_map_path = os.path.join(pkg_share, map_file_path)  
    gui = LaunchConfiguration('gui')
    urdf_model = LaunchConfiguration('urdf_model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace') #creating a namespace that can easily be changed from minibot to minibot. 
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')



    # NAME SPACE CREATION  
    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        name='use_namespace',
        default_value='True',
        description='Whether to apply a namespace to the navigation stack')
    
    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_model_path,
        description='Absolute path to robot urdf file')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')

    declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
        name='gui',
        default_value='True',
        description='Flag to enable joint_state_publisher_gui')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')

    # autostart nav_2 stack
    declare_autostart_cmd = DeclareLaunchArgument(
        name='autostart', 
        default_value='True',
        description='Automatically startup the nav2 stack')

    declare_params_file_cmd = DeclareLaunchArgument(
        name='params_file',
        default_value=nav2_params_path,
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    declare_laserscan_params_file_cmd = DeclareLaunchArgument(
        name='depthimage_to_laserscan_yaml_path',
        default_value=depthimage_to_laserscan_yaml_path,
        description='depthimage_to_laserscan parameters file')
        

    declare_slam_cmd = DeclareLaunchArgument(
        name='slam',
        default_value='True',
        description='Whether to run SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        name='map',
        default_value=static_map_path,
        description='Full path to map file to load')
    # Specify the actions

    # Publish the joint state values for the non-fixed joints in the URDF file.
    start_joint_state_publisher_cmd = Node(
        condition=UnlessCondition(gui),
        package='joint_state_publisher',
        namespace=namespace,
        executable='joint_state_publisher',
        name='joint_state_publisher')

    # A GUI to manipulate the joint state values
    start_joint_state_publisher_gui_node = Node(
        condition=IfCondition(gui),
        namespace=namespace,
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui')
    # remappings = [('/minibot_a_t265_pose_frame', 'minibot_a_t265_pose_frame')
    #         ]
    remappings = [('/tf', 'tf'),
                ('/tf_static', 'tf_static')]
    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time,
                    # 'namespace' : namespace,
                     'robot_description': Command(['xacro ', urdf_model])}],
        remappings=remappings,
        arguments=[default_urdf_model_path])

    # ARUCO ESTIMATION
    start_aruco_marker_pose_transform_cmd = Node(
        package=package_name,
        namespace=namespace,
        executable='aruco_marker_pose_estimation_tf.py',
        parameters=[{'use_sim_time': use_sim_time, 'image_topic':'/minibot_a_d435/color/image_raw',}])

    #ROBOT LOCALIZATION using an Extended Kalman filter
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       namespace=namespace,
       output='screen',
       parameters=[robot_localization_file_path, {'use_sim_time': use_sim_time}]
    )
    



    teleop = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join( 'launch', 'controller-teleop.launch.py'))
    ,launch_arguments={'namespace':namespace}.items())#,

    # NAVIGATION 2
    start_nav2_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
    launch_arguments = {'namespace': '',
                        'use_namespace': use_namespace,
                        'map': map_yaml_file,
                        'use_sim_time': 'False',
                        'params_file': params_file,
                        'yaml_filename': params_file,
                        'slam': slam,
                        'autostart': autostart}.items())
  
    # Launch RViz
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file])

    start_depthimage_to_laserscan_cmd = Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan_node',
            namespace=namespace,
            remappings=[('depth','/minibot_a_d435/depth/image_rect_raw'),
                        ('depth_camera_info', '/minibot_a_d435/depth/camera_info'),
                        ('scan', '/minibot_a/scan'),
                        # ('range_min', '0.45'),
                        ('depth_frame_id', 'minibot_a_d435_depth_frame'),
                        ('output_frame','minibot_a_d435_depth_frame' ),
                        ('camera_depth_optical_frame','minibot_a_d435_depth_frame' ),
                        

                       ],
                        # ('camera_depth_frame', 'camera_link')],
            parameters=[{depthimage_to_laserscan_yaml_path}])
    


    depth_frame = Node(
    package="tf2_ros",
    executable='static_transform_publisher',
    namespace=namespace,
    arguments=["0.0", "0.0", "0.0", "0", "0", "0", "minibot_a_d435_depth_frame", "camera_depth_frame"],
    #                                   camera_link    minibot_a_d435_depth_frame
    # arguments=["-1.0", "0.50", "0.53", "0", "3.141592654", "-1.57079633", "camera_link", "aruco_marker"],
    parameters=[{'use_sim_time': use_sim_time}],
    output="screen")

    start_aruco_marker_pose_static_transform_cmd = Node(
    package="tf2_ros",
    executable='static_transform_publisher',
    namespace=namespace,
    arguments=["1.0", "0.0", "0.0", "0", "0", "0", "camera_link", "aruco_marker"],
    # arguments=["-1.0", "0.50", "0.53", "0", "3.141592654", "-1.57079633", "camera_link", "aruco_marker"],
    parameters=[{'use_sim_time': use_sim_time}],
    output="screen")

    start_odom_to_baselink_transform_cmd = Node(
    package="tf2_ros",
    executable='static_transform_publisher',
    namespace=namespace,
    arguments=["0", "0.0", "0.0", "0", "0", "0", "odom", "base_link"],
    # arguments=["-1.0", "0.50", "0.53", "0", "3.141592654", "-1.57079633", "camera_link", "aruco_marker"],
    parameters=[{'use_sim_time':False }],
    output="screen")


    start_odom_static_transform_cmd = Node(
    package="tf2_ros",
    executable='static_transform_publisher',
    namespace=namespace,
    arguments=["1", "0.0", "0.0", "0", "0", "0", "map", "odom"],
    # arguments=["-1.0", "0.50", "0.53", "0", "3.141592654", "-1.57079633", "camera_link", "aruco_marker"],
    parameters=[{'use_sim_time': use_sim_time}],
    output="screen")

    #aruco marker traversal
      # Launch navigation to the charging dock

    start_navigate_to_charging_dock_cmd = Node(
    package='swarm_crawler',
    executable=nav_to_charging_dock_script)   

    # Create the launch description and populate
    ld = LaunchDescription()
    # ld.PushRosNamespace(namespace=namespace)
    ld.add_action(declare_use_sim_time_cmd)
    
    #NAMESPACES 
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    # bad attempt to fix namespace issues 
    # bringup_cmd_group = GroupAction([
    #     PushRosNamespace(
    #         condition=IfCondition(use_namespace),
    #         namespace=namespace)])
    # ld.PushRosNamespace(
    #         condition=IfCondition(use_namespace),
    #         namespace=namespace)
    # ld.add_action(bringup_cmd_group)

    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_joint_state_publisher_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)

    ld.add_action(declare_laserscan_params_file_cmd)


    # STARTING ARUCO MARKER POSE TRANSFORM
    # ld.add_action(start_aruco_marker_pose_transform_cmd)

    # STARTING STATE PUBLISHER NODES
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_gui_node)

    # TRANSFORMS
    ld.add_action(start_odom_to_baselink_transform_cmd)
    ld.add_action(start_aruco_marker_pose_static_transform_cmd)
    # ld.add_action(start_odom_static_transform_cmd) # this is bad for SLAM. it will not expand the map.
    # ld.add_action(start_map_static_transform_cmd)
    # necessary "camera_depth_frame" redundant transform-- couldnt figure out the cause. 
    ld.add_action(depth_frame)

    start_depth_center_script = Node(
        package=package_name,
        executable='show_center_depth2.py')
    
    ld.add_action(start_depth_center_script)


    # ld.add_action(start_base_link_to_aruco_marker_transform_cmd)
 
    # ld.add_action(teleop)
    ld.add_action(start_depthimage_to_laserscan_cmd)

    # ld.add_action(robot_localization_launch_arg)
    # time.sleep(10)

    # Nav 2 launch 
    # the errors that occured with nav2 were primarily due to the namespace issue stuff 
    start_nav2 = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_share,'launch','unused_launch' ,'nav2.py')),
    launch_arguments = {'namespace': ''}.items())
    # ld.add_action(start_nav2)

      # Start ArUco marker detector node
    start_aruco_marker_detector_cmd = Node(
    package=package_name,
    executable='aruco_marker_detector.py')  

    # AUTONOMOUS DOCKING
    ld.add_action(start_aruco_marker_detector_cmd)
    # ld.add_action(start_navigate_to_charging_dock_cmd)



    # navigation stuff

    ld.add_action(robot_localization_node)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(start_nav2_cmd)
    ld.add_action(start_rviz_cmd)
    # camera stuff 
    # ld.add_action(realsense)
    # ld.add_action(launch_realsense_d435)
    # ld.add_action(launch_realsense_t265)

      # Go to the staging area pose
    #   navigator.goToPose(staging_area_pose) #THIS MIGHT BE IT!

    return ld

# ros2 topic pub /battery_status sensor_msgs/BatteryState '{voltage: 2.16, percentage: 0.24, power_supply_status: 3}' 


