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
    
    # Set the path to this package.
    pkg_share = FindPackageShare(package='swarm_crawler').find('swarm_crawler')

    # Set the path to the RViz configuration settings
    default_rviz_config_path = os.path.join(
        pkg_share, 'rviz/rviz_basic_settings.rviz')

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
    
    # start_map_to_base_link_transform_cmd = Node(
    # package=package_name,
    # executable='map_to_base_link_transform.py')

    # start_base_link_to_aruco_marker_transform_cmd = Node(
    # package=package_name,
    # executable='base_link_to_aruco_marker_transform.py')
    # # Declare the launch arguments
      # Declare the launch arguments

    # NAME SPACE CREATION  
    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value=robot_name,
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        name='use_namespace',
        default_value='true',
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

    # Specify the actions

    # Publish the joint state values for the non-fixed joints in the URDF file.
    start_joint_state_publisher_cmd = Node(
        condition=UnlessCondition(gui),
        package='joint_state_publisher',
        # namespace=namespace,
        executable='joint_state_publisher',
        name='joint_state_publisher')

    # A GUI to manipulate the joint state values
    start_joint_state_publisher_gui_node = Node(
        condition=IfCondition(gui),
        # namespace=namespace,
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui')

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        # namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time,
                    # 'namespace' : namespace,
                     'robot_description': Command(['xacro ', urdf_model])}],
        arguments=[default_urdf_model_path])

    # ARUCO ESTIMATION
    start_aruco_marker_pose_transform_cmd = Node(
        package=package_name,
        namespace=namespace,
        executable='aruco_marker_pose_estimation_tf.py',
        parameters=[{'use_sim_time': use_sim_time, 'image_topic':'/minibot_a_d435/color/image_raw',}])
    
    start_depth_center_script = Node(
        package=package_name,
        namespace=namespace,
        executable='show_center_depth.py',
        parameters=[])

    #ROBOT LOCALIZATION   using an Extended Kalman filter
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[robot_localization_file_path, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    depthimage_to_laserscan_yaml_path =  os.path.join(pkg_share, 'config/depthimage_to_laserscan.yaml')

    start_depthimage_to_laserscan_cmd = Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan_node',
            remappings=[('depth','/minibot_a_d435/depth/image_rect_raw'),
                        ('depth_camera_info', '/minibot_a_d435/depth/camera_info'),
                        ('scan', '/minibot_a/scan')],
                        # ('camera_depth_frame', 'camera_link')],
            parameters=[depthimage_to_laserscan_yaml_path, ('use_sim_time', use_sim_time)])
    


    # ros2 run depthimage_to_laserscan depthimage_to_laserscan_node --remap range_min:=0 0.0 --remap depth:=/minibot_a_d435/depth/image_rect_raw --remap depth_camera_info:=/minibot_a_d435/depth/camera_info
    # robot_localization_realsenses_launch.py'
    # )
    
    teleop = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join( 'launch', 'controller-teleop.launch.py'))
    ,launch_arguments={'namespace':namespace}.items())#,
    # condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))
#   start_gazebo_server_cmd = IncludeLaunchDescription(
#     PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
#     condition=IfCondition(use_simulator),
#     launch_arguments={'world': world}.items())
    # launch_realsense_d435 = IncludeLaunchDescription(
    # PythonLaunchDescriptionSource(os.path.join('../../realsense-ros/realsense2_camera/launch/rs_launch.py'))
    # , launch_arguments={'camera_name':namespace}.items())#,
    # , launch_arguments={'camera_name': "minibot_a_d435","serial_no":"'830112071549'"}.items())#,


    # launch_realsense_t265 = IncludeLaunchDescription(
    # PythonLaunchDescriptionSource(os.path.join('/home/ben/Desktop/realsenseTest/realsense-ros/realsense2_camera/launch/rs_launch.py'))
    # , launch_arguments={'camera_name':"minibot_a_t265"}.items())#,

    # realsense = IncludeLaunchDescription(
    # PythonLaunchDescriptionSource(os.path.join('/home/ben/Desktop/realsenseTest/realsense-ros/realsense2_camera/launch/rs_launch.py'))
    # , launch_arguments={'camera_name':namespace}.items())#,
#  ros2 launch realsense2_camera rs_launch.py enable_fisheye1:=false enable_fisheye2:=false camera_name:=Minibot1T265
    # teleop\
    # teleop= Node(
    #     package=package_name,
    #     executable='aruco_marker_pose_estimation_tf.py',
    #     parameters=[{'use_sim_time': use_sim_time}])    

    # Launch RViz
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file])


    start_aruco_marker_pose_static_transform_cmd = Node(
    package="tf2_ros",
    executable='static_transform_publisher',
    namespace=namespace,
    arguments=["1.0", "0.0", "0.0", "0", "0", "0", "camera_link", "aruco_marker"],
    # arguments=["-1.0", "0.50", "0.53", "0", "3.141592654", "-1.57079633", "camera_link", "aruco_marker"],
    parameters=[{'use_sim_time': use_sim_time}],
    output="screen")

    # start_aruco_marker_pose_static_transform_cmd = Node(
    # package="tf2_ros",
    # executable='static_transform_publisher',
    # namespace=namespace,
    # arguments=["1.0", "1.0", "0.0", "0", "0", "0", "camera_link", "odom_frame"],
    # # arguments=["-1.0", "0.50", "0.53", "0", "3.141592654", "-1.57079633", "camera_link", "aruco_marker"],
    # parameters=[{'use_sim_time': use_sim_time}],
    # output="screen")


    # Create the launch description and populate
    ld = LaunchDescription()

    #NAMESPACES 
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    # Declare the launch options
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_joint_state_publisher_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(start_aruco_marker_pose_transform_cmd)

    # Add any actions
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    # ld.add_action(start_joint_state_publisher_gui_node)

    # ld.add_action(start_map_to_base_link_transform_cmd)
    # ld.add_action(start_base_link_to_aruco_marker_transform_cmd)
    ld.add_action(start_aruco_marker_pose_static_transform_cmd)
    ld.add_action(teleop)
    # ld.add_action(realsense)
    # ld.add_action(launch_realsense_d435)
    # ld.add_action(launch_realsense_t265)

    # ld.add_action(robot_localization_launch_arg)
    # time.sleep(2)
    ld.add_action(start_rviz_cmd)

    ld.add_action(robot_localization_node)
    ld.add_action(start_depthimage_to_laserscan_cmd)

    #


    return ld
