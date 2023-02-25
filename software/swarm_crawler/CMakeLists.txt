cmake_minimum_required(VERSION 3.8)
project(swarm_crawler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
# find_package(aruco_interface REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# rosidl_target_interfaces(aruco_interface
#   ${PROJECT_NAME} "rosidl_typesupport_py")



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY config launch maps meshes models params rviz urdf worlds swarm_crawler 
  DESTINATION share/${PROJECT_NAME}
)

# find_package(rosidl_default_generators REQUIRED)
# find_package(ros2_aruco_interfaces REQUIRED)
# ament_python_install_package(ros2_aruco_interfaces)
# add_subdirectory("ros2_aruco_interfaces")
# set(ros2_aruco_interfaces_DIR "scripts/ros2_aruco_interfaces")
# ament_python_install_package("ros2_aruco_interfaces")

install(PROGRAMS
  scripts/aruco_marker_detector.py
  scripts/aruco_marker_pose_estimation_tf.py
  # scripts/ros2_aruco_interfaces
  # scripts/run_inspection.py
  # scripts/lift_controller.py
  # scripts/nav_to_pose.py
  # scripts/nav_through_poses.py
  # scripts/pick_and_deliver.py
  # scripts/robot_navigator.py
  # scripts/security_demo.py
  # scripts/waypoint_follower.py
  # scripts/battery_state/battery_state_pub.py
  # scripts/battery_state/charging_battery_pub.py
  # scripts/battery_state/full_battery_pub.py
  # scripts/battery_state/low_battery_pub.py
  # scripts/navigate_to_charging_dock.py
  # scripts/navigate_to_charging_dock_v2.py
  # scripts/navigate_to_charging_dock_v3.py
  scripts/navigate_to_charging_dock_v4.py
  scripts/transforms/map_to_base_link_transform.py
  scripts/transforms/base_link_to_aruco_marker_transform.py
  DESTINATION lib/${PROJECT_NAME}
)


# rosidl_generate_interfaces(${PROJECT_NAME}
#   "scripts/msg/ArucoMarkers.msg"
#   DEPENDENCIES geometry_msgs
#   DEPENDENCIES std_msgs
#  )


ament_package()
