cmake_minimum_required(VERSION 3.5)
project(deposit)

# =====================
# C++ STANDARD
# =====================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# =====================
# DEPENDENCIES
# =====================
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

find_package(aruco_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

find_package(tf2_ros REQUIRED)

find_package(ros2_kdl_package REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)

# KDL
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)



# =====================
# KDL CORE LIBRARY
# =====================
add_library(kdl_core
  src/kdl_robot.cpp
  src/kdl_control.cpp
  src/kdl_planner.cpp
)

target_include_directories(kdl_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(kdl_core
  rclcpp
  geometry_msgs
  trajectory_msgs
  orocos_kdl
  kdl_parser
)

# =====================
# EXECUTABLES
# =====================

# Pick & Place
add_executable(pick_package
  src/pick_package.cpp
)

ament_target_dependencies(pick_package
  rclcpp
  rclcpp_action
  std_msgs
  sensor_msgs
  trajectory_msgs
  aruco_msgs
  ros_gz_interfaces
  ros2_kdl_package
  tf2_ros
)

target_link_libraries(pick_package
  kdl_core
)

# ArUco detector
add_executable(aruco_detector
  src/aruco_detector.cpp
)

ament_target_dependencies(aruco_detector
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  OpenCV
)

# ROS2 KDL node
add_executable(ros2_kdl_node
  src/ros2_kdl_node.cpp
)

ament_target_dependencies(ros2_kdl_node
  rclcpp
  rclcpp_action
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  ros2_kdl_package
)


target_link_libraries(ros2_kdl_node
  kdl_core
)

# KDL example
add_executable(ros2_kdl_node_example
  src/ros2_kdl_node_example.cpp
)

ament_target_dependencies(ros2_kdl_node_example
  rclcpp
  rclcpp_action
)

target_link_libraries(ros2_kdl_node_example
  kdl_core
)

# Trajectory Action Client
add_executable(trajectory_action_client
  src/trajectory_action_client.cpp
)

ament_target_dependencies(trajectory_action_client
  rclcpp
  rclcpp_action
  ros2_kdl_package
)

# =====================
# INSTALL
# =====================
install(
  TARGETS
    kdl_core
    pick_package
    aruco_detector
    ros2_kdl_node
    ros2_kdl_node_example
    trajectory_action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY launch worlds models scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
