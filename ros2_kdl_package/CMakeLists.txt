cmake_minimum_required(VERSION 3.8)
project(ros2_kdl_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === Dipendenze necessarie ===
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(aruco_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)   #Per il subscriber PoseStamped


# === Generazione Action ===
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Trajectory.action"
)
ament_export_dependencies(rosidl_default_runtime)

add_executable(ros2_kdl_node
  src/ros2_kdl_node.cpp
  src/kdl_robot.cpp
  src/kdl_planner.cpp
  src/kdl_control.cpp
)
target_include_directories(ros2_kdl_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_compile_features(ros2_kdl_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(ros2_kdl_node
  rclcpp
  rclcpp_action
  std_msgs
  sensor_msgs
  kdl_parser
  orocos_kdl
  controller_manager_msgs
  rosidl_default_runtime
  geometry_msgs
  Eigen3
)

rosidl_get_typesupport_target(cpp_typesupport_target_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(ros2_kdl_node "${cpp_typesupport_target_node}")

install(TARGETS ros2_kdl_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

# ACTION CLIENT (trajectory_action_client)
add_executable(trajectory_action_client
  src/trajectory_action_client.cpp
)
target_include_directories(trajectory_action_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(trajectory_action_client PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(trajectory_action_client
  rclcpp
  rclcpp_action
  rosidl_default_runtime
)

rosidl_get_typesupport_target(cpp_typesupport_target_client
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(trajectory_action_client "${cpp_typesupport_target_client}")

install(TARGETS trajectory_action_client
  DESTINATION lib/${PROJECT_NAME})

ament_package()


