cmake_minimum_required(VERSION 3.8)
project(only_robot_arm)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(pollination_cycle_node src/pollination_cycle_node.cpp)
ament_target_dependencies(
  pollination_cycle_node
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  sensor_msgs
  tf2_geometry_msgs
  tf2_ros
)

install(
  DIRECTORY
    launch
    urdf
    meshes
    config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/watermelon_flower_marker.py
    scripts/environment_visual_marker.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
    pollination_cycle_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
