cmake_minimum_required(VERSION 3.8)
project(only_robot_arm_pollination)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(pollination_cycle_node src/pollination_cycle_node.cpp)
target_compile_features(pollination_cycle_node PUBLIC cxx_std_17)
target_include_directories(pollination_cycle_node PRIVATE ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(
  pollination_cycle_node
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  sensor_msgs
  tf2_ros
  visualization_msgs
)

install(TARGETS pollination_cycle_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
