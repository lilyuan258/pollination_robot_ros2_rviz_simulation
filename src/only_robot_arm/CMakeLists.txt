cmake_minimum_required(VERSION 3.8)
project(only_robot_arm)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    urdf
    meshes
    config
    textures
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/watermelon_flower_marker.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()