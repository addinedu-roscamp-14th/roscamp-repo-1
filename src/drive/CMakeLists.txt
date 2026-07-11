cmake_minimum_required(VERSION 3.8)
project(drive)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch params rviz behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
