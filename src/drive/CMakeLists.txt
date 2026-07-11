cmake_minimum_required(VERSION 3.8)
project(drive)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/send_nav_goal
    scripts/target_map_pose_to_nav_goal
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch params rviz behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
