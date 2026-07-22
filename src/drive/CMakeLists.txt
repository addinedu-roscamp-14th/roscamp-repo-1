cmake_minimum_required(VERSION 3.8)
project(drive)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ParkInSpot.action"
  DEPENDENCIES action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

install(
  PROGRAMS
    scripts/send_nav_goal
    scripts/parking_action_server
    scripts/reverse_parking
    scripts/target_map_pose_to_nav_goal
    scripts/target_waypoints_to_nav_goal
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch params rviz behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
