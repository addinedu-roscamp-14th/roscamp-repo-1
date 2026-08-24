cmake_minimum_required(VERSION 3.8)
project(drive)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ParkInSpot.action"
  "srv/AdaptiveLidarRecovery.srv"
  DEPENDENCIES action_msgs
)

add_library(drive_adaptive_lidar_recovery_bt_node SHARED
  src/adaptive_lidar_recovery_bt.cpp
)
target_compile_definitions(drive_adaptive_lidar_recovery_bt_node
  PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(drive_adaptive_lidar_recovery_bt_node
  behaviortree_cpp
  nav2_behavior_tree
  nav_msgs
  rclcpp
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(drive_adaptive_lidar_recovery_bt_node
  "${cpp_typesupport_target}")

ament_export_dependencies(rosidl_default_runtime)

install(
  PROGRAMS
    scripts/amcl_pose_heartbeat
    scripts/send_nav_goal
    scripts/parking_action_server
    scripts/reverse_parking
    scripts/parking_new
    scripts/navigation_event_publisher
    scripts/other_robot_obstacle
    scripts/costmap_scan_filter
    scripts/adaptive_lidar_recovery
    scripts/target_map_pose_to_nav_goal
    scripts/target_waypoints_to_nav_goal
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS drive_adaptive_lidar_recovery_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch params rviz behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_adaptive_lidar_recovery
    test/test_adaptive_lidar_recovery.py
  )
  ament_add_pytest_test(
    test_costmap_scan_filter
    test/test_costmap_scan_filter.py
  )
  ament_add_pytest_test(
    test_other_robot_obstacle
    test/test_other_robot_obstacle.py
  )
endif()

ament_package()
