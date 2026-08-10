cmake_minimum_required(VERSION 3.8)
project(porter_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmState.msg"
  "msg/PixelNavigationCommand.msg"
  "msg/PortEvent.msg"
  "msg/VehicleState.msg"
  "action/DispatchArmCommand.action"
  "action/DispatchNavigation.action"
  DEPENDENCIES geometry_msgs std_msgs
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
