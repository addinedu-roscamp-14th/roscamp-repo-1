cmake_minimum_required(VERSION 3.8)
project(pinky)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(SRV_FILES
  "srv/SetLed.srv"
  "srv/Emotion.srv"
  "srv/SetBrightness.srv"
  "srv/SetLamp.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${SRV_FILES}
  DEPENDENCIES builtin_interfaces std_msgs action_msgs
)

ament_python_install_package(pinky_nodes)

set(SLLIDAR_SDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/sdk")
file(GLOB SLLIDAR_SDK_SRC
  "${SLLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/dataunpacker/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/dataunpacker/unpacker/*.cpp"
)

include_directories(
  ${SLLIDAR_SDK_PATH}/include
  ${SLLIDAR_SDK_PATH}/src
)

add_executable(sllidar_node src/sllidar_node.cpp ${SLLIDAR_SDK_SRC})
ament_target_dependencies(sllidar_node
  rclcpp
  std_srvs
  sensor_msgs
)

add_executable(sllidar_client src/sllidar_client.cpp)
ament_target_dependencies(sllidar_client
  rclcpp
  std_srvs
  sensor_msgs
)

install(
  DIRECTORY launch config meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/bringup
    scripts/battery_publisher
    scripts/led_server
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS sllidar_node sllidar_client
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime rclcpp std_msgs sensor_msgs std_srvs)
ament_package()
