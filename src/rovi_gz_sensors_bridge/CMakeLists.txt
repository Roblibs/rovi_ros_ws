cmake_minimum_required(VERSION 3.8)
project(rovi_gz_sensors_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz-msgs REQUIRED)

add_executable(rovi_gz_sensors_bridge_node src/rovi_gz_sensors_bridge_node.cpp)
ament_target_dependencies(rovi_gz_sensors_bridge_node
  rclcpp
  sensor_msgs
  ros_gz_bridge
)
target_link_libraries(rovi_gz_sensors_bridge_node
  gz-transport::core
  gz-msgs::core
)

install(TARGETS rovi_gz_sensors_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

