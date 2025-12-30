cmake_minimum_required(VERSION 3.8)
project(rovi_odom_integrator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(diagnostic_updater REQUIRED)

add_executable(rovi_odom_integrator_node src/rovi_odom_integrator_node.cpp)
ament_target_dependencies(rovi_odom_integrator_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  diagnostic_updater
)

install(TARGETS rovi_odom_integrator_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
