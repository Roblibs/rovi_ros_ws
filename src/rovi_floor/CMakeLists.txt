cmake_minimum_required(VERSION 3.8)
project(rovi_floor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenSSL REQUIRED)

add_executable(floor_runtime_node src/floor_runtime_node.cpp)
ament_target_dependencies(floor_runtime_node
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  image_transport
  cv_bridge
)
target_link_libraries(floor_runtime_node ${OpenCV_LIBRARIES})
target_link_libraries(floor_runtime_node OpenSSL::Crypto)

add_executable(floor_calibrate_node src/floor_calibrate_node.cpp)
ament_target_dependencies(floor_calibrate_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  image_transport
  cv_bridge
)
target_link_libraries(floor_calibrate_node ${OpenCV_LIBRARIES})
target_link_libraries(floor_calibrate_node OpenSSL::Crypto)

install(TARGETS
  floor_runtime_node
  floor_calibrate_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
