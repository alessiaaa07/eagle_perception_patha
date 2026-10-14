cmake_minimum_required(VERSION 3.8)
project(cone_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(eagle_perception_interfaces REQUIRED)

add_executable(cone_detection_node src/cone_detection_node.cpp)

ament_target_dependencies(cone_detection_node
  rclcpp
  sensor_msgs
  eagle_perception_interfaces
)

install(TARGETS cone_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
