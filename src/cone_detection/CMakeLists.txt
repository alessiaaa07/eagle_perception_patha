cmake_minimum_required(VERSION 3.8)
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()
project(cone_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(visualization_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(eagle_perception_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation)

add_executable(cone_detection_node src/cone_detection_node.cpp)

ament_target_dependencies(cone_detection_node
  rclcpp
  sensor_msgs
  pcl_conversions
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
  eagle_perception_interfaces
  visualization_msgs
)
target_include_directories(cone_detection_node PRIVATE
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(cone_detection_node
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

install(TARGETS cone_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)