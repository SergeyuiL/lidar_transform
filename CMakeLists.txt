cmake_minimum_required(VERSION 3.0.2)
project(lidar_transform)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(lidar_transform_node src/lidar_transform_node.cpp)

target_link_libraries(lidar_transform_node ${catkin_LIBRARIES})
add_dependencies(lidar_transform_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
