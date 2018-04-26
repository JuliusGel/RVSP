cmake_minimum_required(VERSION 2.8.3)
project(follow_wall)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(follow_node src/follow_node.cpp)
target_link_libraries(follow_node ${catkin_LIBRARIES})
