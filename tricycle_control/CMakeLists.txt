cmake_minimum_required(VERSION 2.8.3)
project(tricycle_control)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  controller_interface
  realtime_tools
  tf
  tf2
  angles
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
)

add_library(
  tricycle_controller
  src/TriCycleController.cpp
  src/TriCycleOdometer.cpp
)
target_link_libraries(tricycle_controller ${catkin_LIBRARIES})
