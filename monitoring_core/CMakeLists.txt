cmake_minimum_required(VERSION 2.8.3)
project(monitoring_core)

## Compile as C++11, supported in ROS Kinetic and newer

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  monitoring_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES monitoring_core
  CATKIN_DEPENDS monitoring_msgs
  DEPENDS
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_library(monitoring_core
  src/monitor.cpp
)
target_link_libraries(monitoring_core
  ${catkin_LIBRARIES}
)
add_dependencies(monitoring_core monitoring_msgs_generate_messages_cpp)


