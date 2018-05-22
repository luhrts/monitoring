cmake_minimum_required(VERSION 2.8.3)
project(monitoring_visualization)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  monitoring_msgs
  roscpp
  rospy
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(gui_msg_concat_node src/guiconcatenation.cpp)
target_link_libraries(gui_msg_concat_node
  ${catkin_LIBRARIES}
)
