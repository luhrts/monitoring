cmake_minimum_required(VERSION 2.8.3)
project(monitoring_visualization)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  monitoring_msgs
  roscpp
  rospy
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS roscpp 
)


include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(gui_msg_concat_node src/guiconcatenation.cpp)
target_link_libraries(gui_msg_concat_node
  ${catkin_LIBRARIES}
)
add_dependencies(gui_msg_concat_node ${catkin_EXPORTED_TARGETS})

install(TARGETS gui_msg_concat_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)


