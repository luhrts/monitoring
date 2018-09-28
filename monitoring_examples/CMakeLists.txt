cmake_minimum_required(VERSION 2.8.3)
project(monitoring_examples)

find_package(catkin REQUIRED COMPONENTS
  monitoring_core
  roscpp
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES monitoring_examples
#  CATKIN_DEPENDS monitoring_core rosccp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(simple_monitor src/simple_monitor.cpp)

add_dependencies(simple_monitor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(simple_monitor
    ${catkin_LIBRARIES}
)

#############
## Install ##
#############

 install(TARGETS simple_monitor
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

catkin_install_python(PROGRAMS scripts/simple_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

