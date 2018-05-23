cmake_minimum_required(VERSION 2.8.3)
project(monitoring_bridge)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  monitoring_msgs
  roscpp
  rospy
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS monitoring_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(monitoring_to_diagnostics src/monitoringtodiagnostics.cpp)
target_link_libraries(monitoring_to_diagnostics
  ${catkin_LIBRARIES}
)
add_dependencies(monitoring_to_diagnostics ${catkin_EXPORTED_TARGETS})

add_executable(diagnostics_to_monitoring src/diagnosticstomonitoring.cpp)
target_link_libraries(diagnostics_to_monitoring
  ${catkin_LIBRARIES}
)
add_dependencies(diagnostics_to_monitoring ${catkin_EXPORTED_TARGETS})
