cmake_minimum_required(VERSION 2.8.3)
project(ros_monitoring)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(PROCPS REQUIRED libprocps)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)




catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_monitoring
  CATKIN_DEPENDS message_runtime
 DEPENDS PROCPS
)

add_message_files(
  FILES
  Processes.msg
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
 ${catkin_INCLUDE_DIRS}
 ${PROCPS_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/ros_monitoring.cpp
# )


add_executable(cpu_monitoring_node src/cpumonitor.cpp)
add_executable(ram_monitoring_node src/rammonitor.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(cpu_monitoring_node
  ${catkin_LIBRARIES}
  ${PROCPS_LIBRARIES}
  
)

target_link_libraries(ram_monitoring_node
  ${catkin_LIBRARIES}
  ${PROCPS_LIBRARIES}
  
)
