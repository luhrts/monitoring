cmake_minimum_required(VERSION 2.8.3)
project(ros_monitoring)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ros_rt_benchmark_lib
  rt_benchmark_lib
  roscpp
  std_msgs
  message_generation
  
  
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(PROCPS REQUIRED libprocps)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
  FILES
  
  KeyValue.msg
  Machine.msg
  MonitoringArray.msg
  MonitoringInfo.msg
  Process.msg
  Processes.msg
  Error.msg
  
  
)

generate_messages(
  DEPENDENCIES
    std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_monitoring
  CATKIN_DEPENDS message_runtime
 DEPENDS PROCPS
)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${PROCPS_INCLUDE_DIRS}
)

## Declare a C++ library

add_library(fdiSDK
  src/fdir/fdiSDK.cpp
  src/fdir/configinterface.cpp
  src/fdir/std_validator/max.cpp
  src/fdir/std_validator/min.cpp
  src/fdir/std_validator/between.cpp
)

add_library(recoverySDK
  src/recovery/recoverysdk.cpp
  src/recovery/errorhandlerinterface.cpp
  src/recovery/std_handler/restartnodehandler.cpp
)
#target_link_libraries(fdiconfigInterface
#    ${catkin_LIBRARIES}
# )

####################### MONITOR NODES #########################

add_executable(cpu_monitoring_node src/monitors/cpumonitor.cpp)
add_executable(ram_monitoring_node src/monitors/rammonitor.cpp)
add_executable(network_monitoring_node src/monitors/networkmonitor.cpp)
add_executable(rosnode_monitoring_node src/monitors/nodemonitor.cpp)
add_executable(rostopic_monitoring_node src/monitors/topicmonitor.cpp)

target_link_libraries(cpu_monitoring_node
  ${catkin_LIBRARIES}

)
target_link_libraries(ram_monitoring_node
  ${catkin_LIBRARIES}
  ${PROCPS_LIBRARIES}

)
target_link_libraries(network_monitoring_node
  ${catkin_LIBRARIES}

)
target_link_libraries(rosnode_monitoring_node
  ${catkin_LIBRARIES}
)
target_link_libraries(rostopic_monitoring_node
  ${catkin_LIBRARIES}
)




####################### FDIR NODES #########################

add_executable(fdi_example_node src/fdir/configExample.cpp)

target_link_libraries(fdi_example_node
  ${catkin_LIBRARIES}

  fdiSDK
)

#add_executable(recovery_example_node src/recovery/recoveryExample.cpp)

#target_link_libraries(recovery_example_node
#  ${catkin_LIBRARIES}

#  recoverySDK
#)


find_package(catkin REQUIRED COMPONENTS roslaunch)
roslaunch_add_file_check(launch)
