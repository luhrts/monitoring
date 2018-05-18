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

  Gui.msg
  GuiInfo.msg

)

generate_messages(
  DEPENDENCIES
    std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fdiSDK recoverySDK
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
  src/fdir/std_validator/nodeavailable.cpp
)

target_link_libraries(fdiSDK
    ${catkin_LIBRARIES}
 )
add_dependencies(fdiSDK ros_monitoring_generate_messages_cpp)

add_library(recoverySDK
  src/recovery/recoverysdk.cpp
  src/recovery/errorhandlerinterface.cpp
  src/recovery/std_handler/restartnodehandler.cpp
  src/recovery/std_handler/stoplaunchfile.cpp
  src/recovery/std_handler/outputerrormessage.cpp
  src/recovery/std_handler/error_to_speech.cpp
)
target_link_libraries(recoverySDK
    ${catkin_LIBRARIES}
 )
add_dependencies(recoverySDK ros_monitoring_generate_messages_cpp)

add_library(monitorLIB
  src/monitors/monitormsg.cpp
)
target_link_libraries(monitorLIB
  ${catkin_LIBRARIES}
)
add_dependencies(monitorLIB ros_monitoring_generate_messages_cpp)
####################### MONITOR NODES #########################

add_executable(cpu_monitoring_node src/monitors/cpumonitor.cpp)
add_executable(ram_monitoring_node src/monitors/rammonitor.cpp)
add_executable(network_monitoring_node src/monitors/networkmonitor.cpp)
add_executable(statistics_monitoring_node src/monitors/statisticsmonitor.cpp)

target_link_libraries(cpu_monitoring_node
  ${catkin_LIBRARIES}
  monitorLIB

)
add_dependencies(cpu_monitoring_node ros_monitoring_generate_messages_cpp)

target_link_libraries(ram_monitoring_node
  ${catkin_LIBRARIES}
  ${PROCPS_LIBRARIES}
  monitorLIB
)
add_dependencies(ram_monitoring_node ros_monitoring_generate_messages_cpp)

target_link_libraries(network_monitoring_node
  ${catkin_LIBRARIES}
  monitorLIB
)
add_dependencies(network_monitoring_node ros_monitoring_generate_messages_cpp)


target_link_libraries(statistics_monitoring_node
  ${catkin_LIBRARIES}
  monitorLIB
)
add_dependencies(statistics_monitoring_node ros_monitoring_generate_messages_cpp)






####################### FDIR NODES #########################

add_executable(fdi_example_node src/fdir/fdiExample.cpp)

target_link_libraries(fdi_example_node
  ${catkin_LIBRARIES}

  fdiSDK
)

add_dependencies(fdi_example_node ros_monitoring_generate_messages_cpp)

add_executable(recovery_example_node src/recovery/recoveryExample.cpp)

target_link_libraries(recovery_example_node
  ${catkin_LIBRARIES}

  recoverySDK
)

add_dependencies(recovery_example_node ros_monitoring_generate_messages_cpp)

####################### TOOL NODES #########################
add_executable(gui_msg_concat_node src/guiconcatenation.cpp)
target_link_libraries(gui_msg_concat_node
  ${catkin_LIBRARIES}
)
add_dependencies(gui_msg_concat_node ros_monitoring_generate_messages_cpp)


add_executable(monitoring_to_diagnostics src/monitoringtodiagnostics.cpp)
target_link_libraries(monitoring_to_diagnostics
  ${catkin_LIBRARIES}
)
add_dependencies(monitoring_to_diagnostics ros_monitoring_generate_messages_cpp)


add_executable(diagnostics_to_monitoring src/diagnosticstomonitoring.cpp)
target_link_libraries(diagnostics_to_monitoring
  ${catkin_LIBRARIES}
)
add_dependencies(diagnostics_to_monitoring ros_monitoring_generate_messages_cpp)

add_executable(speech_output src/speech_output.cpp)
target_link_libraries(speech_output
  ${catkin_LIBRARIES}
)
add_dependencies(speech_output ros_monitoring_generate_messages_cpp)




find_package(catkin REQUIRED COMPONENTS roslaunch)
roslaunch_add_file_check(launch)
