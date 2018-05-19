cmake_minimum_required(VERSION 2.8.3)
project(monitoring_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

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
    #INCLUDE_DIRS include
    #LIBRARIES
    CATKIN_DEPENDS
        message_runtime
        std_msgs
    #DEPENDS
)
