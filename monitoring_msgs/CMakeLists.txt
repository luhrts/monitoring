cmake_minimum_required(VERSION 2.8.3)
project(monitoring_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
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
  INCLUDE_DIRS 
  LIBRARIES  
  CATKIN_DEPENDS message_runtime
  DEPENDS
)
