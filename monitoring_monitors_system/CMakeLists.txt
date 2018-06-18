cmake_minimum_required(VERSION 2.8.3)
project(monitoring_monitors_system)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  monitoring_core
  roscpp
  rospy
)

find_library(PROCPS procps)
if(NOT PROCPS_FOUND)
      message(WARNING "[MOnitoring] libprocps-dev not found, RAM Monitor nodes will not be build")
else()
    include_directories(${PROCPS_INCLUDE_DIRS})
endif()


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS monitoring_core
  DEPENDS PROCPS
)


include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(cpu_monitoring_node src/cpumonitor.cpp)
target_link_libraries(cpu_monitoring_node
  ${catkin_LIBRARIES}
)

add_executable(cpu_frequency_monitor_node src/cputhrottlemonitor.cpp)
target_link_libraries(cpu_frequency_monitor_node
${catkin_LIBRARIES}

)

if(PROCPS_FOUND)
    add_executable(ram_monitoring_node src/rammonitor.cpp)
    target_link_libraries(ram_monitoring_node
      ${catkin_LIBRARIES}
      ${PROCPS_LIBRARIES}
    )
endif()

add_executable(network_monitoring_node src/networkmonitor.cpp)
target_link_libraries(network_monitoring_node
  ${catkin_LIBRARIES}
)

add_executable(wifi_signal_strength_monitoring_node src/wifistrengthmonitor.cpp)
target_link_libraries(wifi_signal_strength_monitoring_node
  ${catkin_LIBRARIES}
)










