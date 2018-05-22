cmake_minimum_required(VERSION 2.8.3)
project(monitoring_monitors_system)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  monitoring_core
  roscpp
  rospy
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(PROCPS REQUIRED libprocps)


catkin_package(
  INCLUDE_DIRS include

 DEPENDS PROCPS
)


include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${PROCPS_INCLUDE_DIRS}
)

add_executable(cpu_monitoring_node src/cpumonitor.cpp)
add_executable(ram_monitoring_node src/rammonitor.cpp)
add_executable(network_monitoring_node src/networkmonitor.cpp)

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

