cmake_minimum_required(VERSION 2.8.3)
project(monitoring_monitors_system)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  monitoring_core
  roscpp
  rospy
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(PROCPS libprocps)
if(NOT PROCPS_FOUND)
      message(WARNING "[Monitoring] libprocps-dev not found, RAM Monitor nodes will not be build")
else()
    include_directories(${PROCPS_INCLUDE_DIRS})
endif()


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS monitoring_core
)


include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 external/ping_util
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

add_executable(wifi_monitor src/wifi_monitor.cpp)
target_link_libraries(wifi_monitor
  ${catkin_LIBRARIES}
)

add_executable(clockdiff_monitor_node src/clockdiffmonitor.cpp)
target_link_libraries(clockdiff_monitor_node
  ${catkin_LIBRARIES}
)

add_executable(ping_monitor external/ping_util/ping.cpp src/ping_monitor.cpp)
target_link_libraries(ping_monitor ${Boost_LIBRARIES} ${catkin_LIBRARIES})

string(ASCII 27 ESCAPE)
add_custom_command(
  OUTPUT message
  COMMAND sudo -n sh -c 'cd ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}\; chown root.root ping_monitor\; chmod 4755 ping_monitor' || (echo "${ESCAPE}[34m#\ type\ following\ command\ before\ execute\ ping_monitor...\ sudo\ chown\ root.root\ ./bin/ping_monitor\;\ sudo\ chmod 4755\ ./bin/ping_monitor${ESCAPE}[0m")
  DEPENDS bin/ping_monitor)
add_custom_target(message_all ALL DEPENDS message)

install(CODE
  "execute_process(COMMAND sudo -n sh -c \"cd \$ENV{DESTDIR}/${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION} && chown root.root ping_monitor && ls -al ping_monitor && chmod 4755 ping_monitor && ls -al ping_monitor\")
")

install(TARGETS cpu_monitoring_node cpu_frequency_monitor_node ram_monitoring_node network_monitoring_node wifi_monitor clockdiff_monitor_node ping_monitor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} 
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
catkin_install_python(PROGRAMS scripts/ping_monitor.py scripts/ntp_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

