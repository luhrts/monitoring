cmake_minimum_required(VERSION 2.8.3)
project(monitoring_monitors_ros)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  monitoring_core
  roscpp
  rospy
  
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS monitoring_core
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)


add_executable(statistics_monitoring_node src/statisticsmonitor.cpp)
target_link_libraries(statistics_monitoring_node
  ${catkin_LIBRARIES}
)

add_executable(topic_value_monitor src/topic_value_monitor.cpp)
target_link_libraries(topic_value_monitor
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(ros_statistics_monitor_test test/launch/ros_statistics_monitor_test.test test/src/ros_statistics_monitor_test.cpp)
  target_link_libraries(ros_statistics_monitor_test ${catkin_LIBRARIES})

  add_rostest_gtest(node_ressource_monitor_test test/launch/node_ressource_monitor.test test/src/node_ressource_monitor_test.cpp)
  target_link_libraries(node_ressource_monitor_test ${catkin_LIBRARIES})
endif()


