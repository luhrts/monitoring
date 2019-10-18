cmake_minimum_required(VERSION 2.8.3)
project(monitoring_monitors_ros)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  monitoring_core
  roscpp
  rospy
  tf
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


add_executable(tf_monitoring_node src/tf_monitor.cpp)
target_link_libraries(tf_monitoring_node
  ${catkin_LIBRARIES}
)

add_executable(pub_for_test  test/src/Test_pub_ros_monitore.cpp )
target_link_libraries(pub_for_test
  ${catkin_LIBRARIES}
)

add_executable(sub_for_test  test/src/Test_sub_ros_monitore.cpp )
target_link_libraries(sub_for_test
  ${catkin_LIBRARIES}
)

add_executable(topic_value_monitor src/topic_value_monitor.cpp)
target_link_libraries(topic_value_monitor
  ${catkin_LIBRARIES}
)
 
install(TARGETS statistics_monitoring_node tf_monitoring_node topic_value_monitor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/node_ressource_monitor.py scripts/nodemonitor.py scripts/topicmonitor.py scripts/monitorstatistics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(ros_statistics_monitor_test test/launch/ros_statistics_monitor_test.test test/src/ros_statistics_monitor_test.cpp)
  target_link_libraries(ros_statistics_monitor_test ${catkin_LIBRARIES})

  add_rostest_gtest(node_ressource_monitor_test test/launch/node_ressource_monitor.test test/src/node_ressource_monitor_test.cpp)
  target_link_libraries(node_ressource_monitor_test ${catkin_LIBRARIES})
endif()


