cmake_minimum_required(VERSION 2.8.3)
project(monitoring_core)

## Compile as C++11, supported in ROS Kinetic and newer

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  monitoring_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS monitoring_msgs
  DEPENDS
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/monitor.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME} monitoring_msgs_generate_messages_cpp)


catkin_python_setup()

add_executable(performance_monitor_test_node test/src/performance_monitor_test_node.cpp)
target_link_libraries(performance_monitor_test_node ${catkin_LIBRARIES} ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)

    find_package(rostest REQUIRED)

    catkin_add_gtest(simple_monitor_test test/src/simple_monitor_test.cpp)
    target_link_libraries(simple_monitor_test
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
    )

    add_rostest_gtest(performance_monitor_test test/launch/performance_monitor_test.test test/src/performance_monitor_test.cpp)
    target_link_libraries(performance_monitor_test
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
    )

endif()
