cmake_minimum_required(VERSION 2.8.3)
project(monitoring_fdir)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  monitoring_msgs
  roscpp
  rospy
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fdiSDK recoverySDK
  CATKIN_DEPENDS monitoring_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



add_library(fdiSDK
  src/fdi/fdiSDK.cpp
  src/fdi/configinterface.cpp
  src/fdi/std_validator/max.cpp
  src/fdi/std_validator/min.cpp
  src/fdi/std_validator/between.cpp
  src/fdi/std_validator/nodeavailable.cpp
)

target_link_libraries(fdiSDK
    ${catkin_LIBRARIES}
 )
add_dependencies(fdiSDK ${catkin_EXPORTED_TARGETS})


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
add_dependencies(recoverySDK ${catkin_EXPORTED_TARGETS})

add_executable(fdi_example_node src/fdi/fdiExample.cpp)

target_link_libraries(fdi_example_node
  ${catkin_LIBRARIES}

  fdiSDK
)


add_executable(recovery_example_node src/recovery/recoveryExample.cpp)

target_link_libraries(recovery_example_node
  ${catkin_LIBRARIES}

  recoverySDK
)

install(TARGETS recoverySDK fdiSDK
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

