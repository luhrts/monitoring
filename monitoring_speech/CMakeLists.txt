cmake_minimum_required(VERSION 2.8.3)
project(monitoring_speech)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)


catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)

add_executable(speech_output src/speech_output.cpp)
target_link_libraries(speech_output
   ${catkin_LIBRARIES}
 )

install(TARGETS speech_output
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


