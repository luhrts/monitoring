[1mdiff --git a/monitoring_monitors_ros/CMakeLists.txt b/monitoring_monitors_ros/CMakeLists.txt[m
[1mindex 6a9cde4..3b53691 100644[m
[1m--- a/monitoring_monitors_ros/CMakeLists.txt[m
[1m+++ b/monitoring_monitors_ros/CMakeLists.txt[m
[36m@@ -8,7 +8,7 @@[m [mfind_package(catkin REQUIRED COMPONENTS[m
   monitoring_core[m
   roscpp[m
   rospy[m
[31m-  [m
[32m+[m[32m  tf[m
 )[m
 [m
 catkin_package([m
[36m@@ -24,6 +24,8 @@[m [minclude_directories([m
 [m
 add_executable(statistics_monitoring_node src/statisticsmonitor.cpp)[m
 [m
[32m+[m[32madd_executable(tf_monitoring_node src/tf_monitor.cpp)[m
[32m+[m
 add_executable(pub_for_test  test/src/Test_pub_ros_monitore.cpp )[m
 [m
 add_executable(sub_for_test  test/src/Test_sub_ros_monitore.cpp )[m
[36m@@ -32,6 +34,9 @@[m [madd_executable(sub_for_test  test/src/Test_sub_ros_monitore.cpp )[m
 target_link_libraries(statistics_monitoring_node [m
   ${catkin_LIBRARIES}[m
 )[m
[32m+[m[32mtarget_link_libraries(tf_monitoring_node[m[41m [m
[32m+[m[32m  ${catkin_LIBRARIES}[m
[32m+[m[32m)[m
 target_link_libraries(pub_for_test [m
   ${catkin_LIBRARIES}[m
 )[m
