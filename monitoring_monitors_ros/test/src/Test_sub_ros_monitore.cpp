 #include "ros/ros.h"
 #include "std_msgs/String.h"
#include "std_msgs/Int16.h"





 void IntCallback(const std_msgs::Int16& msg)
 {

 }

 int main(int argc, char **argv)
 {

     ros::init(argc, argv, "sub_for_test_monitore");
      ros::NodeHandle n;
       ros::Subscriber TestSubMonitoreRosInt= n.subscribe("Test_topic_int16", 1000, IntCallback);


       ros::spin();

        return 0;
 }
