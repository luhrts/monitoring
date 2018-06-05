 #include "ros/ros.h"
 #include "std_msgs/String.h"
#include "std_msgs/Int16.h"



 void StringCallback(const std_msgs::String& msg)
 {

 }

 void IntCallback(const std_msgs::Int16& msg)
 {

 }

 int main(int argc, char **argv)
 {

     ros::init(argc, argv, "sub_for_test");
      ros::NodeHandle n;
      // ros::Subscriber Sub_test_Int = n.subscribe("Test_topic_string", 1000, StringCallback);
       ros::Subscriber Sub_test_String= n.subscribe("Test_topic_int16", 1000, IntCallback);


       ros::spin();

        return 0;
 }
