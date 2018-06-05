#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"

   int main(int argc, char **argv)
   {

     ros::init(argc, argv, "Test_pub_monitore_ros");

     ros::NodeHandle n;

     //ros::Publisher TestPubMonitoreRosString = n.advertise<std_msgs::String>("/Test_topic_string", 1000);
     ros::Publisher TestPubMonitoreRosInt = n.advertise<std_msgs::Int16>("/Test_topic_int16", 1000);



     ros::Rate loop_rate(10);

     std_msgs::String  Test_msgs_string;

     std_msgs::Int16 Test_msgs_int;

     //Test_msgs_string.data="Test_msg";

     Test_msgs_int.data=123;


   while(ros::ok())
     {



      //TestPubMonitoreRosString.publish(Test_msgs_string);
      //ros::spinOnce();


      TestPubMonitoreRosInt.publish(Test_msgs_int);

      ros::spinOnce();

      loop_rate.sleep();
    }



  }
