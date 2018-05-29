#include <gtest/gtest.h>
#include <ros/ros.h>
#include <monitoring_monitors_ros/statisticsmonitor.h>
#include<iostream>
std::string Test_msgs;
std::string Test_Topic;
rosgraph_msgs::TopicStatistics Test_ts;
ros::Subscriber Test_sub;
ros::Publisher Test_pub;



class Test_class
{
public:
  Test_class (ros::NodeHandle& n)
{
      Test_pub = n.advertise<rosgraph_msgs::TopicStatistics> ("/statistics", 100);
      Test_sub = n.subscribe("/monitoring", 1, &Test_class::TestCallback,this);
      ros::spinOnce();

  }

  virtual ~Test_class(){}

void pub()
{

    ros::Duration(1).sleep();
    ros::Rate loop_rate(1);

    for(int i=0;i<=10;i++)
       {
     Test_pub.publish(Test_ts);
     loop_rate.sleep();
     ros::spinOnce();

       }


}

 void TestCallback(const monitoring_msgs::MonitoringArray& Test_msg)
    {
   //  EXPECT_TRUE(false);
     monitoring_msgs::MonitoringArray msg;
     msg=Test_msg;

       if(msg.info[0].values[0].key == "Pub: ")
        {
      ASSERT_EQ(msg.info[0].values[0].value,"TEST_PUB");
        }
       else if(msg.info[0].values[0].key == "sub:")
        {
       ASSERT_EQ(msg.info[0].values[0].value,"TEST_SUB");
         }
        else
       {
           ASSERT_TRUE(false);
       }
    }
};





TEST(Ros_Monitore, Monitore_Test_pub)
{
    ros::NodeHandle nh;
    Test_class Test_1(nh);

    Test_msgs="TEST_PUB";
    Test_Topic="TEST_TOPIC";
    Test_ts.topic=Test_Topic;
    Test_ts.node_pub=Test_msgs;

     Test_1.pub();





}

TEST(Ros_Monitore, Monitore_Test_sub)
{
    ros::NodeHandle nh;
    Test_class Test_1(nh);
    Test_msgs="TEST_SUB";
    Test_Topic="TEST_TOPIC";
    Test_ts.topic=Test_Topic;
    Test_ts.node_sub=Test_msgs;

      Test_1.pub();


}




int main(int argc, char **argv) {
    ros::init(argc, argv, "ros_monitore_test");
    testing::InitGoogleTest(&argc, argv);





    return RUN_ALL_TESTS();
}

