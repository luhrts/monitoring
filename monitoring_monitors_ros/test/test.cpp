#include <gtest/gtest.h>
#include <ros/ros.h>
#include <monitoring_core/monitor.h>
#include <monitoring_monitors_ros/statisticsmonitor.h>
uint i;
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
      Test_sub = n.subscribe("/monitoring", 100, &Test_class::TestCallback,this);
  }

  virtual ~Test_class(){}

void pub()
{
    ros::Rate loop_rate(10);

    //for ( i=0;i<=10;i++)
    // {
     Test_pub.publish(Test_ts);
     ros::spinOnce();
     loop_rate.sleep();
   //  }


}

 void TestCallback(const monitoring_msgs::MonitoringArray& Test_msg)
    {
       monitoring_msgs::MonitoringArray msg;
        msg = Test_msg;
        if(msg.info[0].values[i].key == "pub:")
        {
       ASSERT_EQ(msg.info[0].values[i].value,"TEST_PUB");
        }
        else if(msg.info[0].values[i].key == "sub:")
        {
       ASSERT_EQ(msg.info[0].values[i].value,"TEST_SUB");
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
//    StatisticMonitor Test_SM(nh);


    Test_msgs="TEST_PUB";
    Test_Topic="TEST_TOPIC";
    Test_ts.topic=Test_Topic;
    Test_ts.node_pub=Test_msgs;

    Test_1.pub();

    ASSERT_TRUE(true);

}

TEST(Ros_Monitore, Monitore_Test_sub)
{
    ros::NodeHandle nh;
    Test_class Test_1(nh);
   //StatisticMonitor Test_SM(nh);
    Test_msgs="TEST_SUB";
    Test_Topic="TEST_TOPIC";
    Test_ts.topic=Test_Topic;
    Test_ts.node_sub=Test_msgs;

   Test_1.pub();


    ASSERT_TRUE(true);

}




int main(int argc, char **argv) {
    ros::init(argc, argv, "Test_monitore_ros");
    testing::InitGoogleTest(&argc, argv);





    return RUN_ALL_TESTS();
}

