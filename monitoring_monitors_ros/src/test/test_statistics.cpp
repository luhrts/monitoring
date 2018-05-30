#include <gtest/gtest.h>
#include <ros/ros.h>
#include <monitoring_monitors_ros/statisticsmonitor.h>
std::string value;


class Test_class
{
    ros::Subscriber Test_sub;
    ros::Publisher Test_pub;


public:
  Test_class (ros::NodeHandle& n)
{
      Test_pub = n.advertise<rosgraph_msgs::TopicStatistics> ("/statistics", 100);
      Test_sub = n.subscribe("/monitoring", 1, &Test_class::TestCallback,this);

  }

  virtual ~Test_class(){}

void pub(rosgraph_msgs::TopicStatistics Test_ts)
{

    ros::Duration(2).sleep();
    ros::Rate loop_rate(10);

    for(int i=0;i<=10;i++)
       {
     Test_pub.publish(Test_ts);
     loop_rate.sleep();
     ros::spinOnce();

       }


}

 void TestCallback(const monitoring_msgs::MonitoringArray& Callback_Test_msg)
    {

     ASSERT_TRUE(true);

     monitoring_msgs::MonitoringArray msg;
     msg=Callback_Test_msg;
       if(msg.info[0].values[0].key == "Pub: ")
        {
           if(msg.info[0].values[0].value=="velodyne")
           {
               ASSERT_TRUE(true);
           }
           if(msg.info[0].values[0].value=="/teleop_turtle")
           {
               ASSERT_TRUE(true);
           }
    //  ASSERT_EQ(msg.info[0].values[0].value,"velodyne");
        }
       else if(msg.info[0].values[0].key == "Sub:")
        {
           if(msg.info[0].values[0].value=="pc_to_pc2")
           {
               ASSERT_TRUE(true);
           }
           if(msg.info[0].values[0].value=="/turtlesim")
           {
               ASSERT_TRUE(true);         }
           else if(msg.info[0].values[0].key == "Topic Missing")
           {
                 ASSERT_TRUE(true);
           }

        else
       {
                ASSERT_TRUE(false);
           }
       }
    }
};





TEST(Ros_Monitore, Monitore_Test_statistics_1)
{
    ros::NodeHandle nh;
    Test_class Test_1(nh);
    std::string Test_msgs;
    std::string Test_Topic;
    rosgraph_msgs::TopicStatistics Test_ts;
    Test_Topic="/scan";
    Test_ts.topic=Test_Topic;
    Test_ts.node_pub="velodyne";
    Test_ts.node_sub="pc_to_pc2";


     Test_1.pub(Test_ts);


     ASSERT_TRUE(true);




}

TEST(Ros_Monitore, Monitore_Test_statistics_2)
{
    ros::NodeHandle nh;
    Test_class Test_1(nh);
    std::string Test_msgs;
    std::string Test_Topic;
    rosgraph_msgs::TopicStatistics Test_ts;
    Test_Topic="/turtle1/cmd_vel";
    Test_ts.topic=Test_Topic;
    Test_ts.node_pub="/teleop_turtle";
    Test_ts.node_sub="/turtlesim";


     Test_1.pub(Test_ts);


     ASSERT_TRUE(true);



}




int main(int argc, char **argv) {
    ros::init(argc, argv, "ros_monitore_test");
    testing::InitGoogleTest(&argc, argv);





    return RUN_ALL_TESTS();
}

