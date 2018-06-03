#include <gtest/gtest.h>
#include <ros/ros.h>
#include <monitoring_monitors_ros/statisticsmonitor.h>
#include<iostream>

class Test_class
{
  ros::Subscriber Test_sub;
  ros::Publisher Test_pub;


public:
  Test_class (ros::NodeHandle& n)
  {
    Test_pub = n.advertise<rosgraph_msgs::TopicStatistics> ("/statistics", 100);
    Test_sub = n.subscribe("/monitoring", 100, &Test_class::TestCallback,this);

  }

  virtual ~Test_class(){}

  void pub(rosgraph_msgs::TopicStatistics Test_ts)
  {

    ros::Duration(3).sleep();
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
    // ASSERT_TRUE(false);

    monitoring_msgs::MonitoringArray msg;
    msg=Callback_Test_msg;

    for(uint j=0;j<msg.info[0].values.size();j++)
    {


      if(msg.info[0].values[j].key == "Pub: ")
      {

        if(!(msg.info[0].values[j].value=="velodyne" || msg.info[0].values[j].value=="/teleop_turtle"))
        {
          ASSERT_TRUE(false);
        }
      }
      if(msg.info[0].values[j].key == "Sub: ")
      {
        if(!(msg.info[0].values[j].value=="pc_to_pc2" || msg.info[0].values[j].value=="/turtlesim"))
        {
          ASSERT_TRUE(false);
        }
      }
      if(msg.info[0].values[j].key == "Topic Missing")
      {
        ASSERT_EQ(msg.info[0].values[j].errorlevel,1.0);
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
  // Test_ts.window_start=ros::Duration(0.5);
  //Test_ts.window_stop=ros::Duration(0.0);
  Test_ts.delivered_msgs=1;
  Test_ts.dropped_msgs=1;
  Test_ts.period_mean=ros::Duration(0.5);
  Test_ts.traffic=1;
  Test_ts.period_stddev=ros::Duration(0.1);
  Test_ts.period_max=ros::Duration(1);
  Test_ts.stamp_age_stddev=ros::Duration(0.1);
  Test_ts.stamp_age_max=ros::Duration(1);

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
  // Test_ts.window_start=ros::Duration(0.5);
  //Test_ts.window_stop=ros::Duration(0.0);
  Test_ts.delivered_msgs=1;
  Test_ts.dropped_msgs=1;
  Test_ts.period_mean=ros::Duration(0.5);
  Test_ts.traffic=1;
  Test_ts.period_stddev=ros::Duration(0.1);
  Test_ts.period_max=ros::Duration(1);
  Test_ts.stamp_age_stddev=ros::Duration(0.1);
  Test_ts.stamp_age_max=ros::Duration(1);


  Test_1.pub(Test_ts);


  ASSERT_TRUE(true);



}




int main(int argc, char **argv) {
  ros::init(argc, argv, "ros_monitore_test");
  testing::InitGoogleTest(&argc, argv);





  return RUN_ALL_TESTS();
}

