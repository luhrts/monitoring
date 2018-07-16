<<<<<<< HEAD
#include <gtest/gtest.h>
=======
﻿#include <gtest/gtest.h>
>>>>>>> 7e980397b2465727873c6d2b1bf0f8bcda5b7b15
#include <ros/ros.h>
#include <monitoring_monitors_ros/statisticsmonitor.h>
#include<iostream>

<<<<<<< HEAD
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
=======
class Data_Check_class
{
    ros::Subscriber Test_sub_for_ros_monitoring;
    std::vector<monitoring_msgs::MonitoringArray> Data;

public:
  Data_Check_class (ros::NodeHandle& n)
{
      Test_sub_for_ros_monitoring = n.subscribe("/monitoring", 1, &Data_Check_class::TestCallback,this);
  }

  virtual ~Data_Check_class(){}

  void waitAndSpin(int iterations = 500){
    int count = 0;
    ros::Rate loop_rate(10);
    while(ros::ok() && count++ < iterations){
      ros::spinOnce();
      loop_rate.sleep();
    }
  }


 void TestCallback(const monitoring_msgs::MonitoringArray& Callback_Test_msg)
 {
    Data.push_back(Callback_Test_msg);

 }



 void Data_check_topic_int16()
 {

       for (monitoring_msgs::MonitoringArray array : Data){
         for (monitoring_msgs::MonitoringInfo info : array.info){
             EXPECT_EQ(info.name,"/statistics_monitoring");
             EXPECT_EQ(info.description,"Statistics for ROS Topics");
           for (monitoring_msgs::KeyValue keyValue: info.values){
             if (keyValue.key == "Pub: "){
                    EXPECT_EQ(keyValue.value,"pub_for_test");
             }
            else if (keyValue.key == "Sub: "){
                   EXPECT_EQ(keyValue.value,"sub_for_test");
             }
            else if (keyValue.key == "Test_topic_int16/TopicMissing"){
                  EXPECT_EQ(keyValue.errorlevel,1.0);
             }
            else if (keyValue.key == "Test_topic_int16/frequency" && !keyValue.errorlevel == 0.5){
                    EXPECT_LE("10.2",keyValue.value);
                    EXPECT_GE("9.8",keyValue.value);
             }
            else if (keyValue.key == "Test_topic_int16/frequency" && keyValue.errorlevel == 0.5){
                    EXPECT_LE("9.8",keyValue.value);
                    EXPECT_GE("10.2",keyValue.value);
             }
            else if (keyValue.key == "Test_topic_int16/size" && keyValue.errorlevel == 0.5){
                  EXPECT_EQ("0.000000",keyValue.value);
             }
            else if (keyValue.key == "Test_topic_int16/size" && !keyValue.errorlevel == 0.5){
                 EXPECT_LE("3",keyValue.value);
                 EXPECT_GE("1",keyValue.value);
             }
            else
             {
                 EXPECT_TRUE(false);
             }
         }
       }
      }
 }

>>>>>>> 7e980397b2465727873c6d2b1bf0f8bcda5b7b15
};




<<<<<<< HEAD

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
=======
TEST(Ros_Monitore, Monitore_Test_statistics_int16)
{
    ros::NodeHandle nh;
    Data_Check_class Data_Check(nh);
    Data_Check.Data_check_topic_int16();

>>>>>>> 7e980397b2465727873c6d2b1bf0f8bcda5b7b15



}




int main(int argc, char **argv) {
<<<<<<< HEAD
  ros::init(argc, argv, "ros_monitore_test");
  testing::InitGoogleTest(&argc, argv);
=======
    ros::init(argc, argv, "ros_monitore_test");
    testing::InitGoogleTest(&argc, argv);
>>>>>>> 7e980397b2465727873c6d2b1bf0f8bcda5b7b15





<<<<<<< HEAD
  return RUN_ALL_TESTS();
=======
    return RUN_ALL_TESTS();
>>>>>>> 7e980397b2465727873c6d2b1bf0f8bcda5b7b15
}

