#include <gtest/gtest.h>
#include <ros/ros.h>
#include <monitoring_monitors_ros/statisticsmonitor.h>
#include<iostream>

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
            else if ((keyValue.key == "Test_topic_int16/frequency") && (keyValue.errorlevel != 0.5)){
                    EXPECT_LE("10.2",keyValue.value);
                    EXPECT_GE("9.8",keyValue.value);
             }
            else if ((keyValue.key == "Test_topic_int16/frequency") && (keyValue.errorlevel == 0.5)){
                    EXPECT_LE("9.8",keyValue.value);
                    EXPECT_GE("10.2",keyValue.value);
             }
            else if ((keyValue.key == "Test_topic_int16/size") && (keyValue.errorlevel == 0.5)){
                  EXPECT_EQ("0.000000",keyValue.value);
             }
            else if ((keyValue.key == "Test_topic_int16/size") && (keyValue.errorlevel != 0.5)){
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

};





TEST(Ros_Monitore, Monitore_Test_statistics_int16)
{
    ros::NodeHandle nh;
    Data_Check_class Data_Check(nh);
    Data_Check.Data_check_topic_int16();




}



int main(int argc, char **argv) {
  ros::init(argc, argv, "ros_monitore_test");
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

