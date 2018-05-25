#include <gtest/gtest.h>
#include <ros/ros.h>
#include <monitoring_core/monitor.h>


TEST(MonitoringCore, addValueString) {
    ros::NodeHandle nh;
    Monitor monitor_1(nh, "String_Test");

  for(int i=0;i<500;i++)
  {
      string Test_key(i, 'a');
      string Test_unit(i, 'b');
      string Test_value(i, 'b');




    monitor_1.addValue(Test_key, Test_value, Test_unit,0.0);

    ASSERT_TRUE(true);
  }
}

TEST(MonitoringCore, addValueFloat) {
    ros::NodeHandle nh;

    Monitor monitor_2(nh, "Float_Test");

    monitor_1.addValue("char", y, "Char unit",0.0);
    monitor_1.publish();

    ASSERT_TRUE(true);
}

TEST(MonitoringCore, addValueString_empty) {
    ros::NodeHandle nh;

    Monitor monitor_1(nh, "Gao_monitor_int_example");

    std::string y="";

    monitor_1.addValue("char", y, "Char unit",0.0);
    monitor_1.publish();

    ASSERT_TRUE(true);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Gao_monitor");


    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
