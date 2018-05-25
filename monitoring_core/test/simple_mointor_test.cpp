#include <gtest/gtest.h>
#include <ros/ros.h>
#include <monitoring_core/monitor.h>


TEST(MonitoringCore, addValueString)
{
    ros::NodeHandle nh;
    Monitor monitor_1(nh, "String_Test");

  for(int i=0;i<500;i++)
  {
      std::string Test_key(i, 'a');
      std::string Test_unit(i, 'b');
      std::string Test_value(i, 'b');




    monitor_1.addValue(Test_key, Test_value, Test_unit,0.0);

    ASSERT_TRUE(true);
  }
}

TEST(MonitoringCore, addValueFloatRand)
{
    ros::NodeHandle nh;

    Monitor monitor_2(nh, "Float_Max_Rand_Test");
    Monitor monitor_3(nh, "Float_Min_Rand_Test");
    Monitor monitor_4(nh, "Float_null_Rand_Test");


    monitor_2.addValue("Float_Test",3.402823466e+38f, "Float_Test",0.0);
    monitor_3.addValue("Float_Test",1.175494351e-38f, "Float_Test",0.0);
    monitor_4.addValue("Float_Test",0, "Float_Test",0.0);


    ASSERT_TRUE(true);
}

TEST(MonitoringCore, addValueFloat)
{
    ros::NodeHandle nh;

    for(int i=0;i<20;i++)
    {
        float x=1234.1234*i;
        Monitor monitor_5(nh, "Float_+_Test");
        Monitor monitor_6(nh, "Float_-_Test");


        monitor_5.addValue("Float_Test",x, "Float_Test",0.0);
        monitor_6.addValue("Float_Test",-x, "Float_Test",0.0);


        ASSERT_TRUE(true);
    }

}
TEST(MonitoringCore, addValueErrorLevel) {
    ros::NodeHandle nh;

    Monitor monitor_7(nh, "ErrorLevel_Test");

    for(int i=0;i<=4;i++)
    {
       float y=0;
       ++y;
       monitor_7.addValue("ErrorLevel_Test",0.0, "ErrorLevel_Test",y);

      ASSERT_TRUE(true);
    }


}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Gao_monitor");


    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
