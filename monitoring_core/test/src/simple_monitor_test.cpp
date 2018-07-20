#include <gtest/gtest.h>
#include <ros/ros.h>
#include <monitoring_core/monitor.h>
#include <iostream>



TEST(MonitoringCore, addValueString)
{
    ros::NodeHandle nh;
    Monitor monitor_1(nh, "String_Test");

    for(int i=0;i<500;i++)
    {
        std::string Test_key(i, 'a');
        std::string Test_unit(i, 'b');
        std::string Test_value(i, 'c');

        monitor_1.addValue(Test_key, Test_value, Test_unit,0.0);
        ASSERT_TRUE(true);
        ASSERT_EQ(Test_key, monitor_1.ma.info[0].values[i].key);
        ASSERT_EQ(Test_unit, monitor_1.ma.info[0].values[i].unit);
        ASSERT_EQ(Test_value, monitor_1.ma.info[0].values[i].value);
    }
}

TEST(MonitoringCore, addValueFloatRand)
{
    ros::NodeHandle nh;

    Monitor monitor_2(nh, "Float_Max_Rand_Test");
    Monitor monitor_3(nh, "Float_Min_Rand_Test");
    Monitor monitor_4(nh, "Float_null_Rand_Test");

    monitor_2.addValue("Float_Test",3.402823466e+38f, "Float_Test",0.0);
    monitor_3.addValue("Float_Test",-3.402823466e+38f, "Float_Test",0.0);
    monitor_4.addValue("Float_Test",0, "Float_Test",0.0);

    ASSERT_LE("340280346638528859811704183484516925440.000000", monitor_2.ma.info[0].values[0].value);
    ASSERT_GE("-340284346638528859811704183484516925440.000000", monitor_3.ma.info[0].values[0].value);
    ASSERT_GE("340284306638528859811704183484516925440.000000", monitor_2.ma.info[0].values[0].value);
    ASSERT_LE("-340280346638528859811704183484516925440.000000", monitor_3.ma.info[0].values[0].value);
    ASSERT_EQ("0.000000", monitor_4.ma.info[0].values[0].value);
}

TEST(MonitoringCore, addValueFloat)
{
    ros::NodeHandle nh;
    Monitor monitor_5(nh, "Float_+_Test");
    Monitor monitor_6(nh, "Float_-_Test");
    float RandemNumber1;
    float RandemNumber2;
    for(int i=0;i<1000;i++)
    {
        float x=123.123*i;
        monitor_5.addValue("Float_Test",x, "Float_Test",0.0);
        monitor_6.addValue("Float_Test",-x, "Float_Test",0.0);
        RandemNumber1= atof(monitor_5.ma.info[0].values[i].value.c_str());
        RandemNumber2= atof(monitor_6.ma.info[0].values[i].value.c_str());

        ASSERT_EQ(RandemNumber1,x);
        ASSERT_EQ(RandemNumber2,-x);
    }

}
TEST(MonitoringCore, addValueErrorLevel)
{
    ros::NodeHandle nh;

    Monitor monitor_7(nh, "ErrorLevel_Test");

    for(int i=0;i<=4;i++)
    {
        float y=0.0;
        ++y;
        monitor_7.addValue("ErrorLevel_Test",0.0, "ErrorLevel_Test",y);
        ASSERT_EQ(monitor_7.ma.info[0].values[i].errorlevel,y);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Test_monitor_core");

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
