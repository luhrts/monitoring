#include <gtest/gtest.h>
#include <ros/ros.h>
#include <monitoring_core/utils/monitoring_subscriber.h>
#include <iostream>



TEST(MonitoringCore, performanceTestCpp100)
{
    ros::NodeHandle nh;
    MonitoringSubscriber test(nh);
    test.waitAndSpin();

    EXPECT_GT(test.receivedArrays_.size(), 0);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "performance_monitor_test");

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
