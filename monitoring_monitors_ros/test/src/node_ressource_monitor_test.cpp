#include <gtest/gtest.h>
#include <ros/ros.h>
#include <monitoring_msgs/MonitoringArray.h>
#include <monitoring_core/utils/monitoring_subscriber.h>
#include <iostream>


TEST(Node_Ressource_Monitor, cpu_percent)
{
    ros::NodeHandle nh;
    MonitoringSubscriber test(nh);
    test.waitAndSpin();

    EXPECT_GT(test.receivedArrays_.size(), 0);
    EXPECT_TRUE(test.hasReceivedKey("cpu_percent"));


}

int main(int argc, char **argv) {
    ros::init(argc, argv, "node_resouce_monitor_test");
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
