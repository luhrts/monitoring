/*
 * Monitor.h
 *
 *  Created on: Feb 5, 2018
 *      Author: matthias
 */

#ifndef SRC_MONITORS_MONITOR_H_
#define SRC_MONITORS_MONITOR_H_
#include "gtest/gtest_prod.h"  
#include "monitoring_msgs/MonitoringArray.h"
#include "ros/ros.h"

enum AggregationStrategies{
    LAST,
    FIRST,
    MIN,
    MAX,
    AVG
};
struct Sum{
  float sum;
  int num;
};

class Monitor {
public:
    Monitor();
    Monitor(std::string monitorDescription, bool autoPublishing = true);
    Monitor(ros::NodeHandle &n, std::string monitorDescription, bool autoPublishing = true);
    ~Monitor();

    void addValue(std::string key, std::string value, std::string unit, float errorlevel, AggregationStrategies aggregation = LAST);
    void addValue(std::string key, float value, std::string unit, float errorlevel, AggregationStrategies aggregation = LAST);
    void publish();

    void resetMsg();

private:
    Sum sum_;
    void init(std::string monitorDescription);
    void initROS(ros::NodeHandle &n, bool autoPublishing);

    ros::Publisher pub;
    ros::Timer timer;   //takes the publishing frequency
    void timerCallback(const ros::TimerEvent& te);

    monitoring_msgs::MonitoringArray ma;
    std::string node_name_;
    std::string monitor_description_;
    int miIndex;

    std::string host_name_;

    ///////////////////Gtest/////////////////////
    FRIEND_TEST(MonitoringCore, addValueString);
    FRIEND_TEST(MonitoringCore, addValueFloatRand);
    FRIEND_TEST(MonitoringCore, addValueFloat);
    FRIEND_TEST(MonitoringCore, addValueErrorLevel);
    FRIEND_TEST(MonitoringCore, aggregationLast);
    FRIEND_TEST(MonitoringCore, aggregationFirst);
    FRIEND_TEST(MonitoringCore, aggregationMin);
    FRIEND_TEST(MonitoringCore, aggregationMax);



};

#endif /* SRC_MONITORS_MONITOR_H_ */
