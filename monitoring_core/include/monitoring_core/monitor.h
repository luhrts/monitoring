/*
 *  Copyright (c) 2018, University of Hannover
 *                      Institute for Systems Engineering - RTS
 *                      Professor Bernardo Wagner
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
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
  float errl;
};

struct Value{
    float value;
    float errl;
    std::string unit;
};

class Monitor {
public:
    Monitor();
    Monitor(std::string monitorDescription, bool autoPublishing = true);
    Monitor(ros::NodeHandle &n, std::string monitorDescription, bool autoPublishing = true);
    ~Monitor();

    void addValue(std::string key, std::string value, std::string unit, float errorlevel, AggregationStrategies aggregation = LAST);
    void addValue(std::string key, float value, std::string unit, float errorlevel, AggregationStrategies aggregation = LAST);
    void addValue(std::string key, int value, std::string unit, float errorlevel, AggregationStrategies aggregation = LAST);
    void addValue(std::string key, long unsigned int value, std::string unit, float errorlevel, AggregationStrategies aggregation = LAST);
    void addValue(std::string key, double value, std::string unit, float errorlevel, AggregationStrategies aggregation = LAST);
    void publish();

    void resetMsg();

private:

    void init(std::string monitorDescription);
    void initROS(ros::NodeHandle &n, bool autoPublishing);
    void initNewData(std::string key, std::string value, std::string unit, float errorlevel);
    void initNewNumericalData(std::string key, float value, std::string unit, float errorlevel);
    void timerCallback(const ros::TimerEvent& te);
    std::string floatToString(float value);
    void writeDataToMsg();

    monitoring_msgs::MonitoringArray ma;
    std::string node_name_;
    std::string monitor_description_;
    int miIndex;
    ros::Publisher pub;
    ros::Timer timer;   //takes the publishing frequency
    std::string host_name_;
    std::map<std::string, Sum> values_for_avg_;
    std::map<std::string, Value> current_values_;

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
