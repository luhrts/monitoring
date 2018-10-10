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
#ifndef SRC_STATISTICSMONITOR_H_
#define SRC_STATISTICSMONITOR_H_

#include "ros/ros.h"
#include "rosgraph_msgs/TopicStatistics.h"
#include "monitoring_core/monitor.h"



struct StatisticsInfo{
  std::string topic;
  float frequency;
  std::string sub;
  std::string pub;
  int size;
  std::string type;
  ros::Time time;
};

struct TopicRequirement{
  std::string topic;
  std::string source;
  std::string destination;
  float frequency;
  int size;
  float dFrequency;
  int dSize;
  std::string type;
  float errorlevel;
};

/**
 * @brief The StatisticMonitor class uses ros statistics to monitor topics
 */
class StatisticMonitor
{
public:
  StatisticMonitor(ros::NodeHandle &n);
  ~StatisticMonitor();

private:
  /**
   * @brief loadConfig reads the parameter that are provided via the ros parameterserver and saves them to topicRequirements
   * @param n
   */
  void loadConfig(ros::NodeHandle &n);
  /**
   * @brief statisticsCallback saves all incoming statistic data in statisticData
   * @param stats incoming data
   */
  void statisticsCallback(rosgraph_msgs::TopicStatistics stats);
  /**
   * @brief compareStatisticDataWithRequirements compares statisticData and topicRequirements
   */
  void compareStatisticDataWithRequirements();
  void deleteOldMessages();
  ros::Subscriber stats_sub;  ///< subscirber for topic statistics
  Monitor *msg;   ///< msg that saves information that will be published
  float freq;   ///< working frequency
  std::vector<StatisticsInfo> statisticData;    ///< contains data from ros_statistics
  std::vector<TopicRequirement> topicRequirements;    ///< contains user defined topic requirements

  double timeTilDeletingOldMessages;
  AggregationStrategies aggregation;
  int monitor_mode;
};
#endif /* SRC_STATISTICSMONITOR_H_ */
