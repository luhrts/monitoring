#ifndef SRC_STATISTICSMONITOR_H_
#define SRC_STATISTICSMONITOR_H_

#include "ros/ros.h"
#include "rosgraph_msgs/TopicStatistics.h"
#include "ros_monitoring/monitors/monitormsg.h"



struct StatisticsInfo{
  std::string topic;
  float frequency;
  std::string sub;
  std::string pub;
  int size;
  std::string type;
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

class StatisticMonitor
{
public:
  StatisticMonitor(ros::NodeHandle &n);
  ~StatisticMonitor();

private:
  void loadConfig(ros::NodeHandle &n);

  void statisticsCallback(rosgraph_msgs::TopicStatistics stats);
  void compareStatisticDataWithRequirements();
  ros::Subscriber stats_sub;
  MonitorMsg *msg;
  float freq;
  std::vector<StatisticsInfo> statisticData;
  std::vector<TopicRequirement> topicRequirements;
};
#endif /* SRC_STATISTICSMONITOR_H_ */
