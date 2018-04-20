#ifndef SRC_STATISTICSMONITOR_H_
#define SRC_STATISTICSMONITOR_H_

#include "ros/ros.h"
#include "rosgraph_msgs/TopicStatistics.h"
#include "ros_monitoring/monitors/monitormsg.h"



struct StatisticsInfo{
  std::string topic;
  float frequence;
  std::vector<std::string> subs;
  std::vector<std::string> pubs;
};

class StatisticMonitor
{
public:
  StatisticMonitor(ros::NodeHandle &n);
  ~StatisticMonitor();

private:
  void loadConfig(ros::NodeHandle &n);

  void statisticsCallback(rosgraph_msgs::TopicStatistics stats);

  ros::Subscriber stats_sub;
  MonitorMsg *msg;
  float freq;
  std::vector<StatisticsInfo> statisticData;
};
#endif /* SRC_STATISTICSMONITOR_H_ */
