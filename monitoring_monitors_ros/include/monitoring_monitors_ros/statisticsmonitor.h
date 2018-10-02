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
