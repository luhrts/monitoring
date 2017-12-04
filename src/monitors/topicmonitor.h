/*
 * topicmonitor.h
 *
 *  Created on: Nov 22, 2017
 *      Author: matthias
 */

#ifndef SRC_TOPICMONITOR_H_
#define SRC_TOPICMONITOR_H_

#include "ros/ros.h"
#include "ros_monitoring/MonitoringInfo.h"

class TopicMonitor
{
public:
  TopicMonitor();
  virtual ~TopicMonitor();

  std::vector<std::string> getTopics();

private:
  std::vector<std::string> topicList;
};

#endif /* SRC_TOPICMONITOR_H_ */
