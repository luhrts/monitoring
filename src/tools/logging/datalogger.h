/*
 * datalogger.h
 *
 *  Created on: Dec 14, 2017
 *      Author: matthias
 */

#ifndef SRC_TOOLS_LOGGING_DATALOGGER_H_
#define SRC_TOOLS_LOGGING_DATALOGGER_H_

#include "ros/ros.h"
#include "ros_monitoring/MonitoringInfo.h"
#include "ros_monitoring/Error.h"

#include <iostream>
#include <fstream>
#include <queue>

class DataLogger
{
public:
  DataLogger(std::string filename);
  virtual ~DataLogger();

  void writeData();

private:
  void callbackMonitors(ros_monitoring::MonitoringInfo mi);
  void callbackErrors(ros_monitoring::Error errors);

  void writeSystemInformation();
  ros::Subscriber sub_monitoring, sub_errors;
  std::string filename;
  std::queue<std::string> logBuffer;
};

#endif /* SRC_TOOLS_LOGGING_DATALOGGER_H_ */
