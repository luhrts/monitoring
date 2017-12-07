/*
 * FdiSDK.h
 *
 *  Created on: Dec 4, 2017
 *      Author: matthias
 */

#ifndef SRC_FDIR_FDISDK_H_
#define SRC_FDIR_FDISDK_H_

#include "ros/ros.h"
#include <queue>
#include "ros_monitoring/MonitoringInfo.h"
#include "ros_monitoring/Error.h"
#include "string"
#include "configinterface.h"
#include "max.h"

struct fdiconfig{
  std::string op;
  float value;
  std::string error;
  float errorlevel;
};

class FdiSDK
{
public:
  FdiSDK(ros::NodeHandle& n);
  virtual ~FdiSDK();

  void load_config(ros::NodeHandle& n);
  void registerFDIObject(ConfigInterface object, std::string msg);
  void fdi();

private:
  void monitorCallback(ros_monitoring::MonitoringInfo mi);
  std::queue<ros_monitoring::KeyValue> msgBuffer;
  ros::Subscriber sub;
  ros::Publisher pub;

  std::map<std::string, std::vector<ConfigInterface>> fdiConfigList;
};

#endif /* SRC_FDIR_FDISDK_H_ */
