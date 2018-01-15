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
#include "ros_monitoring/MonitoringArray.h"
#include "ros_monitoring/Error.h"
#include "string"
#include "configinterface.h"
#include "std_validator/max.h"
#include "std_validator/min.h"
#include "std_validator/between.h"
#include "std_validator/nodeavailable.h"

/**
 * this struct is deprecated and was used befor the configinterface.h
 */
struct fdiconfig{
  std::string op;
  float value;
  std::string error;
  float errorlevel;
};


/**
 * This SDK allows you to setup a Fault detection and identification system for your Robot.
 * Use the Register function to setup your own or pre-developt validators.
 */
class FdiSDK
{
public:
  FdiSDK(ros::NodeHandle& n);
  virtual ~FdiSDK();

  void load_config(ros::NodeHandle& n);
  void registerFDIObject(ConfigInterface* object, std::string msg);
  void checkForFDI();

private:
  void monitorCallback(ros_monitoring::MonitoringArray ma);
  std::queue<ros_monitoring::KeyValue> msgBuffer;
  ros::Subscriber sub;
  ros::Publisher pub;

  std::map<std::string, std::vector<ConfigInterface *> > fdiConfigList;
};

#endif /* SRC_FDIR_FDISDK_H_ */
