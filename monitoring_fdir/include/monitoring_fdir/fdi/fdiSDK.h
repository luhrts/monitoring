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
#include "monitoring_msgs/MonitoringArray.h"
#include "monitoring_msgs/Error.h"
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
  void monitorCallback(monitoring_msgs::MonitoringArray ma);
  std::queue<monitoring_msgs::KeyValue> msgBuffer;
  ros::Subscriber sub;
  ros::Publisher pub;

  std::map<std::string, std::vector<ConfigInterface *> > fdiConfigList;
};

#endif /* SRC_FDIR_FDISDK_H_ */
