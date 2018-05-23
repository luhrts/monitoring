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
 * This SDK allows you to setup a Fault detection and identification system for your Robot.
 * Use the Register function to setup your own or pre-developt validators.
 */
class FdiSDK
{
public:
  FdiSDK(ros::NodeHandle& n);
  virtual ~FdiSDK();

  /**
   * @brief registerFDIObject is used to register validators to check for errors in the system
   * @param object is the validator
   * @param msg is the name of the message it wants to listen to
   */
  void registerFDIObject(ConfigInterface* object, std::string msg);

private:
  /**
   * @brief monitorCallback this callback will automaticly buffer the messages. Which will be handled in the checkforFDI function.
   * @param ma
   */
  void monitorCallback(monitoring_msgs::MonitoringArray ma);
  ros::Subscriber sub;  ///< subscribes to monitoring for all monitored data
  ros::Publisher pub;   ///< publisher for error msgs

  std::map<std::string, std::vector<ConfigInterface *> > fdiConfigList; ///< a map to assign incoming msgs to the registered validators
};

#endif /* SRC_FDIR_FDISDK_H_ */
