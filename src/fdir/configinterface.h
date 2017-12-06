/*
 * configinterface.h
 *
 *  Created on: Dec 6, 2017
 *      Author: matthias
 */

#ifndef SRC_FDIR_CONFIGINTERFACE_H_
#define SRC_FDIR_CONFIGINTERFACE_H_

#include "ros/ros.h"
#include "ros_monitoring/KeyValue.h"
#include "ros_monitoring/Error.h"
#include "../monitors/help.cpp"

class ConfigInterface
{
public:
  ConfigInterface(ros::Publisher& publisher);
  virtual ~ConfigInterface();

  virtual void check(ros_monitoring::KeyValue newMsg);

private:
  ros::Publisher pub;

  void publishError(ros_monitoring::Error errormsg);
};

#endif /* SRC_FDIR_CONFIGINTERFACE_H_ */
