/*
 * configinterface.cpp
 *
 *  Created on: Dec 6, 2017
 *      Author: matthias
 */

#include "configinterface.h"


#include "ros/ros.h"
#include "ros_monitoring/MonitoringArray.h"
#include <unistd.h>


ConfigInterface::ConfigInterface(ros::Publisher& publisher) {
  pub = publisher;
}

ConfigInterface::~ConfigInterface() {}


/**
 * standartizied publish call to ease the use
 */
void ConfigInterface::publishError(ros_monitoring::Error errormsg) {
  char hostname[30];
  size_t len;

  if (gethostname(hostname, len))
  {
    ROS_ERROR("Could not read Hostname!");
  }
  errormsg.pc.Hostname = hostname;
  pub.publish(errormsg);
}

