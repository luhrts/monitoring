/*
 * max.cpp
 *
 *  Created on: Dec 6, 2017
 *      Author: matthias
 */

#include "max.h"

Max::Max(float value, std::string errormsg; float errorLevel, ros::Publisher& publisher)
{
  maxValue = value;
  msg= errormsg;
  pub = publisher;
  errorlevel = errorLevel;
}

Max::~Max() {}


void Max::check(ros_monitoring::KeyValue newMsg) {
  if(maxValue< newMsg.value) {
    ROS_ERROR("ERROR: %s is higher then expected, Errorlevel to %f", newMsg.key, errorlevel);
    ros_monitoring::Error errormsg;
    errormsg.key=msg;
    errormsg.value = newMsg.value;
    errormsg.level = errorlevel;
    publishError(errormsg);
  }

}
