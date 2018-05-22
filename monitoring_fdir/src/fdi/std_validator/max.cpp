/*
 * max.cpp
 *
 *  Created on: Dec 6, 2017
 *      Author: matthias
 */

#include "monitoring_fdir/fdi/std_validator/max.h"

Max::Max(float value, std::string errormsg, float errorLevel, ros::Publisher& publisher)
  :maxValue(value)
  ,msg(errormsg)
  ,errorlevel(errorLevel)
  ,ConfigInterface(publisher)
{
}

Max::~Max() {}

/**
 * Checks if the keyvalue pair is over the configured value.
 */
void Max::check(monitoring_msgs::KeyValue newMsg) {
//  ROS_INFO("Checking CPU TEMP %s", newMsg.value.c_str());

  std::string::size_type sz;
  float value = std::stof (newMsg.value,&sz);
  if(maxValue< value) {
    ROS_WARN("ERROR: Value: %f is higher then expected (%f), Errorlevel to %f", value, maxValue, errorlevel);
    monitoring_msgs::Error errormsg;
    errormsg.header.stamp = ros::Time::now();
    errormsg.key = msg;
    errormsg.value = newMsg.value;
    errormsg.unit = newMsg.unit;
    errormsg.errorlevel = errorlevel;
    publishError(errormsg);
  }

}
