

/*
 * max.h
 *
 * Created on: Dec 6, 2017
 *    Author:matthias
 */

#ifndef SRC_MAX_H_
#define SRC_MAX_H_


#include "ros/ros.h"
#include "ros_monitoring/MonitoringInfo.h"
#include "configinterface.h"

class Max : Public ConfigInterface
{
 public:
  Max(float maxValue, float errorLevel, ros::Publisher pub);
  virtual ~Max();

  void check(ros_monitoring::KeyValue newMsg);

 private:
  float maxValue, errorlevel;

}

#endif /* SRC_MAX_H_ */
