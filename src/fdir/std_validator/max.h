

/*
 * max.h
 *
 * Created on: Dec 6, 2017
 *    Author:matthias
 */

#ifndef SRC_FDIR_MAX_H_
#define SRC_FDIR_MAX_H_


#include "ros/ros.h"
#include "ros_monitoring/MonitoringInfo.h"
#include "../configinterface.h"

class Max : public ConfigInterface
{
 public:
  Max(float maxValue, std::string errormsg, float errorLevel, ros::Publisher& publisher);
  ~Max();

  void check(ros_monitoring::KeyValue newMsg);

 private:
  float maxValue, errorlevel;
  std::string msg;

};

#endif /* SRC_FDIR_MAX_H_ */
