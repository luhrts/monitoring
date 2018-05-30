/*
 * Monitor.h
 *
 *  Created on: Feb 5, 2018
 *      Author: matthias
 */

#ifndef SRC_MONITORS_MONITOR_H_
#define SRC_MONITORS_MONITOR_H_
#include "gtest/gtest_prod.h"  
#include "monitoring_msgs/MonitoringArray.h"
#include "ros/ros.h"

class Monitor {
public:
  Monitor(ros::NodeHandle &n, std::string monitorDescription, bool autoPublishing = true);
  virtual ~Monitor();

  void addValue(std::string key, std::string value, std::string unit, float errorlevel);
  void addValue(std::string key, float value, std::string unit, float errorlevel);
	void publish();

	void resetMsg();

private:
	ros::Publisher pub;
  ros::Timer timer;   //takes the publishing frequency
  void timerCallback(const ros::TimerEvent& te);

  monitoring_msgs::MonitoringArray ma;
  std::string node_name_;
  std::string monitor_description_;
  int miIndex = 0;
<<<<<<< HEAD
///////////////////Gtest/////////////////////
FRIEND_TEST(MonitoringCore, addValueString);
FRIEND_TEST(MonitoringCore, addValueFloatRand);
FRIEND_TEST(MonitoringCore, addValueFloat);
FRIEND_TEST(MonitoringCore, addValueErrorLevel);
=======
>>>>>>> ec3b999014c7db378724d36e63a8072d098f88ba

};

#endif /* SRC_MONITORS_MONITOR_H_ */
