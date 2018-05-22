/*
 * guiconcatenation.h
 *
 *  Created on: Jan 5, 2018
 *      Author: matthias
 */

#ifndef SRC_GUICONCATENATION_H_
#define SRC_GUICONCATENATION_H_

#include "ros/ros.h"
#include "monitoring_msgs/MonitoringArray.h"
#include "monitoring_msgs/Error.h"
#include "monitoring_msgs/Gui.h"

class GuiConcatenation {
public:
	GuiConcatenation(ros::NodeHandle& n);
	virtual ~GuiConcatenation();

  monitoring_msgs::Gui getMsg();
private:
  void monitor_cb(monitoring_msgs::MonitoringArray ma);
  void error_cb(monitoring_msgs::Error er);
	ros::Subscriber monitor_sub, error_sub;
  monitoring_msgs::Gui msg;
};

#endif /* SRC_GUICONCATENATION_H_ */
