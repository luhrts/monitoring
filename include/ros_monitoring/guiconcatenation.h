/*
 * guiconcatenation.h
 *
 *  Created on: Jan 5, 2018
 *      Author: matthias
 */

#ifndef SRC_GUICONCATENATION_H_
#define SRC_GUICONCATENATION_H_

#include "ros/ros.h"
#include "ros_monitoring/MonitoringArray.h"
#include "ros_monitoring/Error.h"
#include "ros_monitoring/Gui.h"

class GuiConcatenation {
public:
	GuiConcatenation(ros::NodeHandle& n);
	virtual ~GuiConcatenation();

	ros_monitoring::Gui getMsg();
private:
	void monitor_cb(ros_monitoring::MonitoringArray ma);
	void error_cb(ros_monitoring::Error er);
	ros::Subscriber monitor_sub, error_sub;
	ros_monitoring::Gui msg;
};

#endif /* SRC_GUICONCATENATION_H_ */
