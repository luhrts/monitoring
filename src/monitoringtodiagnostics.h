/*
 * monitoringtodiagnostics.h
 *
 *  Created on: Jan 12, 2018
 *      Author: matthias
 */

#ifndef SRC_MONITORINGTODIAGNOSTICS_H_
#define SRC_MONITORINGTODIAGNOSTICS_H_

#include "ros/ros.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "ros_monitoring/MonitoringArray.h"

class MonitoringToDiagnostics {
public:
	MonitoringToDiagnostics(ros::NodeHandle& n);
	virtual ~MonitoringToDiagnostics();

	void publish();
private:
	void monitor_callback(ros_monitoring::MonitoringArray msg);
	ros::Publisher diag_pub;
	ros::Subscriber monitor_sub;
	diagnostic_msgs::DiagnosticArray da;
};

#endif /* SRC_MONITORINGTODIAGNOSTICS_H_ */
