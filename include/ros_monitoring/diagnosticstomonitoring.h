#ifndef SRC_DIAGNOSTICSTOMONITORING_H_
#define SRC_DIAGNOSTICSTOMONITORING_H_

#include "ros/ros.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "ros_monitoring/MonitoringArray.h"

class DiagnosticsToMonitoring {
public:
	DiagnosticsToMonitoring(ros::NodeHandle& n);
	virtual ~DiagnosticsToMonitoring();

	void publish();
private:
	void diagnostics_callback(diagnostic_msgs::DiagnosticArray msg);
	ros::Publisher monitor_pub;
	ros::Subscriber diag_sub;
	ros_monitoring::MonitoringArray ma;
};

#endif /* SRC_MONITORINGTODIAGNOSTICS_H_ */
