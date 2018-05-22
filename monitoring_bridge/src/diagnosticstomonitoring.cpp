#include "ros_monitoring/diagnosticstomonitoring.h"

DiagnosticsToMonitoring::DiagnosticsToMonitoring(ros::NodeHandle &node) {
  monitor_pub = node.advertise<ros_monitoring::MonitoringArray>("/monitoring", 1);
  diag_sub = node.subscribe("/diagnostics", 10, &DiagnosticsToMonitoring::diagnostics_callback, this);
}

DiagnosticsToMonitoring::~DiagnosticsToMonitoring() {

}

void DiagnosticsToMonitoring::diagnostics_callback(diagnostic_msgs::DiagnosticArray msg) {
  for(int i=0; i<msg.status.size(); i++) {
    ros_monitoring::MonitoringInfo mi;
    double errorlevel = msg.status[i].level/2.0;
    if(errorlevel >= 1.0) {
      errorlevel = 1.0;
    }
    mi.name = msg.status[i].name;
    mi.description = msg.status[i].message;
    mi.header.stamp = msg.header.stamp;
    for (int j=0; j<msg.status[i].values.size(); j++) {
      ros_monitoring::KeyValue kv;
      kv.key = msg.status[i].values[j].key;
      kv.value = msg.status[i].values[j].value;
      kv.errorlevel = errorlevel;
      mi.values.push_back(kv);
    }
    ma.info.push_back(mi);
  }

}

void DiagnosticsToMonitoring::publish() {
  ma.header.stamp = ros::Time::now();
  monitor_pub.publish(ma);
  ros_monitoring::MonitoringArray newMa;
  ma = newMa;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "diagnostics_to_monitoring");
	ros::NodeHandle n("~");


	float freq = 1;
	if (!n.getParam("frequency", freq)) {
		ROS_WARN("No frequency supplied. Working with %f Hz.", freq);
	}
	ros::Rate loop_rate(freq);
	DiagnosticsToMonitoring DTM(n);

	while(ros::ok()) {
		DTM.publish();

		ros::spinOnce();
		loop_rate.sleep();
	}


}
