/*
 * monitoringtodiagnostics.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: matthias
 */

#include "ros_monitoring/monitoringtodiagnostics.h"

MonitoringToDiagnostics::MonitoringToDiagnostics(ros::NodeHandle& n) {
	diag_pub = n.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics_agg", 1);
	monitor_sub = n.subscribe("/monitoring", 50, &MonitoringToDiagnostics::monitor_callback, this);

}

MonitoringToDiagnostics::~MonitoringToDiagnostics() {
	// TODO Auto-generated destructor stub
}

void MonitoringToDiagnostics::publish() {
	diag_pub.publish(da);
	diagnostic_msgs::DiagnosticArray newDA;
	da = newDA;

}
void MonitoringToDiagnostics::monitor_callback(ros_monitoring::MonitoringArray msg) {
	for(int i=0; i<msg.info.size(); i++) {
		diagnostic_msgs::DiagnosticStatus ds;
		float meanError=0;
		ds.name = msg.info[i].name;
		ds.message = msg.info[i].description;

		for(int j=0; j<msg.info[i].values.size(); j++) {
			diagnostic_msgs::KeyValue kv;
			kv.key = msg.info[i].values[j].key;
			kv.value = msg.info[i].values[j].value;
			ds.values.push_back(kv);
			meanError += msg.info[i].values[j].errorlevel;
		}

		ds.level = (int) (meanError/msg.info[i].values.size()  * 2.9);
		da.status.push_back(ds);
	}


}

int main(int argc, char **argv) {

	ros::init(argc, argv, "monitoring_to_diagnostics");
	ros::NodeHandle n("~");


	float freq = 1;
	if (!n.getParam("frequency", freq)) {
		ROS_WARN("No frequency supplied. Working with %f Hz.", freq);
	}
	ros::Rate loop_rate(freq);
	MonitoringToDiagnostics MTD(n);

	while(ros::ok()) {
		MTD.publish();

		ros::spinOnce();
		loop_rate.sleep();
	}


}
