/*
 * rammonitor.cpp
 *
 *  Created on: Nov 3, 2017
 *      Author: matthias
 */
#include "ros/ros.h"
#include <proc/sysinfo.h>
#include "rammonitor.h"
#include "std_msgs/Float32.h"
#include "ros_monitoring/MonitoringInfo.h"
#include "string.h"


#include "help.cpp"

RAMMonitor::RAMMonitor() {
	// TODO Auto-generated constructor stub

}

RAMMonitor::~RAMMonitor() {
	// TODO Auto-generated destructor stub
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "ram_monitor");
	ros::NodeHandle n("~");
	ros::Publisher monitor_pub = n.advertise<ros_monitoring::MonitoringInfo>("/monitoring/all", 1);
	ros::Publisher used_pub;
	ros::Publisher percentage_pub;
	std_msgs::Float32 percentage;
	std_msgs::Float32 used;

	float freq = 1;
	if (!n.getParam("frequency", freq)) {
		ROS_WARN("No frequency supplied. Working with %d Hz.", freq);
	}

	bool bUsed = false;
	if (n.getParam("used", bUsed)) {
		if (bUsed) {
			used_pub = n.advertise<std_msgs::Float32>("/monitoring/ram/used",
					1);
		}
	}

	bool bPercent = false;
	if (n.getParam("percent", bPercent)) {
		if (bPercent) {
			percentage_pub = n.advertise<std_msgs::Float32>(
					"/monitoring/ram/percentage", 1);
		}
	}

	ros::Rate loop_rate(freq);

	while (ros::ok()) {
		meminfo();
		ros_monitoring::MonitoringInfo mi;
		mi.name = ros::this_node::getName();
		mi.description = "A RAM-Monitor";
		fillMachineInfo(mi);

		/*	kb_main_total, kb_main_used, kb_main_free,
		 kb_main_shared, kb_main_buffers, kb_main_cached*/

		if (bUsed) {
			used.data = (float) kb_main_used;
			used_pub.publish(used);
		}
		if (bPercent) {
			percentage.data = ((float) kb_main_used / (float) kb_main_total)
					* 100.0;
			percentage_pub.publish(percentage);
		}
		ros_monitoring::KeyValue used;
		used.key = "used";
		char value[200];
		sprintf(value, "%d", (long) kb_main_used);

		used.value = value;
		mi.values.push_back(used);

		ros_monitoring::KeyValue percent;
		percent.key = "percent";
		sprintf(value, "%f", percentage.data);
		percent.value = value;
		if(percentage.data > 70) {
			percent.error.level = 0.7;
		} else {
			percent.error.level = 0;
		}
		mi.values.push_back(percent);

		monitor_pub.publish(mi);

		loop_rate.sleep();

	}

}
