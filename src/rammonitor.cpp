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



RAMMonitor::RAMMonitor() {
	// TODO Auto-generated constructor stub

}

RAMMonitor::~RAMMonitor() {
	// TODO Auto-generated destructor stub
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "ram_monitor");
	ros::NodeHandle n;
	ros::Publisher used_pub = n.advertise<std_msgs::Float32>("monitoring/ram/used", 1000);
	ros::Publisher percentage_pub = n.advertise<std_msgs::Float32>("monitoring/ram/percentage", 1000);
	ros::Rate loop_rate(10);


	std_msgs::Float32 percentage;

	std_msgs::Float32 used;

	while (ros::ok()) {
		meminfo();

		/*	kb_main_total, kb_main_used, kb_main_free,
			kb_main_shared, kb_main_buffers, kb_main_cached*/

		used.data=(float) kb_main_used;
		used_pub.publish(used);

		percentage.data=(float) kb_main_used/(float) kb_main_total;
		percentage_pub.publish(percentage);

		loop_rate.sleep();

	}

}
