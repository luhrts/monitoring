/*
 * youbotbatteriemonitor.cpp
 *
 *  Created on: Jan 2, 2018
 *      Author: matthias
 */

#include "youbotbatteriemonitor.h"

YoubotBatterieMonitor::YoubotBatterieMonitor() {
	// TODO Auto-generated constructor stub

}

YoubotBatterieMonitor::~YoubotBatterieMonitor() {
	// TODO Auto-generated destructor stub
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "youbot_batterie_monitor");
	ros::NodeHandle n("~");
	ros::Publisher monitor_pub = n.advertise < ros_monitoring::MonitoringArray
			> ("/monitoring/all", 1);

	float freq = 1;
	if (!n.getParam("frequency", freq)) {
		ROS_WARN("No frequency supplied. Working with %f Hz.", freq);
	}



}
