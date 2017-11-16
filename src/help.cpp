/*
 * help.cpp
 *
 *  Created on: Nov 15, 2017
 *      Author: matthias
 */

#include "ros/ros.h"
#include "ros_monitoring/MonitoringArray.h"
#include <unistd.h>

void fillMachineInfo(ros_monitoring::MonitoringInfo& mi) {
	mi.header.stamp = ros::Time::now();
	char name[30];
	size_t len;
	if(gethostname(name, len)) {
		ROS_ERROR("Could not read Hostname!");
	}
	mi.pc.Hostname = name;
	mi.pc.ip = "";
}
