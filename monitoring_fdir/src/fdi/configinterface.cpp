/*
 * configinterface.cpp
 *
 *  Created on: Dec 6, 2017
 *      Author: matthias
 */

#include "monitoring_fdir/fdi/configinterface.h"

#include "ros/ros.h"
#include "monitoring_msgs/MonitoringArray.h"
#include <unistd.h>

ConfigInterface::ConfigInterface(ros::Publisher& publisher) {
	pub = publisher;

	size_t len;
	if (gethostname(hostname, len)) {
		ROS_ERROR("Could not read Hostname!");
	}
}

ConfigInterface::~ConfigInterface() {
}

/**
 * standartizied publish call to ease the use
 */
void ConfigInterface::publishError(monitoring_msgs::Error errormsg) {

	errormsg.pc.Hostname = hostname;
	pub.publish(errormsg);
}

