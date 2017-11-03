/*
 * cpumonitor.cpp
 *
 *  Created on: Nov 3, 2017
 *      Author: matthias
 */

#include "cpumonitor.h"
#include "ros/ros.h"
#include <stdlib.h>



CpuMonitor::CpuMonitor() {
	// TODO Auto-generated constructor stub

}

CpuMonitor::~CpuMonitor() {
	// TODO Auto-generated destructor stub
}








int main( int argc, char **argv )
{

	ros::init(argc, argv, "cpu_monitor");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
	double loadavg[3];
	while(ros::ok()) {
		getloadavg(loadavg, 3); //works/updates around every 5 seconds
		ROS_INFO(" %f, %f, %f", loadavg[0],loadavg[1],loadavg[2]);
		loop_rate.sleep();
	}

}
