/*
 * cpumonitor.cpp
 *
 *  Created on: Nov 3, 2017
 *      Author: matthias
 */

#include "cpumonitor.h"
#include "ros/ros.h"

CpuMonitor::CpuMonitor() {
	// TODO Auto-generated constructor stub

}

CpuMonitor::~CpuMonitor() {
	// TODO Auto-generated destructor stub
}



int main( int argc, char **argv )
{

	ros::init(argc, argv, "cpu monitor");
	ros::NodeHandle n;


	while(ros::ok()) {

	}

}
