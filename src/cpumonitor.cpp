/*
 * cpumonitor.cpp
 *
 *  Created on: Nov 3, 2017
 *      Author: matthias
 */

#include "cpumonitor.h"
#include "ros/ros.h"
#include <stdlib.h>
#include "std_msgs/Float32.h"



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
	ros::Publisher avg_pub = n.advertise<std_msgs::Float32>("monitoring/cpu/avg", 1);
	ros::Rate loop_rate(1);
	double loadavg[3];
	std_msgs::Float32 avg;
	while(ros::ok()) {
		getloadavg(loadavg, 3); //works/updates around every 5 seconds
		//ROS_INFO(" %f, %f, %f", loadavg[0],loadavg[1],loadavg[2]);

		avg.data = loadavg[0];
		avg_pub.publish(avg);
		loop_rate.sleep();
	}

}
