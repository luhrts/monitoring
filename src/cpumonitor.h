/*
 * cpumonitor.h
 *
 *  Created on: Nov 3, 2017
 *      Author: matthias
 */

#ifndef SRC_CPUMONITOR_H_
#define SRC_CPUMONITOR_H_

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"

class CpuMonitor {
public:
	CpuMonitor();
	virtual ~CpuMonitor();


	void publishCpuUsage(ros::Publisher pub);
	void publishLoadAvg(ros::Publisher pub);

private:
	unsigned long long lastTotalUser, lastTotalUserLow, lastTotalSys, lastTotalIdle;
	void init();
	double getCurrentValue();
};

#endif /* SRC_CPUMONITOR_H_ */
