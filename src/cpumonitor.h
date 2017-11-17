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
#include "std_msgs/Float32MultiArray.h"
#include "help.cpp"
#include "ros_monitoring/MonitoringInfo.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"

#include "ros_monitoring/Processes.h"

#include "ros_rt_benchmark_lib/benchmark.h"

class CpuMonitor {
public:
	CpuMonitor();
	virtual ~CpuMonitor();


	void publishCpuUsage(ros::Publisher pub, ros_monitoring::MonitoringInfo& mi);
	void publishLoadAvg(ros::Publisher pub, ros_monitoring::MonitoringInfo& mi);
	void publishProcessCpuUsage(ros::Publisher pub, ros_monitoring::MonitoringInfo& mi);
	void publishCPUTemp(ros::Publisher pub, ros_monitoring::MonitoringInfo& mi);

	double getCPUCoreLoad(int n);

private:
	std::vector<unsigned long long> lastTotalUser, lastTotalUserLow, lastTotalSys, lastTotalIdle;
	void init();
	double getCurrentValue();

};

#endif /* SRC_CPUMONITOR_H_ */
