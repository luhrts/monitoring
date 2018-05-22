/*
 * cpumonitor.h
 *
 *  Created on: Nov 3, 2017
 *      Author: matthias
 */

#ifndef SRC_CPUMONITOR_H_
#define SRC_CPUMONITOR_H_

#include "ros/ros.h"
#include "monitoring_core/monitor.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include <vector>

class CpuMonitor
{
public:
  CpuMonitor();
  virtual ~CpuMonitor();

  float getCurrentCpuLoad();
  float getLoadAvg();
//  void publishProcessCpuUsage(ros::Publisher pub, ros_monitoring::MonitoringInfo& mi);
  float getCPUTemp();
  double getCPUCoreLoad(int n);

private:
  std::vector<unsigned long long> lastTotalUser, lastTotalUserLow, lastTotalSys, lastTotalIdle;
  int temp_index;
  void init();

};

#endif /* SRC_CPUMONITOR_H_ */
