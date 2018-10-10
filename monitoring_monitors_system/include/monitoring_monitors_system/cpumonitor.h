/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018  University of Hannover
 *                     Institute for Systems Engineering - RTS
 *                     Prof. Dr.-Ing. Bernardo Wagner
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name University of Hannover nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef SRC_CPUMONITOR_H_
#define SRC_CPUMONITOR_H_

#include "ros/ros.h"
#include "monitoring_core/monitor.h"


/**
 * @brief The CpuMonitor class provides a monitor for the CPU
 */
class CpuMonitor
{
public:
  CpuMonitor();
  virtual ~CpuMonitor();

  /**
   * @brief getCurrentCpuLoad calculates the overall cpu load of all cores. This is the mean over the time it was last called
   * @return overall cpu load in percent
   */
  float getCurrentCpuLoad();
  /**
   * @brief getLoadAvg gets the linux-specific loadavg with a rather fast updatetime
   * @return the avg value of the system
   */
  float getLoadAvg();
  /**
   * @brief getCPUTemp reads the cpu temp from the sys pseudo-file system of the kernel
   * @return the cpu temperature in °C
   */
  float getCPUTemp();
  /**
   * @brief getCPUCoreLoad calculates the load of one cpu core. This is the mean over the time it was last called
   * @param n specifys the cpu number of the interested cpu core
   * @return cpu core load in percent
   */
  double getCPUCoreLoad(int n);

  int hwmon_index_;   ///< index which temperature sensor is listed as the cpu temperature sensor
  int temp_index_;

private:
  std::vector<unsigned long long> lastTotalUser, lastTotalUserLow, lastTotalSys, lastTotalIdle; ///< vector to save the previous jiffie states of each cpu core and for overall
  void init();      ///< inits the jiffie state vectors aswell as the temp_index

};

#endif /* SRC_CPUMONITOR_H_ */
