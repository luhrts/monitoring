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

#include "ros/ros.h"

#include "monitoring_core/monitor.h"


int readScalingMaxCPUFrequency(int cpuNumb) {
  char path[80];
  FILE* file;
  sprintf(path, "/sys/devices/system/cpu/cpu%d/cpufreq/scaling_max_freq", cpuNumb);
  //  ROS_INFO("checking: %s", path);
  file = fopen(path, "r");
  int input = 0;
  fscanf(file, "%d", &input);
  fclose(file);
  return input;
}

int readMaxCPUFrequency(int cpuNumb) {
  char path[80];
  FILE* file;
  sprintf(path, "/sys/devices/system/cpu/cpu%d/cpufreq/cpuinfo_max_freq", cpuNumb);
  //  ROS_INFO("checking: %s", path);
  file = fopen(path, "r");
  int input = 0;
  fscanf(file, "%d", &input);
  fclose(file);
  return input;
}

int readCurCPUFrequency(int cpuNumb) {
  char path[80];
  FILE* file;
  sprintf(path, "/sys/devices/system/cpu/cpu%d/cpufreq/scaling_cur_freq", cpuNumb);
  //  ROS_INFO("checking: %s", path);
  file = fopen(path, "r");
  int input = 0;
  fscanf(file, "%d", &input);
  fclose(file);
  return input;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cpu_frequency_monitor");
  ros::NodeHandle n("~");

  int numCPU = sysconf(_SC_NPROCESSORS_ONLN);
  std::vector<int> maxFrequencys;
  for(int i=0; i<numCPU; i++) {
    maxFrequencys.push_back(readMaxCPUFrequency(i));
  }

  double freq = 1;
  int monitor_mode;
  AggregationStrategies aggregation;

  if (!n.getParam("monitoring/frequency", freq))
  {
    ROS_WARN("No frequency supplied. Working with %f Hz.", freq);
  }
  if (n.getParam("monitor_mode", monitor_mode))
  {
    switch (monitor_mode){
    case 1 :
      aggregation = AggregationStrategies::LAST;
      break;
    case 2 :
      aggregation = AggregationStrategies::FIRST;
      break;
    case 3 :
      aggregation = AggregationStrategies::MIN;
      break;
    case 4 :
      aggregation = AggregationStrategies::MAX;
      break;
    case 5 :
      aggregation = AggregationStrategies::AVG;
      break;

    }
  }


  ros::Rate loop_rate(freq);

  Monitor msg(n, "CPU-Frequency-Monitor" );

  while (ros::ok())
  {
    for(int i=0; i<numCPU; i++) {
      double freq = readCurCPUFrequency(i);
      float max_freq = readScalingMaxCPUFrequency(i);

      msg.addValue("cpu" + std::to_string(i) + "/frequency", freq/1000/1000, "GHz", 0.0);

      if(fabs(maxFrequencys[i]-max_freq) > 1.0) { // evaluate to error since max_freq is not a

        msg.addValue("cpu" + std::to_string(i) + "/max_freq",  max_freq/1000/1000, "GHz", 0.4,aggregation);
      } else {
        msg.addValue("cpu" + std::to_string(i) + "/max_freq",  max_freq/1000/1000, "GHz", 0.0,aggregation);
      }
    }

    ros::spinOnce();

    loop_rate.sleep();

  }

}
