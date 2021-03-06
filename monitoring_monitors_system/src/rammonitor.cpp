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
#include <proc/sysinfo.h>

#include "monitoring_core/monitor.h"



int main(int argc, char **argv)
{
  ros::init(argc, argv, "ram_monitor");
  ros::NodeHandle n("~");

  double freq;
  n.param<double>("frequency", freq, 1.0);

  bool bUsed;
  n.param<bool>("used", bUsed, true);

  bool bPercent;
  n.param<bool>("percent", bPercent, true);
  int monitor_mode;
  n.param<int>("monitor_mode", monitor_mode,1);

  double warnPercent;
  n.param<double>("percent_total_warn", warnPercent, 80);

  double errorPercent;
  n.param<double>("percent_total_error", errorPercent, 95);


  AggregationStrategies aggregation;
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

  ros::Rate loop_rate(freq);

  Monitor msg(n, "RAM-Monitor" );
  while (ros::ok())
  {

    meminfo();		//geting ram info via sysinfo lib
    /* OUTPUT: kb_main_total, kb_main_used, kb_main_free,
     kb_main_shared, kb_main_buffers, kb_main_cached*/

    // char value[200];
    if (bUsed)
    {
      msg.addValue("total_used", kb_main_used, "kB", 0, aggregation);
    }
    if (bPercent)
    {
      float perc = ((float)kb_main_used / (float)kb_main_total) * 100.0;

      double error = 0.0;
      if (perc > warnPercent){
        error = 0.5;
      }
      if (perc > errorPercent){
        error = 0.7;
      }
      msg.addValue("percentage_used", perc, "%", error,aggregation);
    }
    ros::spinOnce();

    loop_rate.sleep();

  }

}
