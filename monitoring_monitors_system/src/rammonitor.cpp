/*
 * rammonitor.cpp
 *
 *  Created on: Nov 3, 2017
 *      Author: matthias
 */
#include "ros/ros.h"
#include <proc/sysinfo.h>
#include "std_msgs/Float32.h"
#include "ros_monitoring/MonitoringArray.h"
#include "string.h"

#include "ros_rt_benchmark_lib/benchmark.h"

#include "ros_monitoring/help.h"
#include "ros_monitoring/monitors/monitormsg.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ram_monitor");
  ros::NodeHandle n("~");
  ros::Publisher monitor_pub = n.advertise<ros_monitoring::MonitoringArray>("/monitoring", 1);


  float freq = 1;
  if (!n.getParam("frequency", freq))
  {
    ROS_WARN("No frequency supplied. Working with %f Hz.", freq);
  }

  bool bUsed = false;
  if (n.getParam("used", bUsed))
  {
  }

  bool bPercent = false;
  if (n.getParam("percent", bPercent))
  {
  }

  ros::Rate loop_rate(freq);

  MonitorMsg msg(n, ros::this_node::getName(), "RAM-Monitor" );
  while (ros::ok())
  {
    msg.resetMsg();

    meminfo();		//geting ram info via sysinfo lib
    msg.addNewInfoTree(ros::this_node::getName(), "RAM-Monitor");

    /*	kb_main_total, kb_main_used, kb_main_free,
     kb_main_shared, kb_main_buffers, kb_main_cached*/
    char value[200];
    if (bUsed)
    {
      msg.addValue("RAM used", kb_main_used, "kb", 0);
    }
    if (bPercent)
    {
      float perc = ((float)kb_main_used / (float)kb_main_total) * 100.0;
      msg.addValue("RAM % used", perc, "%", 0);
    }
    msg.publish();

    loop_rate.sleep();

  }

}
