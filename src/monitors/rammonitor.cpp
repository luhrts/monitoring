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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ram_monitor");
  ros::NodeHandle n("~");
  ros::Publisher monitor_pub = n.advertise<ros_monitoring::MonitoringArray>("/monitoring", 1);
  ros::Publisher used_pub, percentage_pub;
  std_msgs::Float32 percentage, used;

  float freq = 1;
  if (!n.getParam("frequency", freq))
  {
    ROS_WARN("No frequency supplied. Working with %f Hz.", freq);
  }

  bool bUsed = false;
  if (n.getParam("used", bUsed))
  {
//		if (bUsed) {
//			used_pub = n.advertise<std_msgs::Float32>("/monitoring/ram/used",
//					1);
//		}
  }

  bool bPercent = false;
  if (n.getParam("percent", bPercent))
  {
//		if (bPercent) {
//			percentage_pub = n.advertise<std_msgs::Float32>(
//					"/monitoring/ram/percentage", 1);
//		}
  }

  ros::Rate loop_rate(freq);

  //benchmark init dings
  ROS_RT_Benchmark benchmark;
  benchmark.init();
  ROS_RT_MeasurementDuration* measurement_meminfo = benchmark.createDurationMeasurement("meminfo");

  while (ros::ok())
  {
    measurement_meminfo->start();
    meminfo();		//geting ram info via sysinfo lib
    measurement_meminfo->stop();

    ros_monitoring::MonitoringArray ma;
	ros_monitoring::MonitoringInfo mi;

    mi.name = ros::this_node::getName();
    mi.description = "A RAM-Monitor";
    fillMachineInfo(mi);

    /*	kb_main_total, kb_main_used, kb_main_free,
     kb_main_shared, kb_main_buffers, kb_main_cached*/
    char value[200];
    if (bUsed)
    {

//			used.data = (float) kb_main_used;
//			used_pub.publish(used);

      ros_monitoring::KeyValue used;
      used.key = "ram used";

      sprintf(value, "%ld", (long)kb_main_used);

      used.value = value;
      used.unit = "kb";
      mi.values.push_back(used);

    }
    if (bPercent)
    {

      float perc = ((float)kb_main_used / (float)kb_main_total) * 100.0;
      /*percentage.data = perc
       percentage_pub.publish(percentage);*/

      ros_monitoring::KeyValue percent;
      percent.key = "ram percentage used";
      sprintf(value, "%f", perc);
      percent.value = value;
      percent.unit = "%";
      mi.values.push_back(percent);

    }
    ma.info.push_back(mi);
    monitor_pub.publish(ma);

    loop_rate.sleep();

  }

  benchmark.logData();
}
