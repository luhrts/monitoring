/*
 * Monitor.cpp
 *
 *  Created on: Feb 5, 2018
 *      Author: matthias
 */

#include "monitoring_core/monitor.h"
#include <unistd.h>

Monitor::Monitor(ros::NodeHandle &n, std::string monitorDescription, bool autoPublishing) {
  monitor_description_ = monitorDescription;
  pub = n.advertise<monitoring_msgs::MonitoringArray> ("/monitoring", 1);
  monitoring_msgs::MonitoringInfo mi;

  char hostname[HOST_NAME_MAX];
  int result;
  result = gethostname(hostname, HOST_NAME_MAX);
  if (result)
      {
        perror("gethostname");
        return;
      }
  host_name_ =  hostname;
  node_name_ = ros::this_node::getName();

  mi.name = host_name_+node_name_;
  mi.description = monitorDescription;
  ma.info.push_back(mi);
  miIndex = 0;

  if(autoPublishing) {
    int frequency = 1;
    ros::NodeHandle private_n("~");
    if (!private_n.getParam("monitoring/frequency", frequency))
    {
      ROS_WARN("No frequency supplied for monitoring (%s/monitoring/frequency. Working with %d Hz.", node_name_.c_str(), frequency);
    }

    timer = n.createTimer(ros::Duration(1/frequency), &Monitor::timerCallback, this);
  }

}

Monitor::~Monitor() {
	// TODO Auto-generated destructor stub

}

void Monitor::timerCallback(const ros::TimerEvent& te) {
  publish();
}


void Monitor::addValue(std::string key, std::string value, std::string unit, float errorlevel) {

  monitoring_msgs::KeyValue kv;
  kv.key = key;
  kv.value = value;
  kv.unit = unit;
  kv.errorlevel = errorlevel;
  ma.info[miIndex].values.push_back(kv);
}

void Monitor::addValue(std::string key, float value, std::string unit, float errorlevel){
  char stringvalue[100];    //TODO check if 100 is a good number!
	sprintf(stringvalue, "%f", value);
	addValue(key, stringvalue, unit, errorlevel);
}

void Monitor::publish() {
	ma.header.stamp = ros::Time::now();
	pub.publish(ma);

	resetMsg();
}

void Monitor::resetMsg(){
  monitoring_msgs::MonitoringArray newMA;
	ma = newMA;
  monitoring_msgs::MonitoringInfo mi;
  mi.name = host_name_+node_name_;
  mi.description = monitor_description_;
  ma.info.push_back(mi);
  miIndex = 0;

}
