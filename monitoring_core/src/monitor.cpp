/*
 * Monitor.cpp
 *
 *  Created on: Feb 5, 2018
 *      Author: matthias
 */

#include "monitoring_core/monitor.h"

Monitor::Monitor(ros::NodeHandle &n, std::string monitorDescription) {
  monitor_description_ = monitorDescription;
  pub = n.advertise<monitoring_msgs::MonitoringArray> ("/monitoring", 1);
  monitoring_msgs::MonitoringInfo mi;
  mi.name = ros::this_node::getName(),
  mi.description = monitorDescription;
  ma.info.push_back(mi);
}

Monitor::~Monitor() {
	// TODO Auto-generated destructor stub

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
  mi.name = ros::this_node::getName(),
  mi.description = monitor_description_;
  ma.info.push_back(mi);
  miIndex = 0;

}
