/*
 * Monitor.cpp
 *
 *  Created on: Feb 5, 2018
 *      Author: matthias
 */

#include "monitoring_core/monitor.h"
#include <unistd.h>


Monitor::Monitor(){
  init("default");
}

Monitor::Monitor(ros::NodeHandle &n, std::string monitorDescription, bool autoPublishing) : miIndex(0)
{
  init(monitorDescription);
  initROS(n, autoPublishing);
}

Monitor::Monitor(std::string monitorDescription, bool autoPublishing) : miIndex(0)
{
  ros::NodeHandle n;
  init(monitorDescription);
  initROS(n, autoPublishing);
}

Monitor::~Monitor()
{
	// TODO Auto-generated destructor stub

}

void Monitor::init(std::string monitorDescription)
{
  monitor_description_ = monitorDescription;
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
}

void Monitor::initROS(ros::NodeHandle &n, bool autoPublishing)
{
  pub = n.advertise<monitoring_msgs::MonitoringArray> ("/monitoring", 1);

  if(autoPublishing)
  {
    int frequency = 1;
    ros::NodeHandle private_n("~");
    if (!private_n.getParam("monitoring/frequency", frequency))
    {
      ROS_WARN("No frequency supplied for monitoring (%s/monitoring/frequency. Working with %d Hz.", node_name_.c_str(), frequency);
    }

    timer = n.createTimer(ros::Duration(1/frequency), &Monitor::timerCallback, this);
  }
}


void Monitor::timerCallback(const ros::TimerEvent& te) {
  publish();
}


void Monitor::addValue(std::string key, std::string value, std::string unit, float errorlevel, AggregationStrategies aggregation)
{

  //check if key contains whitespace
  if (key.find_first_of("\t\n ") != std::string::npos){
    ROS_WARN("key: %s, contains a illegal char (whitespace, tab, new_line)", key.c_str());
  }

  //check if the value is already beeing monitored
  bool found = false;
  for (int i = 0; i < ma.info[miIndex].values.size(); ++i)
  {
    if (ma.info[miIndex].values[i].key == key)
    {
      if (aggregation == AggregationStrategies::LAST) // Always update
      {
        ma.info[miIndex].values[i].value = value;
        ma.info[miIndex].values[i].unit = unit;
        ma.info[miIndex].values[i].errorlevel = errorlevel;
      } else if (aggregation == AggregationStrategies::MIN) { // Update if the value is smaller
        if (ma.info[miIndex].values[i].value > value){
          ma.info[miIndex].values[i].value = value;
          ma.info[miIndex].values[i].unit = unit;
          ma.info[miIndex].values[i].errorlevel = errorlevel;
        }
      } else if (aggregation == AggregationStrategies::MAX) { // Update if the value is larger
        if (ma.info[miIndex].values[i].value < value){
          ma.info[miIndex].values[i].value = value;
          ma.info[miIndex].values[i].unit = unit;
          ma.info[miIndex].values[i].errorlevel = errorlevel;
        }
      }

      found = true;
      break;
    }
  }

  // if the key is new, add it to the list
  if (!found){
    monitoring_msgs::KeyValue kv;
    kv.key = key;
    kv.value = value;
    kv.unit = unit;
    kv.errorlevel = errorlevel;

    ma.info[miIndex].values.push_back(kv);
  }
}


void Monitor::addValue(std::string key, float value, std::string unit, float errorlevel, AggregationStrategies aggregation)
{
  char stringvalue[1000];    //TODO check if 1000 is a good number!

    sprintf(stringvalue, "%f", value);

  addValue(key, stringvalue, unit, errorlevel, aggregation);
}

void Monitor::publish()
{
	ma.header.stamp = ros::Time::now();
	pub.publish(ma);

	resetMsg();
}

void Monitor::resetMsg()
{
  monitoring_msgs::MonitoringArray newMA;
	ma = newMA;
  monitoring_msgs::MonitoringInfo mi;
  mi.name = host_name_+node_name_;
  mi.description = monitor_description_;
  ma.info.push_back(mi);
  miIndex = 0;

}
