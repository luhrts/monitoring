/*
 * Monitor.cpp
 *
 *  Created on: Feb 5, 2018
 *      Author: matthias
 */

#include "ros_monitoring/monitors/monitormsg.h"

MonitorMsg::MonitorMsg(ros::NodeHandle &n, std::string monitorName, std::string monitorDescription) {
	pub = n.advertise<ros_monitoring::MonitoringArray> ("/monitoring", 1);
	miIndex=-1;
}

MonitorMsg::~MonitorMsg() {
	// TODO Auto-generated destructor stub

}

void MonitorMsg::addNewInfoTree(std::string name, std::string description) {
	ros_monitoring::MonitoringInfo mi;
	mi.name = name;
	mi.description = description;
	ma.info.push_back(mi);
	miIndex++;
}

void MonitorMsg::addValue(std::string key, std::string value, std::string unit, float errorlevel) {
	if(miIndex<0){
		ROS_ERROR("No Info Tree Node Created!!! Use addNewInfoTree before adding Values");
	} else {
		ros_monitoring::KeyValue kv;
		kv.key = key;
		kv.value = value;
		kv.unit = unit;
		kv.errorlevel = errorlevel;
		ma.info[miIndex].values.push_back(kv);
	}
}


void MonitorMsg::addValue(std::string key, float value, std::string unit, float errorlevel){
	char stringvalue[100];
	sprintf(stringvalue, "%f", value);
	addValue(key, stringvalue, unit, errorlevel);
}

void MonitorMsg::publish() {
	ma.header.stamp = ros::Time::now();
	pub.publish(ma);

	resetMsg();
}


void MonitorMsg::resetMsg(){
	ros_monitoring::MonitoringArray newMA;
	ma = newMA;
	miIndex = -1;

}
