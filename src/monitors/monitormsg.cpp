/*
 * Monitor.cpp
 *
 *  Created on: Feb 5, 2018
 *      Author: matthias
 */

#include "ros_monitoring/monitors/monitormsg.h"

MonitorMsg::MonitorMsg(std::string monitorName, std::string monitorDescription) {

	ros::init(argc, argv, monitorName);
	ros::NodeHandle n("~");

	pub = n.advertise<ros_monitoring::MonitoringArray> ("/monitoring", 1);

	ros_monitoring::MonitoringInfo mi;
	mi.name = monitorName;
	mi.description = monitorDescription;
	ma.push_back(mi);



}

MonitorMsg::~MonitorMsg() {
	// TODO Auto-generated destructor stub

}



void MonitorMsg::addValue(std::string key, std::string value, std::string unit, float errorlevel) {
	ros_monitoring::KeyValue kv;
	kv.key = key;
	kv.value = value;
	kv.unit = unit;
	kv.errorlevel = errorlevel;
	ma.infos[0].values.push_back(kv);
}


void MonitorMsg::addValue(std::string key, float value, std::string unit, float errorlevel){
	std::string stringvalue;
	sprintf(stringvalue, "%f", value);
	addValue(key, stringvalue, unit, errorlevel);
}

void MonitorMsg::publish() {
	pub.publish(ma);

	resetMsg();
}


void resetMsg(){
	ros_monitoring::MonitoringInfo mi;
	ros_monitoring::MonitoringArray newMA;
	mi.name = ma.info[0].name;
	mi.description = ma.info[0].description;
	ma = newMA;
	ma.push_back(mi);
}
