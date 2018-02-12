/*
 * Monitor.h
 *
 *  Created on: Feb 5, 2018
 *      Author: matthias
 */

#ifndef SRC_MONITORS_MONITOR_H_
#define SRC_MONITORS_MONITOR_H_

#include "ros_monitoring/MonitoringArray.h"

class MonitorMsg {
public:
	MonitorMsg(std::string monitorName, std::string monitorDescription);
	virtual ~MonitorMsg();

	void addValue(std::string key, std::string value, std::string unit, float errorlevel);
	void addValue(std::string key, float value, std::string unit, float errorlevel);

	void publish();

private:
	ros::Publisher pub;
	ros_monitoring::MonitoringArray ma;
};

#endif /* SRC_MONITORS_MONITOR_H_ */
