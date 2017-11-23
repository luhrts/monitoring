/*
 * nodemonitor.h
 *
 *  Created on: Nov 22, 2017
 *      Author: matthias
 */

#ifndef SRC_NODEMONITOR_H_
#define SRC_NODEMONITOR_H_

#include "ros/ros.h"
#include "ros_monitoring/MonitoringInfo.h"
#include <XmlRpcValue.h>



class NodeMonitor {
public:
	NodeMonitor();
	NodeMonitor(std::string filename);
	virtual ~NodeMonitor();
	std::vector<std::string> getNodeList();

private:
	std::vector<std::string> readFile(std::string filename);
	std::vector<std::string> nodesList;
};

#endif /* SRC_NODEMONITOR_H_ */
