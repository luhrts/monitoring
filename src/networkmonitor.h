/*
 * networkmonitor.h
 *
 *  Created on: Nov 8, 2017
 *      Author: matthias
 */

#ifndef SRC_NETWORKMONITOR_H_
#define SRC_NETWORKMONITOR_H_

#include "ros/ros.h"

#include "std_msgs/Float32.h"

#define NETWORKPRE "/sys/class/net/"
#define NETWORKSUF "/statistics"
#define NETWORKINTERFACE "enp3s0"

class NetworkMonitor {
public:
	NetworkMonitor();
	virtual ~NetworkMonitor();

	void publishNetworkLoad(ros::Publisher pub);

private:
	float maxNWThroughputPS;
	unsigned int readRXbytes(char nwinterface[]);
	unsigned int readTXbytes(char nwinterface[]);
	unsigned int readRXpackets(char nwinterface[]);
	unsigned int readTXpackets(char nwinterface[]);
	ros::Time lastStamp;
	unsigned int lastRXbytes,lastTXbytes, lastRXpackets,lastTXpackets;

};

#endif /* SRC_NETWORKMONITOR_H_ */
