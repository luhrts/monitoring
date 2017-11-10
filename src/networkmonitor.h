/*
 * networkmonitor.h
 *
 *  Created on: Nov 8, 2017
 *      Author: matthias
 */

#ifndef SRC_NETWORKMONITOR_H_
#define SRC_NETWORKMONITOR_H_

#include "ros/ros.h"
#include <iostream>
#include <string>
#include "std_msgs/Float32.h"

#define NETWORKPRE "/sys/class/net/"
#define NETWORKSUF "/statistics"
#define NETWORKINTERFACE "enp3s0"

class NetworkMonitor {
public:
	NetworkMonitor(float networkThroughput,  char nwit[]);
	virtual ~NetworkMonitor();

	void publishNetworkLoad(ros::Publisher load_pub, ros::Publisher rxbytes_pub, ros::Publisher txbytes_pub, bool bytes, bool load);
	void publishPackets(ros::Publisher rxpackets_pub, ros::Publisher txpackets_pub);

private:
	float maxNWThroughputPS;
	unsigned int readRXbytes(char nwinterface[]);
	unsigned int readTXbytes(char nwinterface[]);
	unsigned int readRXpackets(char nwinterface[]);
	unsigned int readTXpackets(char nwinterface[]);
	ros::Time lastStampBytes, lastStampPackets;
	unsigned int lastRXbytes,lastTXbytes, lastRXpackets,lastTXpackets;
	char *networkinterface;

};

#endif /* SRC_NETWORKMONITOR_H_ */
