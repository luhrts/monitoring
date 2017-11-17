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
#include "ros_monitoring/MonitoringInfo.h"
#include "help.cpp"
#include "ros_rt_benchmark_lib/benchmark.h"

#define NETWORKPRE "/sys/class/net/"
#define NETWORKSUF "/statistics"
#define NETWORKINTERFACE "enp3s0"

class NetworkMonitor {
public:
	NetworkMonitor(float networkThroughput,  char nwit[]);
	virtual ~NetworkMonitor();

	void getNetworkLoad(float& loadinPrx, float& loadinPtx, float& RXBpS, float& TXBpS);
	void publishPackets(ros::Publisher rxpackets_pub, ros::Publisher txpackets_pub, ros_monitoring::MonitoringInfo& mi);

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
