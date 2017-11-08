/*
 * networkmonitor.cpp
 *
 *  Created on: Nov 8, 2017
 *      Author: matthias
 */

#include "networkmonitor.h"



NetworkMonitor::NetworkMonitor() {
	maxNWThroughputPS = 1000000000/8; //TODO read config   			1GB Eth
	lastRXbytes = readRXbytes(NETWORKINTERFACE);
	lastTXbytes = readTXbytes(NETWORKINTERFACE);
	lastRXpackets = readRXpackets(NETWORKINTERFACE);
	lastTXpackets = readTXpackets(NETWORKINTERFACE);
	lastStamp = ros::Time::now();

}

NetworkMonitor::~NetworkMonitor() {
	// TODO Auto-generated destructor stub
}

void NetworkMonitor::publishNetworkLoad(ros::Publisher pub) {


	ros::Time now = ros::Time::now();
	unsigned int rxb = readRXbytes(NETWORKINTERFACE);
	unsigned int txb = readTXbytes(NETWORKINTERFACE);
	unsigned int deltaRXB = rxb - lastRXbytes;
	unsigned int deltaTXB = txb - lastTXbytes;
	ros::Duration deltaT = now - lastStamp;

	float timeT = (float)deltaT.sec + (deltaT.nsec / pow(10, 9));
	float RXBpS = (float) deltaRXB / timeT;
	float TXBpS = (float) deltaTXB / timeT;

	ROS_INFO("RXB pro Sec: %f", RXBpS);
	ROS_INFO("TXB pro Sec: %f", TXBpS);
	ROS_INFO("Networkload: %f", (RXBpS+TXBpS)/maxNWThroughputPS);

	lastRXbytes = rxb;
	lastTXbytes = txb;
	lastStamp = now;


}

unsigned int NetworkMonitor::readRXbytes(char nwinterface[]) {
	unsigned int rx_bytes;
	char path[256];
	sprintf(path,"%s%s%s/rx_bytes",NETWORKPRE,nwinterface,NETWORKSUF);

	FILE* file;
	file = fopen(path, "r");
	fscanf(file, "%d", &rx_bytes);
	fclose(file);
	return rx_bytes;
}

unsigned int NetworkMonitor::readTXbytes(char nwinterface[]) {
	unsigned int tx_bytes;
	char path[256];
	sprintf(path,"%s%s%s/tx_bytes",NETWORKPRE,nwinterface,NETWORKSUF);

	FILE* file;
	file = fopen(path, "r");
	fscanf(file, "%d", &tx_bytes);
	fclose(file);
	return tx_bytes;
}
unsigned int NetworkMonitor::readRXpackets(char nwinterface[]) {
	unsigned int rx_packets;
	char path[256];
	sprintf(path,"%s%s%s/rx_packets",NETWORKPRE,nwinterface,NETWORKSUF);

	FILE* file;
	file = fopen(path, "r");
	fscanf(file, "%d", &rx_packets);
	fclose(file);
	return rx_packets;
}
unsigned int NetworkMonitor::readTXpackets(char nwinterface[]) {
	unsigned int tx_packets;
	char path[256];
	sprintf(path,"%s%s%s/tx_packets",NETWORKPRE,nwinterface,NETWORKSUF);

	FILE* file;
	file = fopen(path, "r");
	fscanf(file, "%d", &tx_packets);
	fclose(file);
	return tx_packets;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "network_monitor");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<std_msgs::Float32>("monitoring/network", 1);

	ros::Rate loop_rate(1);
	NetworkMonitor NWm;

	while (ros::ok()) {

		NWm.publishNetworkLoad(pub);
		loop_rate.sleep();
	}

}
