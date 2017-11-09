/*
 * networkmonitor.cpp
 *
 *  Created on: Nov 8, 2017
 *      Author: matthias
 */

#include "networkmonitor.h"

NetworkMonitor::NetworkMonitor(int networkThroughput) {
	maxNWThroughputPS = (float) (networkThroughput * pow(10, 6)) / 8; //TODO read config   			1GB Eth
	lastRXbytes = readRXbytes(NETWORKINTERFACE);
	lastTXbytes = readTXbytes(NETWORKINTERFACE);
	lastRXpackets = readRXpackets(NETWORKINTERFACE);
	lastTXpackets = readTXpackets(NETWORKINTERFACE);
	lastStampBytes = ros::Time::now();
	lastStampPackets = ros::Time::now();

}

NetworkMonitor::~NetworkMonitor() {
	// TODO Auto-generated destructor stub
}

void NetworkMonitor::publishNetworkLoad(ros::Publisher load_pub,
		ros::Publisher rxbytes_pub, ros::Publisher txbytes_pub, bool bytes,
		bool load) {

	ros::Time now = ros::Time::now();
	unsigned int rxb = readRXbytes(NETWORKINTERFACE);
	unsigned int txb = readTXbytes(NETWORKINTERFACE);
	unsigned int deltaRXB = rxb - lastRXbytes;
	unsigned int deltaTXB = txb - lastTXbytes;
	ros::Duration deltaT = now - lastStampBytes;

	float timeT = (float) deltaT.sec + (deltaT.nsec / pow(10, 9));
	float RXBpS = (float) deltaRXB / timeT;
	float TXBpS = (float) deltaTXB / timeT;
	float loadinP = (RXBpS + TXBpS) / maxNWThroughputPS;

	/*ROS_INFO("RXB pro Sec: %f", RXBpS);
	 ROS_INFO("TXB pro Sec: %f", TXBpS);
	 ROS_INFO("Networkload: %f", loadinP);*/

	lastRXbytes = rxb;
	lastTXbytes = txb;
	lastStampBytes = now;

	if (bytes) {
		std_msgs::Float32 rxbytes, txbytes;
		rxbytes.data = RXBpS;
		rxbytes_pub.publish(rxbytes);
		txbytes.data = TXBpS;
		txbytes_pub.publish(txbytes);
	}
	if (load) {
		std_msgs::Float32 load_msgs;
		load_msgs.data = loadinP;
		load_pub.publish(load_msgs);
	}

}

unsigned int NetworkMonitor::readRXbytes(char nwinterface[]) {
	unsigned int rx_bytes;
	char path[256];
	sprintf(path, "%s%s%s/rx_bytes", NETWORKPRE, nwinterface, NETWORKSUF);

	FILE* file;
	file = fopen(path, "r");
	fscanf(file, "%d", &rx_bytes);
	fclose(file);
	return rx_bytes;
}

unsigned int NetworkMonitor::readTXbytes(char nwinterface[]) {
	unsigned int tx_bytes;
	char path[256];
	sprintf(path, "%s%s%s/tx_bytes", NETWORKPRE, nwinterface, NETWORKSUF);

	FILE* file;
	file = fopen(path, "r");
	fscanf(file, "%d", &tx_bytes);
	fclose(file);
	return tx_bytes;
}
unsigned int NetworkMonitor::readRXpackets(char nwinterface[]) {
	unsigned int rx_packets;
	char path[256];
	sprintf(path, "%s%s%s/rx_packets", NETWORKPRE, nwinterface, NETWORKSUF);

	FILE* file;
	file = fopen(path, "r");
	fscanf(file, "%d", &rx_packets);
	fclose(file);
	return rx_packets;
}
unsigned int NetworkMonitor::readTXpackets(char nwinterface[]) {
	unsigned int tx_packets;
	char path[256];
	sprintf(path, "%s%s%s/tx_packets", NETWORKPRE, nwinterface, NETWORKSUF);

	FILE* file;
	file = fopen(path, "r");
	fscanf(file, "%d", &tx_packets);
	fclose(file);
	return tx_packets;
}

void NetworkMonitor::publishPackets(ros::Publisher rxpackets_pub,
		ros::Publisher txpackets_pub) {
	ros::Time now = ros::Time::now();
	unsigned int rxp = readRXpackets(NETWORKINTERFACE);
	unsigned int txp = readTXpackets(NETWORKINTERFACE);
	unsigned int deltaRXP = rxp - lastRXpackets;
	unsigned int deltaTXP = txp - lastTXpackets;
	ros::Duration deltaT = now - lastStampPackets;

	float timeT = (float) deltaT.sec + (deltaT.nsec / pow(10, 9));
	float RXPpS = (float) deltaRXP / timeT;
	float TXPpS = (float) deltaTXP / timeT;

	lastRXpackets = rxp;
	lastTXpackets = txp;
	lastStampPackets = now;

	std_msgs::Float32 rxpackets, txpackets;
	rxpackets.data = RXPpS;
	txpackets.data = TXPpS;

	txpackets_pub.publish(txpackets);
	rxpackets_pub.publish(rxpackets);

}

int main(int argc, char **argv) {

	ros::init(argc, argv, "network_monitor");
	ros::NodeHandle n("~");
	ros::Publisher RXBpS_pub;
	ros::Publisher TXBpS_pub;
	ros::Publisher RXPpS_pub;
	ros::Publisher TXPpS_pub;
	ros::Publisher load_pub;

	float freq = 1;
	if (!n.getParam("frequency", freq)) {
		ROS_WARN("No frequency supplied. Working with %d Hz.", freq);
	}

	bool bBytes = false;
	if (n.getParam("bytes", bBytes)) {
		if (bBytes) {
			RXBpS_pub = n.advertise<std_msgs::Float32>(
					"/monitoring/network/rx_bytes_per_sec", 1);
			TXBpS_pub = n.advertise<std_msgs::Float32>(
					"/monitoring/network/tx_bytes_per_sec", 1);
		}
	}

	bool bPackets = false;
	if (n.getParam("packets", bPackets)) {
		if (bPackets) {
			RXPpS_pub = n.advertise<std_msgs::Float32>(
					"/monitoring/network/rx_packets_per_sec", 1);
			TXPpS_pub = n.advertise<std_msgs::Float32>(
					"/monitoring/network/tx_packets_per_sec", 1);
		}
	}

	bool bload = false;
	if (n.getParam("load", bload)) {
		if (bload) {
			load_pub = n.advertise<std_msgs::Float32>(
					"/monitoring/network/load", 1);
		}
	}
	float nwThroughput = 100;
	if (!n.getParam("networkthroughput", nwThroughput)) {
		ROS_WARN(
				"No Max Network Throughput configured. Working with 100Mbit/s.");
	}

	ros::Rate loop_rate(freq);

	NetworkMonitor NWm(nwThroughput);

	while (ros::ok()) {

		NWm.publishNetworkLoad(load_pub, RXBpS_pub, TXBpS_pub, bBytes, bload);
		if (bPackets) {
			NWm.publishPackets(RXPpS_pub, TXPpS_pub);
		}
		loop_rate.sleep();
	}

}
