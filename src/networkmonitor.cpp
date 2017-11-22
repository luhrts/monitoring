/*
 * networkmonitor.cpp
 *
 *  Created on: Nov 8, 2017
 *      Author: matthias
 */

#include "networkmonitor.h"

NetworkMonitor::NetworkMonitor(float networkThroughput, char nwit[]) {
	maxNWThroughputPS = (float) (networkThroughput * pow(10, 6)) / 8; //transforms MBit/s to Byte/s
	networkinterface = nwit;
	//reading inital values, because the values are given incrementel
	lastRXbytes = readRXbytes(networkinterface);
	lastTXbytes = readTXbytes(networkinterface);
	lastRXpackets = readRXpackets(networkinterface);
	lastTXpackets = readTXpackets(networkinterface);
	lastStampBytes = ros::Time::now();
	lastStampPackets = ros::Time::now();
}

NetworkMonitor::~NetworkMonitor() {
}

/*
 * Calculates the network trafic per second in bytes and network load in scala 0..1 for rx and tx
 */
void NetworkMonitor::getNetworkLoad(float& loadinPrx, float& loadinPtx,
		float& RXBpS, float& TXBpS) {

	ros::Time now = ros::Time::now();
	unsigned int rxb = readRXbytes(networkinterface);
	unsigned int txb = readTXbytes(networkinterface);
	unsigned int deltaRXB = rxb - lastRXbytes;
	unsigned int deltaTXB = txb - lastTXbytes;
	ros::Duration deltaT = now - lastStampBytes;

	float timeT = (float) deltaT.sec + (deltaT.nsec / pow(10, 9));
	RXBpS = (float) deltaRXB / timeT;
	TXBpS = (float) deltaTXB / timeT;
	loadinPrx = (RXBpS /*+ TXBpS*/) / maxNWThroughputPS;
	loadinPtx = (TXBpS /*+ TXBpS*/) / maxNWThroughputPS;

	/*ROS_INFO("RXB pro Sec: %f", RXBpS);
	 ROS_INFO("TXB pro Sec: %f", TXBpS);
	 ROS_INFO("Networkload: %f", loadinP);*/

	lastRXbytes = rxb;
	lastTXbytes = txb;
	lastStampBytes = now;

}

/*
 * Reads the RXbytes
 */
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

/*
 * Reads the TXbytes
 */
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

/*
 * Reads the RXpackets
 */
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
/*
 * Reads the TXpackets
 */
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

/*
 * calculates the networktrafic per second for packets
 */
void NetworkMonitor::getPackets(float& RXPpS, float& TXPpS) {
	ros::Time now = ros::Time::now();
	unsigned int rxp = readRXpackets(networkinterface);
	unsigned int txp = readTXpackets(networkinterface);
	unsigned int deltaRXP = rxp - lastRXpackets;
	unsigned int deltaTXP = txp - lastTXpackets;
	ros::Duration deltaT = now - lastStampPackets;

	float timeT = (float) deltaT.sec + (deltaT.nsec / pow(10, 9));
	RXPpS = (float) deltaRXP / timeT;
	TXPpS = (float) deltaTXP / timeT;

	lastRXpackets = rxp;
	lastTXpackets = txp;
	lastStampPackets = now;

	return;

}

int main(int argc, char **argv) {

	ros::init(argc, argv, "network_monitor");
	ros::NodeHandle n("~");
	ros::Publisher RXBpS_pub, TXBpS_pub, RXPpS_pub, TXPpS_pub, loadrx_pub,
			loadtx_pub;

	float freq = 1;
	if (!n.getParam("frequency", freq)) {
		ROS_WARN("No frequency supplied. Working with %f Hz.", freq);
	}

	bool bBytes = false;
	if (n.getParam("bytes", bBytes)) {
//		if (bBytes) {
//			RXBpS_pub = n.advertise<std_msgs::Float32>(
//					"/monitoring/network/rx_bytes_per_sec", 1);
//			TXBpS_pub = n.advertise<std_msgs::Float32>(
//					"/monitoring/network/tx_bytes_per_sec", 1);
//		}
	}

	bool bPackets = false;
	if (n.getParam("packets", bPackets)) {
//		if (bPackets) {
//			RXPpS_pub = n.advertise<std_msgs::Float32>(
//					"/monitoring/network/rx_packets_per_sec", 1);
//			TXPpS_pub = n.advertise<std_msgs::Float32>(
//					"/monitoring/network/tx_packets_per_sec", 1);
//		}
	}

	bool bload = false;
	if (n.getParam("load", bload)) {
//		if (bload) {
//			loadrx_pub = n.advertise<std_msgs::Float32>(
//					"/monitoring/network/loadrx", 1);
//			loadtx_pub = n.advertise<std_msgs::Float32>(
//					"/monitoring/network/loadtx", 1);
//		}
	}
	float nwThroughput = 100;
	if (!n.getParam("networkthroughput", nwThroughput)) {
		ROS_WARN(
				"No Max Network Throughput configured. Working with 100Mbit/s.");
	}

	std::string nwinterface = "enp3s0";

	if (!n.getParam("networkinterface", nwinterface)) {
		ROS_WARN("No Network Interface configured. Using %s",
				nwinterface.c_str());
	}

	char *cnwInterface = new char[nwinterface.length() + 1];
	strcpy(cnwInterface, nwinterface.c_str());

	ros::Rate loop_rate(freq);

	NetworkMonitor NWm(nwThroughput, cnwInterface);

	//benchmark init things
	ROS_RT_Benchmark benchmark;
	benchmark.init();
	ROS_RT_MeasurementDuration* measurement_networkload_bytes =
			benchmark.createDurationMeasurement("networkload_in_bytes");
	ROS_RT_MeasurementDuration* measurement_network_packets =
			benchmark.createDurationMeasurement("network_packets");

	char value[40];
	while (ros::ok()) {
		ros_monitoring::MonitoringInfo mi;
		mi.name = ros::this_node::getName();
		mi.description = "A Network-Monitor";
		fillMachineInfo(mi);

		if (bBytes || bload) {
			float loadrx, loadtx, RXBpS, TXBpS;
			measurement_networkload_bytes->start();
			NWm.getNetworkLoad(loadrx, loadtx, RXBpS, TXBpS);
			measurement_networkload_bytes->stop();
			if (bBytes) {
//				std_msgs::Float32 rx_msg, tx_msg;
//				rx_msg.data = RXBpS;
//				RXBpS_pub.publish(rx_msg);
//				tx_msg.data = TXBpS;
//				TXBpS_pub.publish(tx_msg);
				ros_monitoring::KeyValue rx_kv, tx_kv;
				sprintf(value, "rx in Bytes per Second (%s)", cnwInterface);
				rx_kv.key = value;
				sprintf(value, "%f", RXBpS);
				rx_kv.value = value;

				sprintf(value, "tx in Bytes per Second (%s)", cnwInterface);
				tx_kv.key = value;
				sprintf(value, "%f", TXBpS);
				tx_kv.value = value;

				mi.values.push_back(rx_kv);
				mi.values.push_back(tx_kv);

			}

			if (bload) {
//				std_msgs::Float32 rx_load_msg, tx_load_msg;
//				rx_load_msg.data = loadrx;
//				tx_load_msg.data = loadtx;
//				loadrx_pub.publish(rx_load_msg);
//				loadtx_pub.publish(tx_load_msg);

				ros_monitoring::KeyValue loadrx_kv, loadtx_kv;
				sprintf(value, "Network Load RX on %s", cnwInterface);
				loadrx_kv.key = value;
				sprintf(value, "%f", loadrx);
				loadrx_kv.value = value;

				sprintf(value, "Network Load TX on %s", cnwInterface);
				loadtx_kv.key = value;
				sprintf(value, "%f", loadtx);
				loadtx_kv.value = value;

				mi.values.push_back(loadrx_kv);
				mi.values.push_back(loadtx_kv);
			}

		}
		if (bPackets) {
			float RXPpS, TXPpS;
			measurement_network_packets->start();
			NWm.getPackets(RXPpS, TXPpS);
			measurement_network_packets->stop();

			ros_monitoring::KeyValue rxkv, txkv;

			sprintf(value, "rx in Packets per Second (%s)", cnwInterface);
			rxkv.key = value;
			sprintf(value, "%f", RXPpS);
			rxkv.value = value;

			sprintf(value, "tx in Packets per Second (%s)", cnwInterface);
			txkv.key = value;
			sprintf(value, "%f", TXPpS);
			txkv.value = value;

			mi.values.push_back(rxkv);
			mi.values.push_back(txkv);
		}
		loop_rate.sleep();
	}

	delete[] cnwInterface;

	benchmark.logData();
}
