/*
 * networkmonitor.cpp
 *
 *  Created on: Nov 8, 2017
 *      Author: matthias
 */

#include "ros_monitoring/monitors/networkmonitor.h"

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

	float rx_crc_errors, rx_dropped, rx_errors, rx_fifo_errors, rx_frame_errors,
			rx_length_errors, rx_missed_errors, rx_over_errors, tx_aborted_errors, tx_carrier_errors, tx_dropped, tx_errors,
			tx_fifo_errors, tx_heartbeat_errors, tx_window_errors;
	readNetworkErrors(rx_crc_errors, rx_dropped, rx_errors, rx_fifo_errors,
			rx_frame_errors, rx_length_errors, rx_missed_errors, rx_over_errors,
			tx_aborted_errors, tx_carrier_errors, tx_dropped, tx_errors,
								tx_fifo_errors, tx_heartbeat_errors, tx_window_errors);
	lastErrorsVector.push_back(rx_crc_errors);
	lastErrorsVector.push_back(rx_dropped);
	lastErrorsVector.push_back(rx_errors);
	lastErrorsVector.push_back(rx_fifo_errors);
	lastErrorsVector.push_back(rx_frame_errors);
	lastErrorsVector.push_back(rx_length_errors);
	lastErrorsVector.push_back(rx_missed_errors);
	lastErrorsVector.push_back(rx_over_errors);
	lastErrorsVector.push_back(tx_aborted_errors);
	lastErrorsVector.push_back(tx_carrier_errors);
	lastErrorsVector.push_back(tx_dropped);
	lastErrorsVector.push_back(tx_errors);
	lastErrorsVector.push_back(tx_fifo_errors);
	lastErrorsVector.push_back(tx_heartbeat_errors);
	lastErrorsVector.push_back(tx_window_errors);

}

NetworkMonitor::~NetworkMonitor() {
}

unsigned int NetworkMonitor::readNetworkInfo(char nwinterface[], char info[]) {
	char filepath[256];
	sprintf(filepath, "/%s/%s/%s/%s", NETWORKPRE, nwinterface, NETWORKSUF, info);

//	ROS_INFO("PATH: %s", filepath);
	FILE* file;
	file = fopen(filepath, "r");
	unsigned int ret;
	fscanf(file, "%d", &ret);
//	ROS_INFO("Value: %d", ret);
	fclose(file);

	return ret;
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

//	ROS_INFO("New rxb: %d", rxb);

//	ROS_INFO("RXB pro Sec: %f", RXBpS);
//	ROS_INFO("TXB pro Sec: %f", TXBpS);
//	ROS_INFO("Networkload: %f", loadinPtx);

	lastRXbytes = rxb;
	lastTXbytes = txb;
	lastStampBytes = now;

}

/*
 * Reads the RXbytes
 */
unsigned int NetworkMonitor::readRXbytes(char nwinterface[]) {
	return readNetworkInfo(nwinterface, "rx_bytes");
}

/*
 * Reads the TXbytes
 */
unsigned int NetworkMonitor::readTXbytes(char nwinterface[]) {
	return readNetworkInfo(nwinterface, "tx_bytes");
}

/*
 * Reads the RXpackets
 */
unsigned int NetworkMonitor::readRXpackets(char nwinterface[]) {
	return readNetworkInfo(nwinterface, "rx_packets");
}
/*
 * Reads the TXpackets
 */
unsigned int NetworkMonitor::readTXpackets(char nwinterface[]) {
	return readNetworkInfo(nwinterface, "tx_packets");
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
void NetworkMonitor::readNetworkErrors(float& rx_crc_errors, float& rx_dropped,
		float& rx_errors, float& rx_fifo_errors, float& rx_frame_errors,
		float& rx_length_errors, float& rx_missed_errors, float& rx_over_errors,
		float& tx_aborted_errors, float& tx_carrier_errors, float& tx_dropped, float& tx_errors,
		float& tx_fifo_errors, float& tx_heartbeat_errors,
		float& tx_window_errors) {
	rx_crc_errors = readNetworkInfo(networkinterface, "rx_crc_errors");
	rx_dropped = readNetworkInfo(networkinterface, "rx_dropped");
	rx_errors = readNetworkInfo(networkinterface, "rx_errors");
	rx_fifo_errors = readNetworkInfo(networkinterface, "rx_fifo_errors");
	rx_frame_errors = readNetworkInfo(networkinterface, "rx_frame_errors");
	rx_length_errors = readNetworkInfo(networkinterface, "rx_length_errors");
	rx_missed_errors = readNetworkInfo(networkinterface, "rx_missed_errors");
	rx_over_errors = readNetworkInfo(networkinterface, "rx_over_errors");

	tx_aborted_errors = readNetworkInfo(networkinterface, "tx_aborted_errors");
	tx_carrier_errors = readNetworkInfo(networkinterface, "tx_carrier_errors");
	tx_dropped = readNetworkInfo(networkinterface, "tx_dropped");
	tx_errors = readNetworkInfo(networkinterface, "tx_errors");
	tx_fifo_errors = readNetworkInfo(networkinterface, "tx_fifo_errors");
	tx_heartbeat_errors = readNetworkInfo(networkinterface, "tx_heartbeat_errors");
	tx_window_errors = readNetworkInfo(networkinterface, "tx_window_errors");
}
void NetworkMonitor::getErrors(float& rx_crc_errors, float& rx_dropped,
		float& rx_errors, float& rx_fifo_errors, float& rx_frame_errors,
		float& rx_length_errors, float& rx_missed_errors, float& rx_over_errors,
		float& tx_aborted_errors, float& tx_carrier_errors, float& tx_dropped, float& tx_errors,
				float& tx_fifo_errors, float& tx_heartbeat_errors,
				float& tx_window_errors) {

	float a, b, c, d, e, f, g, h, i, j, k, l, m, n, o;

	readNetworkErrors(a, b, c, d, e, f, g, h, i, j, k, l, m, n, o);

	std::vector<float> newErrorsVector;

	//rx
	rx_crc_errors = a - lastErrorsVector[0];
	newErrorsVector.push_back(a);
	rx_dropped = b - lastErrorsVector[1];
	newErrorsVector.push_back(b);
	rx_errors = c - lastErrorsVector[2];
	newErrorsVector.push_back(c);
	rx_fifo_errors = d - lastErrorsVector[3];
	newErrorsVector.push_back(d);
	rx_frame_errors = e - lastErrorsVector[4];
	newErrorsVector.push_back(e);
	rx_length_errors = f - lastErrorsVector[5];
	newErrorsVector.push_back(f);
	rx_missed_errors = g - lastErrorsVector[6];
	newErrorsVector.push_back(g);
	rx_over_errors = h - lastErrorsVector[7];
	newErrorsVector.push_back(h);

	//tx
	tx_aborted_errors = i - lastErrorsVector[8];
	newErrorsVector.push_back(i);
	tx_carrier_errors = j - lastErrorsVector[9];
	newErrorsVector.push_back(j);
	tx_dropped = k - lastErrorsVector[10];
	newErrorsVector.push_back(k);
	tx_errors = l - lastErrorsVector[11];
	newErrorsVector.push_back(l);
	tx_fifo_errors = m - lastErrorsVector[12];
	newErrorsVector.push_back(m);
	tx_heartbeat_errors = n - lastErrorsVector[13];
	newErrorsVector.push_back(n);
	tx_window_errors = o - lastErrorsVector[14];
	newErrorsVector.push_back(o);


	lastErrorsVector = newErrorsVector;

	return;

}

int main(int argc, char **argv) {

	ros::init(argc, argv, "network_monitor");
	ros::NodeHandle n("~");
	ros::Publisher pub = n.advertise<ros_monitoring::MonitoringArray>(
			"/monitoring", 1);

	float freq = 1;
	if (!n.getParam("frequency", freq)) {
		ROS_WARN("No frequency supplied. Working with %f Hz.", freq);
	}

	bool bBytes = false;
	if (n.getParam("bytes", bBytes)) {
	}

	bool bPackets = false;
	if (n.getParam("packets", bPackets)) {
	}

	bool bload = false;
	if (n.getParam("load", bload)) {
	}

	bool bErrors = false;
	if (n.getParam("networkerrors", bErrors)) {
	}

	float nwThroughput = 100;
	if (!n.getParam("networkthroughput", nwThroughput)) {
		ROS_WARN("No Max Network Throughput configured. Working with %fMbit/s.",
				nwThroughput);

	}

	std::string nwinterface = "lo";

	if (!n.getParam("networkinterface", nwinterface)) {
		ROS_WARN("No Network Interface configured. Using %s",
				nwinterface.c_str());
	}

	char *cnwInterface = new char[nwinterface.length() + 1];
	strcpy(cnwInterface, nwinterface.c_str());

	ros::Rate loop_rate(freq);

	NetworkMonitor NWm(nwThroughput, cnwInterface);



	MonitorMsg msg(n, ros::this_node::getName(), "Network-Monitor");

	char value[40];
	while (ros::ok()) {
		msg.resetMsg();
		msg.addNewInfoTree(nwinterface, "Networkinterface monitor");

		if (bBytes || bload) {
			float loadrx, loadtx, RXBpS, TXBpS;
			NWm.getNetworkLoad(loadrx, loadtx, RXBpS, TXBpS);
			if (bBytes) {
				msg.addValue("RX", RXBpS, "Byte/s", 0);
				msg.addValue("TX", TXBpS, "Byte/s", 0);
			}

			if (bload) {
				msg.addValue("Load RX", loadrx, "%", 0);
				msg.addValue("Load TX", loadtx, "%", 0);
			}

		}
		if (bPackets) {
			float RXPpS, TXPpS;

			NWm.getPackets(RXPpS, TXPpS);
			msg.addValue("RX", RXPpS, "Packet/s", 0);
			msg.addValue("TX", TXPpS, "Packet/s", 0);


		}

		if (bErrors) {
			float rx_crc_errors, rx_dropped, rx_errors, rx_fifo_errors,
					rx_frame_errors, rx_length_errors, rx_missed_errors,
					rx_over_errors, tx_aborted_errors, tx_carrier_errors, tx_dropped, tx_errors,
					tx_fifo_errors, tx_heartbeat_errors,
					tx_window_errors;
			NWm.getErrors(rx_crc_errors, rx_dropped, rx_errors, rx_fifo_errors,
					rx_frame_errors, rx_length_errors, rx_missed_errors,
					rx_over_errors, tx_aborted_errors, tx_carrier_errors, tx_dropped, tx_errors,
					tx_fifo_errors, tx_heartbeat_errors, tx_window_errors);

			ros_monitoring::KeyValue a_kv, b_kv, c_kv, d_kv, e_kv, f_kv, g_kv,
					h_kv, i_kv, j_kv, k_kv, l_kv, m_kv, n_kv, o_kv, p_kv;

			msg.addValue("RX_CRC_Errors", rx_crc_errors, "B", 0);
			msg.addValue("RX_Dropped", rx_dropped, "B", 0);
			msg.addValue("RX_Errors", rx_errors, "B", 0);
			msg.addValue("RX_FIFO_Errors", rx_fifo_errors, "B", 0);
			msg.addValue("RX_Frame_Errors", rx_frame_errors, "B", 0);
			msg.addValue("RX_Length_Errors", rx_length_errors, "B", 0);
			msg.addValue("RX_Missed_Errors", rx_missed_errors, "B", 0);
			msg.addValue("RX_Over_Errors", rx_over_errors, "B", 0);

			//########################
			msg.addValue("TX_Carrier_Errors", tx_carrier_errors, "B", 0);
			msg.addValue("TX_Dropped", tx_dropped, "B", 0);
			msg.addValue("TX_Errors", tx_errors, "B", 0);
			msg.addValue("TX_FIFO_Errors", tx_fifo_errors, "B", 0);
			msg.addValue("TX_Heartbeat_Errors", tx_heartbeat_errors, "B", 0);
			msg.addValue("TX_Window_Errors", tx_window_errors, "B", 0);
			msg.addValue("TX_Aborted_Errors", tx_aborted_errors, "B", 0);

		}
		msg.publish();
		loop_rate.sleep();
	}

	delete[] cnwInterface;

}
