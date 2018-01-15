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
			rx_length_errors, rx_missed_errors, rx_over_errors, tx_crc_errors,
			tx_dropped, tx_errors, tx_fifo_errors, tx_frame_errors,
			tx_length_errors, tx_missed_errors, tx_over_errors;
	readNetworkErrors(rx_crc_errors, rx_dropped, rx_errors, rx_fifo_errors,
			rx_frame_errors, rx_length_errors, rx_missed_errors, rx_over_errors,
			tx_crc_errors, tx_dropped, tx_errors, tx_fifo_errors,
			tx_frame_errors, tx_length_errors, tx_missed_errors,
			tx_over_errors);
	lastErrorsVector.push_back(rx_crc_errors);
	lastErrorsVector.push_back(rx_dropped);
	lastErrorsVector.push_back(rx_errors);
	lastErrorsVector.push_back(rx_fifo_errors);
	lastErrorsVector.push_back(rx_frame_errors);
	lastErrorsVector.push_back(rx_length_errors);
	lastErrorsVector.push_back(rx_missed_errors);
	lastErrorsVector.push_back(rx_over_errors);
	lastErrorsVector.push_back(tx_crc_errors);
	lastErrorsVector.push_back(tx_dropped);
	lastErrorsVector.push_back(tx_errors);
	lastErrorsVector.push_back(tx_fifo_errors);
	lastErrorsVector.push_back(tx_frame_errors);
	lastErrorsVector.push_back(tx_length_errors);
	lastErrorsVector.push_back(tx_missed_errors);
	lastErrorsVector.push_back(tx_over_errors);

}

NetworkMonitor::~NetworkMonitor() {
}

unsigned int NetworkMonitor::readNetworkInfo(char nwinterface[], char info[]) {
	char path[256];
	sprintf(path, "/%s/%s/%s/", NETWORKPRE, nwinterface, NETWORKSUF);

	FILE* file;
	file = fopen(path, "r");
	int ret;
	fscanf(file, "%d", ret);
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
		float& tx_crc_errors, float& tx_dropped, float& tx_errors,
		float& tx_fifo_errors, float& tx_frame_errors, float& tx_length_errors,
		float& tx_missed_errors, float& tx_over_errors) {
	rx_crc_errors = readNetworkInfo(networkinterface, "rx_crc_errors");
	rx_dropped = readNetworkInfo(networkinterface, "rx_dropped");
	rx_errors = readNetworkInfo(networkinterface, "rx_errors");
	rx_fifo_errors = readNetworkInfo(networkinterface, "rx_fifo_errors");
	rx_frame_errors = readNetworkInfo(networkinterface, "rx_frame_errors");
	rx_length_errors = readNetworkInfo(networkinterface, "rx_length_errors");
	rx_missed_errors = readNetworkInfo(networkinterface, "rx_missed_errors");
	rx_over_errors = readNetworkInfo(networkinterface, "rx_over_errors");

	tx_crc_errors = readNetworkInfo(networkinterface, "tx_crc_errors");
	tx_dropped = readNetworkInfo(networkinterface, "tx_dropped");
	tx_errors = readNetworkInfo(networkinterface, "tx_errors");
	tx_fifo_errors = readNetworkInfo(networkinterface, "tx_fifo_errors");
	tx_frame_errors = readNetworkInfo(networkinterface, "tx_frame_errors");
	tx_length_errors = readNetworkInfo(networkinterface, "tx_length_errors");
	tx_missed_errors = readNetworkInfo(networkinterface, "tx_missed_errors");
	tx_over_errors = readNetworkInfo(networkinterface, "tx_over_errors");
}
void NetworkMonitor::getErrors(float& rx_crc_errors, float& rx_dropped,
		float& rx_errors, float& rx_fifo_errors, float& rx_frame_errors,
		float& rx_length_errors, float& rx_missed_errors, float& rx_over_errors,
		float& tx_crc_errors, float& tx_dropped, float& tx_errors,
		float& tx_fifo_errors, float& tx_frame_errors, float& tx_length_errors,
		float& tx_missed_errors, float& tx_over_errors) {

	float a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p;

	readNetworkErrors(a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p);

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
	tx_crc_errors = i - lastErrorsVector[8];
	newErrorsVector.push_back(i);
	tx_dropped = j - lastErrorsVector[9];
	newErrorsVector.push_back(j);
	tx_errors = k - lastErrorsVector[10];
	newErrorsVector.push_back(k);
	tx_fifo_errors = l - lastErrorsVector[11];
	newErrorsVector.push_back(l);
	tx_frame_errors = m - lastErrorsVector[12];
	newErrorsVector.push_back(m);
	tx_length_errors = n - lastErrorsVector[13];
	newErrorsVector.push_back(n);
	tx_missed_errors = o - lastErrorsVector[14];
	newErrorsVector.push_back(o);
	tx_over_errors = p - lastErrorsVector[15];
	newErrorsVector.push_back(p);

	lastErrorsVector = newErrorsVector;

	return;

}

int main(int argc, char **argv) {

	ros::init(argc, argv, "network_monitor");
	ros::NodeHandle n("~");
	ros::Publisher pub = n.advertise<ros_monitoring::MonitoringArray>(
			"/monitoring/all", 1);

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

	//benchmark init things
	ROS_RT_Benchmark benchmark;
	benchmark.init();
	ROS_RT_MeasurementDuration* measurement_networkload_bytes =
			benchmark.createDurationMeasurement("networkload_in_bytes");
	ROS_RT_MeasurementDuration* measurement_network_packets =
			benchmark.createDurationMeasurement("network_packets");

	char value[40];
	while (ros::ok()) {
		ros_monitoring::MonitoringArray ma;
		ros_monitoring::MonitoringInfo mi;
		char name[256];
		sprintf(name, "%s (%s)", ros::this_node::getName().c_str(), cnwInterface);
		mi.name = std::string(name);
		mi.description = "A Network-Monitor";
		fillMachineInfo(mi);

		if (bBytes || bload) {
			float loadrx, loadtx, RXBpS, TXBpS;
			measurement_networkload_bytes->start();
			NWm.getNetworkLoad(loadrx, loadtx, RXBpS, TXBpS);
			measurement_networkload_bytes->stop();
			if (bBytes) {
				ros_monitoring::KeyValue rx_kv, tx_kv;
				sprintf(value, "rx in Bytes per Second (%s)", cnwInterface);
				rx_kv.key = value;
				sprintf(value, "%f", RXBpS);
				rx_kv.value = value;
				rx_kv.unit = "B/s";

				sprintf(value, "tx in Bytes per Second (%s)", cnwInterface);
				tx_kv.key = value;
				sprintf(value, "%f", TXBpS);
				tx_kv.value = value;
				tx_kv.unit = "B/s";

				mi.values.push_back(rx_kv);
				mi.values.push_back(tx_kv);

			}

			if (bload) {

				ros_monitoring::KeyValue loadrx_kv, loadtx_kv;
				sprintf(value, "Network Load RX on %s", cnwInterface);
				loadrx_kv.key = value;
				sprintf(value, "%f", loadrx);
				loadrx_kv.value = value;
				loadrx_kv.unit = "%";

				sprintf(value, "Network Load TX on %s", cnwInterface);
				loadtx_kv.key = value;
				sprintf(value, "%f", loadtx);
				loadtx_kv.value = value;
				loadtx_kv.unit = "%";

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
			rxkv.unit = "P/s";

			sprintf(value, "tx in Packets per Second (%s)", cnwInterface);
			txkv.key = value;
			sprintf(value, "%f", TXPpS);
			txkv.value = value;
			txkv.unit = "P/s";

			mi.values.push_back(rxkv);
			mi.values.push_back(txkv);
		}

		if (bErrors) {
			float rx_crc_errors, rx_dropped, rx_errors, rx_fifo_errors,
					rx_frame_errors, rx_length_errors, rx_missed_errors,
					rx_over_errors, tx_crc_errors, tx_dropped, tx_errors,
					tx_fifo_errors, tx_frame_errors, tx_length_errors,
					tx_missed_errors, tx_over_errors;
			NWm.getErrors(rx_crc_errors, rx_dropped, rx_errors, rx_fifo_errors,
					rx_frame_errors, rx_length_errors, rx_missed_errors,
					rx_over_errors, tx_crc_errors, tx_dropped, tx_errors,
					tx_fifo_errors, tx_frame_errors, tx_length_errors,
					tx_missed_errors, tx_over_errors);

			ros_monitoring::KeyValue a_kv, b_kv, c_kv, d_kv, e_kv, f_kv, g_kv,
					h_kv, i_kv, j_kv, k_kv, l_kv, m_kv, n_kv, o_kv, p_kv;
			sprintf(value, "rx_crc_errors (%s)", cnwInterface);
			a_kv.key = value;
			sprintf(value, "%f", rx_crc_errors);
			a_kv.value = value;
			a_kv.unit = "B";
			mi.values.push_back(a_kv);

			sprintf(value, "rx_dropped (%s)", cnwInterface);
			b_kv.key = value;
			sprintf(value, "%f", rx_dropped);
			b_kv.value = value;
			b_kv.unit = "B";
			mi.values.push_back(b_kv);

			sprintf(value, "rx_errors (%s)", cnwInterface);
			c_kv.key = value;
			sprintf(value, "%f", rx_errors);
			c_kv.value = value;
			c_kv.unit = "B";
			mi.values.push_back(c_kv);

			sprintf(value, "rx_fifo_errors (%s)", cnwInterface);
			d_kv.key = value;
			sprintf(value, "%f", rx_fifo_errors);
			d_kv.value = value;
			d_kv.unit = "B";
			mi.values.push_back(d_kv);

			sprintf(value, "rx_frame_errors (%s)", cnwInterface);
			e_kv.key = value;
			sprintf(value, "%f", rx_frame_errors);
			e_kv.value = value;
			e_kv.unit = "B";
			mi.values.push_back(e_kv);

			sprintf(value, "rx_length_errors (%s)", cnwInterface);
			f_kv.key = value;
			sprintf(value, "%f", rx_length_errors);
			f_kv.value = value;
			f_kv.unit = "B";
			mi.values.push_back(f_kv);

			sprintf(value, "rx_missed_errors (%s)", cnwInterface);
			g_kv.key = value;
			sprintf(value, "%f", rx_missed_errors);
			g_kv.value = value;
			g_kv.unit = "B";
			mi.values.push_back(g_kv);

			sprintf(value, "rx_over_errors (%s)", cnwInterface);
			h_kv.key = value;
			sprintf(value, "%f", rx_over_errors);
			h_kv.value = value;
			h_kv.unit = "B";
			mi.values.push_back(h_kv);

			sprintf(value, "tx_crc_errors (%s)", cnwInterface);
			i_kv.key = value;
			sprintf(value, "%f", tx_crc_errors);
			i_kv.value = value;
			i_kv.unit = "B";
			mi.values.push_back(i_kv);

			sprintf(value, "tx_dropped (%s)", cnwInterface);
			j_kv.key = value;
			sprintf(value, "%f", tx_dropped);
			j_kv.value = value;
			j_kv.unit = "B";
			mi.values.push_back(j_kv);

			sprintf(value, "tx_errors (%s)", cnwInterface);
			k_kv.key = value;
			sprintf(value, "%f", tx_errors);
			k_kv.value = value;
			k_kv.unit = "B";
			mi.values.push_back(k_kv);

			sprintf(value, "tx_fifo_errors (%s)", cnwInterface);
			l_kv.key = value;
			sprintf(value, "%f", tx_fifo_errors);
			l_kv.value = value;
			l_kv.unit = "B";
			mi.values.push_back(l_kv);

			sprintf(value, "tx_frame_errors (%s)", cnwInterface);
			m_kv.key = value;
			sprintf(value, "%f", tx_frame_errors);
			m_kv.value = value;
			m_kv.unit = "B";
			mi.values.push_back(m_kv);

			sprintf(value, "tx_length_errors (%s)", cnwInterface);
			n_kv.key = value;
			sprintf(value, "%f", tx_length_errors);
			n_kv.value = value;
			n_kv.unit = "B";
			mi.values.push_back(n_kv);

			sprintf(value, "tx_missed_errors (%s)", cnwInterface);
			o_kv.key = value;
			sprintf(value, "%f", tx_missed_errors);
			o_kv.value = value;
			o_kv.unit = "B";
			mi.values.push_back(o_kv);

			sprintf(value, "tx_over_errors (%s)", cnwInterface);
			p_kv.key = value;
			sprintf(value, "%f", tx_over_errors);
			p_kv.value = value;
			p_kv.unit = "B";
			mi.values.push_back(p_kv);
		}
		ma.info.push_back(mi);
		pub.publish(ma);
		loop_rate.sleep();
	}

	delete[] cnwInterface;

	benchmark.logData();
}
