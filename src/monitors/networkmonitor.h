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
#include "ros_monitoring/MonitoringArray.h"
#include "ros_monitoring/help.h"
#include "ros_rt_benchmark_lib/benchmark.h"

#define NETWORKPRE "sys/class/net"
#define NETWORKSUF "statistics"

class NetworkMonitor
{
public:
  NetworkMonitor(float networkThroughput, char nwit[]);
  virtual ~NetworkMonitor();

  void getNetworkLoad(float& loadinPrx, float& loadinPtx, float& RXBpS, float& TXBpS);
  void getPackets(float& RXPpS, float& TXPpS);
  void getErrors(float& rx_crc_errors, float& rx_dropped, float& rx_errors, float& rx_fifo_errors, float& rx_frame_errors, float& rx_length_errors, float& rx_missed_errors, float& rx_over_errors,
  		float& tx_crc_errors, float& tx_dropped, float& tx_errors, float& tx_fifo_errors, float& tx_frame_errors, float& tx_length_errors, float& tx_missed_errors, float& tx_over_errors);


private:
  unsigned int readNetworkInfo(char nwinterface[], char info[]);
  float maxNWThroughputPS;
  unsigned int readRXbytes(char nwinterface[]);
  unsigned int readTXbytes(char nwinterface[]);
  unsigned int readRXpackets(char nwinterface[]);
  unsigned int readTXpackets(char nwinterface[]);
  void readNetworkErrors(float& rx_crc_errors, float& rx_dropped, float& rx_errors, float& rx_fifo_errors,
  		float& rx_frame_errors,	float& rx_length_errors, float& rx_missed_errors, float& rx_over_errors,
  		float& tx_crc_errors, float& tx_dropped, float& tx_errors, float& tx_fifo_errors, float& tx_frame_errors,
  		float& tx_length_errors, float& tx_missed_errors, float& tx_over_errors);

  ros::Time lastStampBytes, lastStampPackets;
  unsigned int lastRXbytes, lastTXbytes, lastRXpackets, lastTXpackets;
  char *networkinterface;

  std::vector<float> lastErrorsVector;

};

#endif /* SRC_NETWORKMONITOR_H_ */
