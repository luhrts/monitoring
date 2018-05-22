/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018  University of Hannover
 *                     Institute for Systems Engineering - RTS
 *                     Prof. Dr.-Ing. Bernardo Wagner
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name University of Hannover nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef SRC_NETWORKMONITOR_H_
#define SRC_NETWORKMONITOR_H_

#include "ros/ros.h"
#include "monitoring_core/monitor.h"

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
		  float& tx_aborted_errors, float& tx_carrier_errors, float& tx_dropped, float& tx_errors, float& tx_fifo_errors, float& tx_heartbeat_errors, float& tx_window_errors);


private:
  unsigned int readNetworkInfo(char nwinterface[], char info[]);
  float maxNWThroughputPS;
  unsigned int readRXbytes(char nwinterface[]);
  unsigned int readTXbytes(char nwinterface[]);
  unsigned int readRXpackets(char nwinterface[]);
  unsigned int readTXpackets(char nwinterface[]);
  void readNetworkErrors(float& rx_crc_errors, float& rx_dropped, float& rx_errors, float& rx_fifo_errors,
  		float& rx_frame_errors,	float& rx_length_errors, float& rx_missed_errors, float& rx_over_errors,
		float& tx_aborted_errors, float& tx_carrier_errors, float& tx_dropped, float& tx_errors,
						float& tx_fifo_errors, float& tx_heartbeat_errors, float& tx_window_errors);

  ros::Time lastStampBytes, lastStampPackets;
  unsigned int lastRXbytes, lastTXbytes, lastRXpackets, lastTXpackets;
  char *networkinterface;

  std::vector<float> lastErrorsVector;

};

#endif /* SRC_NETWORKMONITOR_H_ */
