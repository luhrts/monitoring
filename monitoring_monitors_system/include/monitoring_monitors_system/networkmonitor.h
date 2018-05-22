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

/**
 * @brief The NetworkMonitor class is a monitor for networkinterfaces like eth, wifi or lo. If you need to monitor multiple interfaces, create a monitor for each interface
 */
class NetworkMonitor
{
public:
  /**
   * @brief NetworkMonitor is the constructor
   * @param networkThroughput is the maximal expected bandwidth (in MBit/s) of this interface (remember, a connection over a 100MBit/s ethernet port can probably not deliver a 100MBit/s. If you need a accurate percentage, you need to test your connection)
   * @param nwit is the name of the interface (like lo or eth0)
   */
  NetworkMonitor(float networkThroughput, char nwit[]);
  virtual ~NetworkMonitor();

  /**
   * @brief getNetworkLoad calculates the load of the objects networkinterface
   * @param loadinPrx returns the received load in 0..1
   * @param loadinPtx returns the transmitted load in 0..1
   * @param RXBpS returns the received in B/s
   * @param TXBpS returns the transmitted in B/s
   */
  void getNetworkLoad(float& loadinPrx, float& loadinPtx, float& RXBpS, float& TXBpS);

  /**
   * @brief getPackets calculates the trafic in packets
   * @param RXPpS returns the received packets/s
   * @param TXPpS returns the transmitted packets/s
   */
  void getPackets(float& RXPpS, float& TXPpS);

  /**
   * @brief getErrors reads all networkinterface errors from the proc filesystem.
   * @param rx_crc_errors
   * @param rx_dropped
   * @param rx_errors
   * @param rx_fifo_errors
   * @param rx_frame_errors
   * @param rx_length_errors
   * @param rx_missed_errors
   * @param rx_over_errors
   * @param tx_aborted_errors
   * @param tx_carrier_errors
   * @param tx_dropped
   * @param tx_errors
   * @param tx_fifo_errors
   * @param tx_heartbeat_errors
   * @param tx_window_errors
   */
  void getErrors(float& rx_crc_errors, float& rx_dropped, float& rx_errors, float& rx_fifo_errors, float& rx_frame_errors, float& rx_length_errors, float& rx_missed_errors, float& rx_over_errors,
		  float& tx_aborted_errors, float& tx_carrier_errors, float& tx_dropped, float& tx_errors, float& tx_fifo_errors, float& tx_heartbeat_errors, float& tx_window_errors);


private:
  /**
   * @brief readNetworkInfo reads the wanted info from the filesystem
   * @param nwinterface is the network interface name
   * @param info is the name that needs to be read
   * @return value of the read value
   */
  unsigned int readNetworkInfo(char nwinterface[], char info[]);
  /**
   * @brief readRXbytes reads the Received bytes
   * @param nwinterface is the network interface name
   * @return received bytes
   */
  unsigned int readRXbytes(char nwinterface[]);
  /**
   * @brief readTXbytes reads the transmitted bytes
   * @param nwinterface is the network interface name
   * @return transmitted bytes
   */
  unsigned int readTXbytes(char nwinterface[]);
  /**
   * @brief readRXpackets reads received packets
   * @param nwinterface is the network interface name
   * @return received packets
   */
  unsigned int readRXpackets(char nwinterface[]);
  /**
   * @brief readTXpackets reads the transmitted packets
   * @param nwinterface is the network interface name
   * @return transmitted packets
   */
  unsigned int readTXpackets(char nwinterface[]);
  /**
   * @brief readNetworkErrors reads all possible network errors
   * @param rx_crc_errors
   * @param rx_dropped
   * @param rx_errors
   * @param rx_fifo_errors
   * @param rx_frame_errors
   * @param rx_length_errors
   * @param rx_missed_errors
   * @param rx_over_errors
   * @param tx_aborted_errors
   * @param tx_carrier_errors
   * @param tx_dropped
   * @param tx_errors
   * @param tx_fifo_errors
   * @param tx_heartbeat_errors
   * @param tx_window_errors
   */
  void readNetworkErrors(float& rx_crc_errors, float& rx_dropped, float& rx_errors, float& rx_fifo_errors,
  		float& rx_frame_errors,	float& rx_length_errors, float& rx_missed_errors, float& rx_over_errors,
		float& tx_aborted_errors, float& tx_carrier_errors, float& tx_dropped, float& tx_errors,
						float& tx_fifo_errors, float& tx_heartbeat_errors, float& tx_window_errors);

  float maxNWThroughputPS;  ///< the maximum network bandwith of this interface
  ros::Time lastStampBytes, lastStampPackets; ///< saves the time the last read was done
  unsigned int lastRXbytes, lastTXbytes, lastRXpackets, lastTXpackets; ///< saves the last read values
  char *networkinterface;   ///< the name of the network interface

  std::vector<float> lastErrorsVector;  ///< saves all last read errorvalues

};

#endif /* SRC_NETWORKMONITOR_H_ */
