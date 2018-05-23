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

#ifndef SRC_MONITORINGTODIAGNOSTICS_H_
#define SRC_MONITORINGTODIAGNOSTICS_H_

#include "ros/ros.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "monitoring_msgs/MonitoringArray.h"

/**
 * @brief The MonitoringToDiagnostics class allows the conversion from monitoring to diagnostics stack
 */
class MonitoringToDiagnostics {
public:
	MonitoringToDiagnostics(ros::NodeHandle& n);
	virtual ~MonitoringToDiagnostics();
  /**
   * @brief publish all data for diagnostics
   */
	void publish();
private:
  /**
   * @brief monitor_callback receives all data from monitoring
   * @param msg
   */
  void monitor_callback(monitoring_msgs::MonitoringArray msg);
  ros::Publisher diag_pub;  ///< diagnostics publisher
  ros::Subscriber monitor_sub;  ///< monitoring subscriber
  diagnostic_msgs::DiagnosticArray da;  ///< array to save incoming data untill it will be published
};

#endif /* SRC_MONITORINGTODIAGNOSTICS_H_ */
