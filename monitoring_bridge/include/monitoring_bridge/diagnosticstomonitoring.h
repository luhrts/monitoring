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

#ifndef SRC_DIAGNOSTICSTOMONITORING_H_
#define SRC_DIAGNOSTICSTOMONITORING_H_

#include "ros/ros.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "monitoring_core/monitor.h"

/**
 * @brief The DiagnosticsToMonitoring class allows to convert diagnostics msgs to monitoring msgs
 */
class DiagnosticsToMonitoring {
public:
	DiagnosticsToMonitoring(ros::NodeHandle& n);
	virtual ~DiagnosticsToMonitoring();
  /**
   * @brief publish publshies all received diagnostics msgs
   */
	void publish();
private:
  /**
   * @brief diagnostics_callback receives all diagnostics msgs
   * @param msg
   */
	void diagnostics_callback(diagnostic_msgs::DiagnosticArray msg);
  ros::Publisher monitor_pub; ///< publishes the msgs
  ros::Subscriber diag_sub;   ///< receives all diagnostics msgs
  Monitor* monitormsg;

};

#endif /* SRC_MONITORINGTODIAGNOSTICS_H_ */
