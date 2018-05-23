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

#include "monitoring_bridge/diagnosticstomonitoring.h"

DiagnosticsToMonitoring::DiagnosticsToMonitoring(ros::NodeHandle &node) {
  monitor_pub = node.advertise<monitoring_msgs::MonitoringArray>("/monitoring", 1);
  diag_sub = node.subscribe("/diagnostics", 10, &DiagnosticsToMonitoring::diagnostics_callback, this);
}

DiagnosticsToMonitoring::~DiagnosticsToMonitoring() {

}

void DiagnosticsToMonitoring::diagnostics_callback(diagnostic_msgs::DiagnosticArray msg) {
  for(int i=0; i<msg.status.size(); i++) {
    monitoring_msgs::MonitoringInfo mi;
    double errorlevel = msg.status[i].level/2.0;
    if(errorlevel >= 1.0) {
      errorlevel = 1.0;
    }
    mi.name = msg.status[i].name;
    mi.description = msg.status[i].message;
    mi.header.stamp = msg.header.stamp;
    for (int j=0; j<msg.status[i].values.size(); j++) {
      monitoring_msgs::KeyValue kv;
      kv.key = msg.status[i].values[j].key;
      kv.value = msg.status[i].values[j].value;
      kv.errorlevel = errorlevel;
      mi.values.push_back(kv);
    }
    ma.info.push_back(mi);
  }

}

void DiagnosticsToMonitoring::publish() {
  ma.header.stamp = ros::Time::now();
  monitor_pub.publish(ma);
  monitoring_msgs::MonitoringArray newMa;
  ma = newMa;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "diagnostics_to_monitoring");
	ros::NodeHandle n("~");


	float freq = 1;
	if (!n.getParam("frequency", freq)) {
		ROS_WARN("No frequency supplied. Working with %f Hz.", freq);
	}
	ros::Rate loop_rate(freq);
	DiagnosticsToMonitoring DTM(n);

	while(ros::ok()) {
		DTM.publish();

		ros::spinOnce();
		loop_rate.sleep();
	}


}
