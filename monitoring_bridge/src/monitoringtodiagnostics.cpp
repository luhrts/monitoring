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

#include "monitoring_bridge/monitoringtodiagnostics.h"

MonitoringToDiagnostics::MonitoringToDiagnostics(ros::NodeHandle& n) {
	diag_pub = n.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics_agg", 1);
	monitor_sub = n.subscribe("/monitoring", 50, &MonitoringToDiagnostics::monitor_callback, this);

}

MonitoringToDiagnostics::~MonitoringToDiagnostics() {
	// TODO Auto-generated destructor stub
}

void MonitoringToDiagnostics::publish() {
	diag_pub.publish(da);
	diagnostic_msgs::DiagnosticArray newDA;
	da = newDA;

}
void MonitoringToDiagnostics::monitor_callback(monitoring_msgs::MonitoringArray msg) {
	for(int i=0; i<msg.info.size(); i++) {
		diagnostic_msgs::DiagnosticStatus ds;
		float meanError=0;
		ds.name = msg.info[i].name;
		ds.message = msg.info[i].description;

		for(int j=0; j<msg.info[i].values.size(); j++) {
			diagnostic_msgs::KeyValue kv;
			kv.key = msg.info[i].values[j].key;
			kv.value = msg.info[i].values[j].value;
			ds.values.push_back(kv);
			meanError += msg.info[i].values[j].errorlevel;
		}

		ds.level = (int) (meanError/msg.info[i].values.size()  * 2.9);
		da.status.push_back(ds);
	}


}

int main(int argc, char **argv) {

	ros::init(argc, argv, "monitoring_to_diagnostics");
	ros::NodeHandle n("~");


	float freq = 1;
	if (!n.getParam("frequency", freq)) {
		ROS_WARN("No frequency supplied. Working with %f Hz.", freq);
	}
	ros::Rate loop_rate(freq);
	MonitoringToDiagnostics MTD(n);

	while(ros::ok()) {
		MTD.publish();

		ros::spinOnce();
		loop_rate.sleep();
	}


}
