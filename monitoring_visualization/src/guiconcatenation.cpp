/*
 * guiconcatenation.cpp
 *
 *  Created on: Jan 5, 2018
 *      Author: matthias
 */

#include "ros_monitoring/guiconcatenation.h"

GuiConcatenation::GuiConcatenation(ros::NodeHandle& n) {
	monitor_sub = n.subscribe("/monitoring", 1000,
			&GuiConcatenation::monitor_cb, this);
	error_sub = n.subscribe("/monitoring/errors", 1000,
			&GuiConcatenation::error_cb, this);

}

GuiConcatenation::~GuiConcatenation() {
	// TODO Auto-generated destructor stub
}

void GuiConcatenation::error_cb(ros_monitoring::Error er) {

	ros_monitoring::GuiInfo info;

	info.name = er.key;
	info.description = er.description;
	info.value = er.value;
	info.errorlevel = er.errorlevel;
	info.unit = er.unit;

	msg.infos.push_back(info);

}

void GuiConcatenation::monitor_cb(ros_monitoring::MonitoringArray ma) {
	for(int j=0; j< ma.info.size(); j++) {
		ros_monitoring::MonitoringInfo mi = ma.info[j];
		ros_monitoring::GuiInfo gi1;
		gi1.name = mi.name;
		gi1.value = "";

		float meanerror = 0;
		for (int i = 0; i < mi.values.size(); i++) {
			ros_monitoring::GuiInfo gi;
			char name[100];
			sprintf(name, "%s/%s", mi.name.c_str(), mi.values[i].key.c_str());
			gi.name = name;
			//gi.description = mi.values[i].;
			gi.value = mi.values[i].value;
			gi.errorlevel = mi.values[i].errorlevel;
			gi.unit = mi.values[i].unit;
			meanerror += mi.values[i].errorlevel;

			msg.infos.push_back(gi);
		}
		meanerror = meanerror / mi.values.size();
		gi1.errorlevel = meanerror;		//TODO ist die reihenfolge wichtig???
		msg.infos.push_back(gi1);
	}

}

ros_monitoring::Gui GuiConcatenation::getMsg() {

	ros_monitoring::Gui ret = msg;
	ros_monitoring::Gui newMsg;
	newMsg.name = "Test";
	msg = newMsg;
	float maxError = 0.0;
	for (int i = 0; i < ret.infos.size(); i++) {
		if (maxError < ret.infos[i].errorlevel) {
			maxError = ret.infos[i].errorlevel;
		}
	}
	ret.errorlevel = maxError;
	return ret;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "gui_msg_concatenation");
	ros::NodeHandle n("~");
	ros::Publisher gui_pub = n.advertise<ros_monitoring::Gui>("/monitoring/gui",
			10);

	float freq = 1;
	if (!n.getParam("frequency", freq)) {
		ROS_WARN("No frequency supplied. Working with %f Hz.", freq);
	}
	ros::Rate loop_rate(freq);
	GuiConcatenation gc(n);

	while (ros::ok()) {
		gui_pub.publish(gc.getMsg());

		ros::spinOnce();
		loop_rate.sleep();
	}

}
