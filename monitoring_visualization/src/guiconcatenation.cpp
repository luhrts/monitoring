/*
 * guiconcatenation.cpp
 *
 *  Created on: Jan 5, 2018
 *      Author: matthias
 */

#include "monitoring_visualization/guiconcatenation.h"

GuiConcatenation::GuiConcatenation(ros::NodeHandle& n,int mode) {
	monitor_sub = n.subscribe("/monitoring", 1000,
                        &GuiConcatenation::monitor_cb, this);
	error_sub = n.subscribe("/monitoring/errors", 1000,
			&GuiConcatenation::error_cb, this);
        MODE=mode;

}

GuiConcatenation::~GuiConcatenation() {
	// TODO Auto-generated destructor stub
}

void GuiConcatenation::error_cb(monitoring_msgs::Error er) {

  monitoring_msgs::GuiInfo info;

	info.name = er.key;
	info.description = er.description;
	info.value = er.value;
	info.errorlevel = er.errorlevel;
	info.unit = er.unit;

	msg.infos.push_back(info);

}

void GuiConcatenation::monitor_cb(monitoring_msgs::MonitoringArray ma) {
	for(int j=0; j< ma.info.size(); j++) {
    monitoring_msgs::MonitoringInfo mi = ma.info[j];
    monitoring_msgs::GuiInfo gi1;
		gi1.name = mi.name;
		gi1.value = "";

		float meanerror = 0;
		for (int i = 0; i < mi.values.size(); i++) {
                        monitoring_msgs::GuiInfo gi;
                        char name[1000];
			sprintf(name, "%s/%s", mi.name.c_str(), mi.values[i].key.c_str());
                        if(MODE == 1){
			gi.name = name;
			//gi.description = mi.values[i].;
			gi.value = mi.values[i].value;
			gi.errorlevel = mi.values[i].errorlevel;
			gi.unit = mi.values[i].unit;
                        meanerror += mi.values[i].errorlevel;

			msg.infos.push_back(gi);
                        }
                        else if(MODE == 2){
                           if (IS.count(name)==0){
                               IS[name]=info_Startegy();
                               IS[name].name=name;
                               IS[name].max= mi.values[i].value;
                               IS[name].min= mi.values[i].value;
                               IS[name].first= mi.values[i].value;
                               IS[name].last= mi.values[i].value;
                               }
                           else{
                               double value_new=atof(mi.values[i].value.c_str());
                               double value_old=atof(IS[name].max.c_str());

                               if(value_old < value_new)
                               {
                                IS[name].max= mi.values[i].value;

                               }
                               value_old=atof(IS[name].min.c_str());
                               if(value_old > value_new)
                               {
                                IS[name].min= mi.values[i].value;

                               }

                               IS[name].last = mi.values[i].value;
                               }
                           char NAME[1000];

                           strcpy(NAME,name);
                           gi.name = strcat(NAME,"/max");
                           gi.value = IS[name].max;
                           gi.errorlevel = mi.values[i].errorlevel;
                           gi.unit = mi.values[i].unit;
                           msg.infos.push_back(gi);

                           strcpy(NAME,name);
                           gi.name = strcat(NAME,"/min");
                           gi.value = IS[name].min;
                           gi.errorlevel = mi.values[i].errorlevel;
                           gi.unit = mi.values[i].unit;
                           msg.infos.push_back(gi);

                           strcpy(NAME,name);
                           gi.name = strcat(NAME,"/first");
                           gi.value = IS[name].first;
                           gi.errorlevel = mi.values[i].errorlevel;
                           gi.unit = mi.values[i].unit;
                           msg.infos.push_back(gi);

                           strcpy(NAME,name);
                           gi.name = strcat(NAME,"/last");
                           gi.value = IS[name].last;
                           gi.errorlevel = mi.values[i].errorlevel;
                           gi.unit = mi.values[i].unit;
                           msg.infos.push_back(gi);

                           meanerror += mi.values[i].errorlevel;


                        }

		}
		if(mi.values.size()!=0) {
			meanerror = meanerror / mi.values.size();
		}
    gi1.errorlevel = meanerror;		//TODO ist die reihenfolge wichtig???
    gi1.unit = "";
    msg.infos.push_back(gi1);
	}

}

monitoring_msgs::Gui GuiConcatenation::getMsg() {

  monitoring_msgs::Gui ret = msg;
  monitoring_msgs::Gui newMsg;
  newMsg.name = "GUI";
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
  ros::Publisher gui_pub = n.advertise<monitoring_msgs::Gui>("/monitoring/gui",
			10);

        float freq = 1;
        int mode= 1;
	if (!n.getParam("frequency", freq)) {
		ROS_WARN("No frequency supplied. Working with %f Hz.", freq);
	}
        if (!n.getParam("MODE", mode)) {
                ROS_WARN("No Mode supplied. Working with default mode.");
        }
	ros::Rate loop_rate(freq);
        GuiConcatenation gc(n, mode);

	while (ros::ok()) {
		gui_pub.publish(gc.getMsg());

		ros::spinOnce();
		loop_rate.sleep();
	}

}
