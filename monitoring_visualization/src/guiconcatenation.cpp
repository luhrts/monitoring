/*
 * guiconcatenation.cpp
 *
 *  Created on: Jan 5, 2018
 *      Author: matthias
 */

#include "monitoring_visualization/guiconcatenation.h"

GuiConcatenation::GuiConcatenation(ros::NodeHandle& n) {
	monitor_sub = n.subscribe("/monitoring", 1000,
                        &GuiConcatenation::monitor_cb, this);
	error_sub = n.subscribe("/monitoring/errors", 1000,
			&GuiConcatenation::error_cb, this);
        GuiConcatenation::Init_Unit_Vector();


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

			gi.name = name;
			//gi.description = mi.values[i].;

			gi.value = mi.values[i].value;
			gi.errorlevel = mi.values[i].errorlevel;
			gi.unit = mi.values[i].unit;
                        meanerror += mi.values[i].errorlevel;
                        suit_unit(gi.value, gi.unit);
			msg.infos.push_back(gi);


		}
		if(mi.values.size()!=0) {
			meanerror = meanerror / mi.values.size();
		}
    gi1.errorlevel = meanerror;		//TODO ist die reihenfolge wichtig???
    gi1.unit = "";
    msg.infos.push_back(gi1);
	}

}
void GuiConcatenation::suit_unit(std::string& value, std::string& unit){
    double double_value = atof(value.c_str());
    if(std::count(Freq_unit.begin(),Freq_unit.end(),unit) != 0){
      while(double_value >1000){
        std::vector<std::string>::iterator iter=find(Freq_unit.begin(),Freq_unit.end(),unit);
        try
        {
            iter +=1;
            unit = *iter;
            double_value = double_value/1000;
        }
        catch (std::exception& e)
        {
        ROS_WARN("Too large Frequency ,cannot transfer unit ");
        }
      }
      value = double_to_string(double_value);
    }
    if(std::count(Size_unit.begin(),Size_unit.end(),unit) != 0){
      while(double_value >1024){
        std::vector<std::string>::iterator iter=find(Freq_unit.begin(),Freq_unit.end(),unit);
        try
        {
            iter +=1;
            unit = *iter;
            double_value = double_value/1024;
        }
        catch (std::exception& e)
        {
        ROS_WARN("Too large Size ,cannot transfer unit ");
        }
       value = double_to_string(double_value);
      }
    }


}
std::string GuiConcatenation::int_to_string(int int_value){
    char char_value[1000];
    sprintf(char_value, "%d", int_value);
    std::string str = char_value;
    return str;

}
std::string GuiConcatenation::double_to_string(double double_value){
    char char_value[1000];
    sprintf(char_value, "%4.4f", double_value);
    std::string str = char_value;
    return str;

}
void GuiConcatenation::Init_Unit_Vector(){
    std::string freq_unit[] = {"Hz","KHz","MHz","GHz"};
    std::string size_unit[] = {"byte","KB","MB","GB"};

    Freq_unit.insert(Freq_unit.begin(),freq_unit,freq_unit+4);
    Size_unit.insert(Size_unit.begin(),size_unit,size_unit+4);

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
