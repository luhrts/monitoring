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
        if (ma.info[j].values.size() == 0)
        {
            continue;
        }

        std::string output;
        
        monitoring_msgs::MonitoringInfo mi = ma.info[j];
        monitoring_msgs::GuiInfo gi1;
        gi1.name = mi.name;
        gi1.value = "";
        
        output += mi.name + ": [";

        float meanerror = 0;
        for (int i = 0; i < mi.values.size(); i++) {
                        monitoring_msgs::GuiInfo gi;
                        char name[1000];
            sprintf(name, "%s/%s", mi.name.c_str(), mi.values[i].key.c_str());

            gi.name = name;
            //gi.description = mi.values[i].;

            gi.value = mi.values[i].value;
            gi.errorlevel = mi.values[i].errorlevel;
            //printf("Errorlevel: %d", mi.values[i].errorlevel); // TODO Errorlevel for Testtool
            gi.unit = mi.values[i].unit;                       // Params: name, value, unit, errorlevel
            meanerror += mi.values[i].errorlevel;
            suit_unit(gi.value, gi.unit);
            msg.infos.push_back(gi);

            output += "(" + mi.values[i].value + mi.values[i].unit + ")";
        }
        output += "]";
        ROS_INFO(output.c_str());
        if(mi.values.size()!=0) {
            meanerror = meanerror / mi.values.size();
        }
    gi1.errorlevel = meanerror;
    gi1.unit = "";
    msg.infos.push_back(gi1);
    // TODO Implement ring-buff with monitoring msgs for Testtool

    }
}
void GuiConcatenation::suit_unit(std::string& value, std::string& unit){
//    ROS_INFO("v: %s, u: %s", value.c_str(), unit.c_str());
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
        std::vector<std::string>::iterator iter=std::find(Size_unit.begin(),Size_unit.end(),unit);

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
      }
      value = double_to_string(double_value);

    }
    if(std::count(Time_unit.begin(),Time_unit.end(),unit) != 0){
        std::string form_time = "";
       if(double_value > 31536000)//more than one year -> linux time
          {
          time_t unix_time;
          unix_time = double_value;
          form_time = asctime(localtime(&unix_time));
          form_time.erase(form_time.size()-1, form_time.size());    // removes last element since asctime adds "\n" ad new line
          value = form_time;
          unit = "";
       }
       else{
//           while(double_value >60){
//             std::vector<std::string>::iterator iter=find(Time_unit.begin(),Time_unit.end(),unit);
//             try
//             {
//                 iter +=1;
//                 unit = *iter;
//                 double_value = floor(double_value/60);
//                 double remainder = fmod(double_value , 60);
//                 form_time = double_to_string(remainder)+unit+form_time;
//             }
//             catch (std::exception& e)
//             {
//             ROS_WARN("Too long time ,cannot transfer unit ");
//             }
//           value = form_time;
//           unit = "";
//           }

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
    std::string freq_unit[] = {"Hz","kHz","MHz","GHz"};
    std::string size_unit[] = {"byte","kB","MB","GB","TB"};
    std::string time_unit[] = {"ms","sec","min","h"};

    Freq_unit.insert(Freq_unit.begin(),freq_unit,freq_unit+4);
    Size_unit.insert(Size_unit.begin(),size_unit,size_unit+5);
    Time_unit.insert(Time_unit.begin(),time_unit,time_unit+4);


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

  double freq = 1;

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
