#include "ros_monitoring/recovery/std_handler/error_to_speech.h"

ErrorToSpeech::ErrorToSpeech() {


}

ErrorToSpeech::~ErrorToSpeech() {
	// TODO Auto-generated destructor stub
}

void ErrorToSpeech::checkError(ros_monitoring::Error msg)
{

	if(checkSaid(msg.key)) {
		char command[200];
		sprintf(command, "espeak \"%s\"", msg.key.c_str());
	  system(command);
	}

}

bool ErrorToSpeech::checkSaid(std::string saing) {
	ros::Time now = ros::Time::now();
	std::map<std::string,ros::Time>::iterator it = lastSaid.find(saing);

	if(it != lastSaid.end())
	{
	   //element found;
		 ros::Duration dT = now - it->second;
	   if(dT.toSec()<10){
			 return false;
		 }
	}
	lastSaid[saing] = now;
	return true;
}
