#include "monitoring_fdir/recovery/std_handler/error_to_speech.h"
#include "std_msgs/String.h"

ErrorToSpeech::ErrorToSpeech(ros::NodeHandle &n) {
	voice_pub = n.advertise<std_msgs::String>("/voice_output", 1);


}

ErrorToSpeech::~ErrorToSpeech() {
	// TODO Auto-generated destructor stub
}

void ErrorToSpeech::checkError(monitoring_msgs::Error msg)
{

	if(checkSaid(msg.key)) {
		std_msgs::String voice;
		voice.data = msg.key;
		voice_pub.publish(voice);

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
