#include "ros_monitoring/recovery/std_handler/error_to_speech.h"

ErrorToSpeech::ErrorToSpeech() {
	flite_init();
  v = register_cmu_us_kal(NULL);

}

ErrorToSpeech::~ErrorToSpeech() {
	// TODO Auto-generated destructor stub
}

void ErrorToSpeech::checkError(ros_monitoring::Error msg)
{
  flite_text_to_speech(msg.key.c_str(), v, "play");
}
