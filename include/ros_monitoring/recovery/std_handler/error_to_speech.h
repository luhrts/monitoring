#ifndef SRC_RECOVERY_STD_HANDLER_ERROR_TO_SPEECH_H_
#define SRC_RECOVERY_STD_HANDLER_ERROR_TO_SPEECH_H_

#include "../errorhandlerinterface.h"

class ErrorToSpeech: public ErrorHandlerInterface {
public:
	ErrorToSpeech();
	virtual ~ErrorToSpeech();

	void checkError(ros_monitoring::Error msg);

private:
	std::map<std::string, ros::Time> lastSaid;

	bool checkSaid(std::string key);

};

#endif /* SRC_RECOVERY_STD_HANDLER_ERROR_TO_SPEECH_H_ */
