#include "ros_monitoring/recovery/recoverysdk.h"

RecoverySDK::RecoverySDK(ros::NodeHandle& n) {
	sub = n.subscribe("/monitoring/errors", 100, &RecoverySDK::errorCallback,
			this);
}

/**
 * register the error handler that it gets called with the checkErrrors function
 */
void RecoverySDK::registerErrorHandler(ErrorHandlerInterface* errorHandler,
		std::string msg) {
	recoveryHandler[msg].push_back(errorHandler);
}

/**
 * Checks and Redirects all messages to the appropriate registered objects
 *
 * NOT IN USE
 */
void RecoverySDK::checkErrors() {
	while (!msgBuffer.empty()) {
		ros_monitoring::Error error = msgBuffer.front();
		if (!(recoveryHandler.find(error.key) == recoveryHandler.end())) {
			std::vector<ErrorHandlerInterface *> recoveryHandlerList =
					recoveryHandler[error.key]; //get the list with objects that are registered on this message
			for (int i = 0; i < recoveryHandlerList.size(); i++) {
				recoveryHandlerList[i]->checkError(error);
			}
		}
		msgBuffer.pop();
	}
}

/**
 * fills the msgbuffer with incoming error msg.
 */
void RecoverySDK::errorCallback(ros_monitoring::Error error) {
	if (!(recoveryHandler.find(error.key) == recoveryHandler.end())) {
		std::vector<ErrorHandlerInterface *> recoveryHandlerList =
				recoveryHandler[error.key]; //get the list with objects that are registered on this message
		for (int i = 0; i < recoveryHandlerList.size(); i++) {
			recoveryHandlerList[i]->checkError(error);
		}
	}
}

