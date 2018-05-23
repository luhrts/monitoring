#include "monitoring_fdir/recovery/recoverysdk.h"

RecoverySDK::RecoverySDK(ros::NodeHandle& n) {
	sub = n.subscribe("/monitoring/errors", 10000, &RecoverySDK::errorCallback,
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
 * fills the msgbuffer with incoming error msg.
 */
void RecoverySDK::errorCallback(monitoring_msgs::Error error) {
	if (!(recoveryHandler.find(error.key) == recoveryHandler.end())) {
		std::vector<ErrorHandlerInterface *> recoveryHandlerList =
				recoveryHandler[error.key]; //get the list with objects that are registered on this message
		for (int i = 0; i < recoveryHandlerList.size(); i++) {
			recoveryHandlerList[i]->checkError(error);
		}
	}
}

