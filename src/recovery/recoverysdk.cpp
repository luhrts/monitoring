#include "recoverysdk.h"

RecoverySDK::RecoverySDK(ros::NodeHandle& n)
{
  sub = n.subscribe("/monitoring/errors", 100, &RecoverySDK::errorCallback, this);
}

void RecoverySDK::registerErrorHandling(ErrorHandlerInterface* errorHandler, std::string msg) {
  recoveryHandler[msg].push_back(errorHandler);
}

void RecoverySDK::checkErrors() {
  while (!msgBuffer.empty())
  {
    ros_monitoring::Error error = msgBuffer.front();
    if (!(recoveryHandler.find(error.key) == recoveryHandler.end()))
    {
      std::vector<ErrorHandlerInterface *> recoveryHandlerList = recoveryHandler[error.key]; //get the list with objects that are registered on this message
      for(int i=0; i<recoveryHandlerList.size(); i++) {
        recoveryHandlerList[i]->checkError(error);
      }
    }
    msgBuffer.pop();
  }
}

void RecoverySDK::errorCallback(ros_monitoring::Error error) {
  msgBuffer.push(error);
}

