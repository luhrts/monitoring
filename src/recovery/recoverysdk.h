#ifndef RECOVERYSDK_H
#define RECOVERYSDK_H

#include "ros_monitoring/Error.h"
#include "errorhandlerinterface.h"
#include <queue>

class RecoverySDK
{
public:
  RecoverySDK(ros::NodeHandle& n);

  void registerErrorHandling(ErrorHandlerInterface* errorHandler, std::string msg);
  void checkErrors();

private:
  void errorCallback(ros_monitoring::Error error);

  std::queue<ros_monitoring::Error> msgBuffer;
  ros::Subscriber sub;

  std::map<std::string, std::vector<ErrorHandlerInterface *>> recoveryHandler;
};

#endif // RECOVERYSDK_H
