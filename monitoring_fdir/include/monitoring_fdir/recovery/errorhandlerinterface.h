#ifndef ERRORHANDLERINTERFACE_H
#define ERRORHANDLERINTERFACE_H

#include "ros/ros.h"
#include "monitoring_msgs/Error.h"

class ErrorHandlerInterface
{
public:
  ErrorHandlerInterface();
  virtual ~ErrorHandlerInterface();

  virtual void checkError(monitoring_msgs::Error msg)=0;

private:

};

#endif // ERRORHANDLERINTERFACE_H
