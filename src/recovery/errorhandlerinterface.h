#ifndef ERRORHANDLERINTERFACE_H
#define ERRORHANDLERINTERFACE_H

#include "ros/ros.h"
#include "ros_monitoring/Error.h"

class ErrorHandlerInterface
{
public:
  ErrorHandlerInterface();
  virtual ~ErrorHandlerInterface();

  virtual void checkError(ros_monitoring::Error msg)=0;

private:
  ros::Publisher pub;
};

#endif // ERRORHANDLERINTERFACE_H
