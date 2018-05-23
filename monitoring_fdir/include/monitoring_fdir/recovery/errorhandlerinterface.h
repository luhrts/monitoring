#ifndef ERRORHANDLERINTERFACE_H
#define ERRORHANDLERINTERFACE_H

#include "ros/ros.h"
#include "monitoring_msgs/Error.h"

/**
 * @brief The ErrorHandlerInterface class is a Interface for error handler. this allows to create classes that react to incoming errors
 */
class ErrorHandlerInterface
{
public:
  ErrorHandlerInterface();
  virtual ~ErrorHandlerInterface();
  /**
   * @brief checkError this function is called when new error msgs are available for the registered handler
   * @param msg incoming error msg
   */
  virtual void checkError(monitoring_msgs::Error msg)=0;

private:

};

#endif // ERRORHANDLERINTERFACE_H
