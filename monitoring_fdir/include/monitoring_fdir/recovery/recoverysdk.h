#ifndef RECOVERYSDK_H
#define RECOVERYSDK_H

#include "monitoring_msgs/Error.h"
#include "errorhandlerinterface.h"
#include <queue>

#include "std_handler/restartnodehandler.h"
#include "std_handler/stoplaunchfile.h"
#include "std_handler/outputerrormessage.h"
#include "std_handler/error_to_speech.h"

/**
 * @brief The RecoverySDK class is a sdk to handle errors
 */
class RecoverySDK
{
public:
  RecoverySDK(ros::NodeHandle& n);
  /**
   * @brief registerErrorHandler to register Error Handler with a error msg name
   * @param errorHandler the handler that will be registered
   * @param msg the corresponding msg name
   */
  void registerErrorHandler(ErrorHandlerInterface* errorHandler, std::string msg);

private:
  /**
   * @brief errorCallback callback function for the ros msgs
   * @param error error msg content
   */
  void errorCallback(monitoring_msgs::Error error);

  ros::Subscriber sub;    ///< subscribes to /monitoring/errors

  std::map<std::string, std::vector<ErrorHandlerInterface *> > recoveryHandler;  ///< contains the registered handler with the corresponding msgs
};

#endif // RECOVERYSDK_H
