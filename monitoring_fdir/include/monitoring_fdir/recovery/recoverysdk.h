/*
 *  Copyright (c) 2018, University of Hannover
 *                      Institute for Systems Engineering - RTS
 *                      Professor Bernardo Wagner
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
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
