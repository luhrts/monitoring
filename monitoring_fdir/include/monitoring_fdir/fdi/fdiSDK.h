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

#ifndef SRC_FDIR_FDISDK_H_
#define SRC_FDIR_FDISDK_H_

#include "ros/ros.h"
#include <queue>
#include "monitoring_msgs/MonitoringArray.h"
#include "monitoring_msgs/Error.h"
#include "string"
#include "configinterface.h"
#include "std_validator/max.h"
#include "std_validator/min.h"
#include "std_validator/between.h"
#include "std_validator/nodeavailable.h"



/**
 * This SDK allows you to setup a Fault detection and identification system for your Robot.
 * Use the Register function to setup your own or pre-developt validators.
 */
class FdiSDK
{
public:
  FdiSDK(ros::NodeHandle& n);
  virtual ~FdiSDK();

  /**
   * @brief registerFDIObject is used to register validators to check for errors in the system
   * @param object is the validator
   * @param msg is the name of the message it wants to listen to
   */
  void registerFDIObject(ConfigInterface* object, std::string msg);

private:
  /**
   * @brief monitorCallback this callback will automaticly buffer the messages. Which will be handled in the checkforFDI function.
   * @param ma
   */
  void monitorCallback(monitoring_msgs::MonitoringArray ma);
  ros::Subscriber sub;  ///< subscribes to monitoring for all monitored data
  ros::Publisher pub;   ///< publisher for error msgs

  std::map<std::string, std::vector<ConfigInterface *> > fdiConfigList; ///< a map to assign incoming msgs to the registered validators
};

#endif /* SRC_FDIR_FDISDK_H_ */
