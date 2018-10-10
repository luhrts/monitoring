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

#ifndef SRC_FDIR_CONFIGINTERFACE_H_
#define SRC_FDIR_CONFIGINTERFACE_H_

#include "ros/ros.h"
#include "monitoring_msgs/KeyValue.h"
#include "monitoring_msgs/Error.h"



/**
 * @brief The ConfigInterface class is a Interface for the implementation of fdi validators. This allows to create classes that can be used with the fdiSDK.
 */
class ConfigInterface
{
public:
  ConfigInterface(ros::Publisher& publisher);
  virtual ~ConfigInterface();

  /**
   * @brief check will be called by the fdiSDK to validate if the key-value pair is a error.
   * You need to check if this is so and publish an error with the function publishError
   * @param newMsg is the keyvalue that was registered for this validator
   */
  virtual void check(monitoring_msgs::KeyValue newMsg) = 0;

protected:
  ros::Publisher pub;
  /**
   * @brief publishError used to publish an occuring error
   * @param errormsg
   */
  void publishError(monitoring_msgs::Error errormsg);
  char hostname[30];
};

#endif /* SRC_FDIR_CONFIGINTERFACE_H_ */
