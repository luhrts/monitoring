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

#ifndef SRC_GUICONCATENATION_H_
#define SRC_GUICONCATENATION_H_

#include "ros/ros.h"
#include "monitoring_msgs/MonitoringArray.h"
#include "monitoring_msgs/Error.h"
#include "monitoring_msgs/Gui.h"
#include <time.h>

class GuiConcatenation {
public:
	GuiConcatenation(ros::NodeHandle& n);
	virtual ~GuiConcatenation();

  /**
   * @brief getMsg function to get the msg for publishing
   * @return msg type for aggregated monitors and error msgs
   */
  monitoring_msgs::Gui getMsg();
private:
  /**
   * @brief monitor_cb callback for monitoring msgs
   * @param ma
   */

  void suit_unit(std::string& value, std::string& unit);
  void monitor_cb(monitoring_msgs::MonitoringArray ma);
  std::string int_to_string(int int_num);
  std::string double_to_string(double double_num);
  std::vector<std::string> Freq_unit;
  std::vector<std::string> Size_unit;
  std::vector<std::string> Time_unit;
  /**
   * @brief error_cb callback for error msgs
   * @param er
   */
  void Init_Unit_Vector();
  void error_cb(monitoring_msgs::Error er);
  ros::Subscriber monitor_sub, error_sub; ///< subscribers for data collection
  monitoring_msgs::Gui msg;
};

#endif /* SRC_GUICONCATENATION_H_ */
