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
#ifndef TF_MONITOR_H
#define TF_MONITOR_H

#include "tf/tf.h"
#include "tf/transform_listener.h"
#include <string>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include "ros/ros.h"

#include "monitoring_core/monitor.h"

using namespace tf;
using namespace ros;
using namespace std;




class TFMonitor
{
public:

  TFMonitor();

  void callback(const ros::MessageEvent<tf::tfMessage const>& msg_evt);

  void static_callback(const ros::MessageEvent<tf::tfMessage const>& msg_evt);

  void process_callback(const tf::tfMessage& message, const std::string & authority, bool is_static);
  

private:

  ros::NodeHandle node_;
  ros::Subscriber subscriber_tf_;
  ros::Subscriber subscriber_tf_static_;

  TransformListener tf_;
  Monitor* monitor_;

  tf::tfMessage message_;

  boost::mutex map_lock_;

  int last_transform_size;
  bool NeedToCheckSperationAndLoop;
  int wait_times_offset=0;



  struct TransformData {
      std::string frame;
      std::string parent;
      
      bool is_static;

      tf::StampedTransform last_transform;

      std::string authority;
  };
  std::string base_parent_frame="";
  std::map<std::string, TransformData> transforms_;
  

};

  AggregationStrategies aggregation;
  int monitor_mode = 4;

#endif // TF_MONITOR_H
