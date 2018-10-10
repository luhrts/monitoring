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


#ifndef MONITORING_SUBSCRIBER_H
#define MONITORING_SUBSCRIBER_H

#include <ros/ros.h>
#include <monitoring_msgs/MonitoringArray.h>

class MonitoringSubscriber
{

public:

    ros::Subscriber subscriber_;
    std::vector<monitoring_msgs::MonitoringArray> receivedArrays_;

    MonitoringSubscriber (ros::NodeHandle& n)
    {
        subscriber_ = n.subscribe("/monitoring", 1, &MonitoringSubscriber::monitoring_callback,this);
    }

    void monitoring_callback(const monitoring_msgs::MonitoringArray& msg)
    {
        receivedArrays_.push_back(msg);
    }

    // Make sure the messages are received
    void waitAndSpin(ros::Duration dur = ros::Duration(0.1), int iterations = 50){
        int count = 0;
        while(ros::ok() && count++ < iterations){
            ros::spinOnce();
            dur.sleep();
        }
    }

    bool hasReceivedKey(std::string key)
    {
        for (monitoring_msgs::MonitoringArray array : receivedArrays_){
            for (monitoring_msgs::MonitoringInfo info : array.info){
                for (monitoring_msgs::KeyValue keyValue: info.values){
                    std::size_t found = keyValue.key.rfind("/")+1;
                    std::string last_key = keyValue.key.substr(found);
                    if (last_key == key){
                        return true;
                    }
                }
            }
        }
        return false;
    }

};






#endif // MONITORING_SUBSCRIBER_H
