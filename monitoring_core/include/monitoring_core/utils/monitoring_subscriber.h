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
