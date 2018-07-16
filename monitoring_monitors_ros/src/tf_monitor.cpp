/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Wim Meeussen */


#include "monitoring_monitors_ros/tf_monitor.h"

using namespace tf;
using namespace ros;
using namespace std;

TFMonitor::TFMonitor()
{
    subscriber_tf_ = node_.subscribe<tf::tfMessage>("tf", 100, boost::bind(&TFMonitor::callback, this, _1));
    subscriber_tf_static_ = node_.subscribe<tf::tfMessage>("tf_static", 100, boost::bind(&TFMonitor::static_callback, this, _1));
}

void TFMonitor::callback(const ros::MessageEvent<tf::tfMessage const>& msg_evt)
{
    const tf::tfMessage& message = *(msg_evt.getConstMessage());
    std::string authority = msg_evt.getPublisherName(); // lookup the authority
    process_callback(message, authority, false);
}

void TFMonitor::static_callback(const ros::MessageEvent<tf::tfMessage const>& msg_evt)
{
    const tf::tfMessage& message = *(msg_evt.getConstMessage());
    std::string authority = msg_evt.getPublisherName() + std::string("(static)"); // lookup the authority
    process_callback(message, authority, true);
}


void TFMonitor::process_callback(const tf::tfMessage& message, const std::string & authority, bool is_static)
{
    double average_offset = 0;
    boost::mutex::scoped_lock my_lock(map_lock_);
    for (unsigned int i = 0; i < message.transforms.size(); i++)
    {
        frame_authority_map[message.transforms[i].child_frame_id] = authority;

        double offset;
        if (is_static)
        {
            offset = 0.0;
        }
        else
        {
            offset = (ros::Time::now() - message.transforms[i].header.stamp).toSec();
        }
        average_offset  += offset;

        std::map<std::string, std::vector<double> >::iterator it = delay_map.find(message.transforms[i].child_frame_id);
        if (it == delay_map.end())
        {
            delay_map[message.transforms[i].child_frame_id] = std::vector<double>(1,offset);
        }
        else
        {
            it->second.push_back(offset);
            if (it->second.size() > 1000)
                it->second.erase(it->second.begin());
        }

    }

    average_offset /= max((size_t) 1, message.transforms.size());

    //create the authority log
    std::map<std::string, std::vector<double> >::iterator it2 = authority_map.find(authority);
    if (it2 == authority_map.end())
    {
        authority_map[authority] = std::vector<double>(1,average_offset);
    }
    else
    {
        it2->second.push_back(average_offset);
        if (it2->second.size() > 1000)
            it2->second.erase(it2->second.begin());
    }

    //create the authority frequency log
    std::map<std::string, std::vector<double> >::iterator it3 = authority_frequency_map.find(authority);
    if (it3 == authority_frequency_map.end())
    {
        authority_frequency_map[authority] = std::vector<double>(1,ros::Time::now().toSec());
    }
    else
    {
        it3->second.push_back(ros::Time::now().toSec());
        if (it3->second.size() > 1000)
            it3->second.erase(it3->second.begin());
    }

}

std::string TFMonitor::outputFrameInfo(const std::map<std::string, std::vector<double> >::iterator& it, const std::string& frame_authority)
{
    std::stringstream ss;
    double average_delay = 0;
    double max_delay = 0;
    for (unsigned int i = 0; i < it->second.size(); i++)
    {
        average_delay += it->second[i];
        max_delay = std::max(max_delay, it->second[i]);
    }
    average_delay /= it->second.size();
    ss << "Frame: " << it->first <<" published by "<< frame_authority << " Average Delay: " << average_delay << " Max Delay: " << max_delay << std::endl;
    return ss.str();
}

void TFMonitor::spin()
{
    // create tf listener
    double max_diff = 0;
    double avg_diff = 0;
    double lowpass = 0.01;
    unsigned int counter = 0;

    while (node_.ok()){
        tf::StampedTransform tmp;
        counter++;
        //    printf("looping %d\n", counter);
        Duration(0.01).sleep();
        if (counter > 20){
            counter = 0;

            std::cout <<std::endl<< std::endl<< std::endl<< "RESULTS: for all Frames" <<std::endl;
            boost::mutex::scoped_lock lock(map_lock_);
            std::cout <<std::endl << "Frames:" <<std::endl;
            std::map<std::string, std::vector<double> >::iterator it = delay_map.begin();
            for ( ; it != delay_map.end() ; ++it)
            {
               cout << outputFrameInfo(it, frame_authority_map[it->first]);
            }
            std::cerr <<std::endl<< "All Broadcasters:" << std::endl;
            std::map<std::string, std::vector<double> >::iterator it1 = authority_map.begin();
            std::map<std::string, std::vector<double> >::iterator it2 = authority_frequency_map.begin();
            for ( ; it1 != authority_map.end() ; ++it1, ++it2)
            {
                double average_delay = 0;
                double max_delay = 0;
                for (unsigned int i = 0; i < it1->second.size(); i++)
                {
                    average_delay += it1->second[i];
                    max_delay = std::max(max_delay, it1->second[i]);
                }
                average_delay /= it1->second.size();
                double frequency_out = (double)(it2->second.size())/std::max(0.00000001, (it2->second.back() - it2->second.front()));
                //cout << "output" <<&(*it2) <<" " << it2->second.back() <<" " << it2->second.front() <<" " << std::max((size_t)1, it2->second.size()) << " " << frequency_out << endl;
                cout << "Node: " <<it1->first << " " << frequency_out <<" Hz, Average Delay: " << average_delay << " Max Delay: " << max_delay << std::endl;
            }

        }
    }

}

int main(int argc, char ** argv)
{
    //Initialize ROS
    init(argc, argv, "tf_monitor");

    std::string searched_param;
    std::string tf_prefix;
    ros::NodeHandle local_nh("~");
    local_nh.searchParam("tf_prefix", searched_param);
    local_nh.getParam(searched_param, tf_prefix);

    //Make sure we don't start before recieving time when in simtime
    int iterations = 0;
    while (ros::Time::now() == ros::Time())
    {
        if (++iterations > 10)
        {
            ROS_INFO("tf_monitor waiting for time to be published");
            iterations = 0;
        }
        ros::WallDuration(0.1).sleep();
    }

    ros::NodeHandle nh;
    boost::thread spinner( boost::bind( &ros::spin ));
    TFMonitor monitor;
    monitor.spin();
    spinner.join();
    return 0;

}



//tf::TransformListener* listener;




//void updateFrames(std::vector<std::string> tf_frames){


//    for(std::string frame : tf_frames){
//        bool new_frame = false;

//        //Check if Frame is knowkn
//        if(transforms_.count(frame) == 0){
//            transforms_[frame] = Transform();
//            transforms_[frame].frame = frame;

//            ROS_INFO("New Frame: %s", frame.c_str());
//            new_frame = true;
//        }


//        //Check Parent
//        std::string parent;
//        if (!listener->getParent(frame, ros::Time(0), parent)){
//            ROS_DEBUG("TF_Monitor: getParent failed for: %s", frame.c_str());
//            transforms_[frame].has_parent = false;

//        } else {
//            transforms_[frame].has_parent = true;
//        }

//        //TF Parent Change =>
//        if(transforms_[frame].parent != parent && !new_frame){
//            ROS_WARN("TF_Monitor: getParent missmatch frame: %s, old_parent: %s new_parent: %s", frame.c_str(), transforms_[frame].parent.c_str(), parent.c_str());
//        }

//        transforms_[frame].parent = parent;


//        // If has parent get Transform data
//        if (transforms_[frame].has_parent) {
//            //Stamped Transform compare
//            tf::StampedTransform current_transform;
//            listener->lookupTransform(frame, parent, ros::Time(0), current_transform);

//            //calculate deltas
//            if(!new_frame){
//                double dx, dy, dz, droll, dpitch, dyaw;
//                ros::Duration dTime;

//                dx = current_transform.getOrigin().getX() - transforms_[frame].last_transform.getOrigin().getX();
//                dy = current_transform.getOrigin().getY() - transforms_[frame].last_transform.getOrigin().getY();
//                dz = current_transform.getOrigin().getZ() - transforms_[frame].last_transform.getOrigin().getZ();

//                double current_roll, current_pitch, current_yaw;
//                tf::Matrix3x3(current_transform.getRotation()).getRPY(current_roll, current_pitch, current_yaw);

//                double last_roll, last_pitch, last_yaw;
//                tf::Matrix3x3(transforms_[frame].last_transform.getRotation()).getRPY(current_roll, current_pitch, current_yaw);

//                droll = current_roll - last_roll;
//                dpitch = current_pitch - last_pitch;
//                dyaw = current_yaw - last_yaw;

//                dTime = current_transform.stamp_ - transforms_[frame].last_transform.stamp_;

//                ROS_INFO("%s -> %s:  \t xyz [%f, %f, %f] rpy [%f, %f, %f] dtime: %f s", parent.c_str(), frame.c_str(), dx, dy, dz, droll, dpitch, dyaw, dTime.toSec());

//            }
//            transforms_[frame].last_transform = current_transform;
//        }




//    }

//}



//int main(int argc, char *argv[]) {
//    ros::init(argc, argv, "tf_monitor");
//    ros::NodeHandle n("~");

//    listener = new tf::TransformListener();

//    ros::Rate rate(1.0);
//    while (n.ok()){
//        ROS_INFO("________________________________________________________________________________________________");
//        std::vector<std::string> tf_frames;
//        listener->getFrameStrings(tf_frames);
//        updateFrames(tf_frames);


//        rate.sleep();
//    }
//    return 0;
//}
