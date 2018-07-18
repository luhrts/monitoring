#include "monitoring_monitors_ros/tf_monitor.h"

using namespace tf;
using namespace ros;
using namespace std;

TFMonitor::TFMonitor()
{
    ros::NodeHandle nh;
    monitor_ = new Monitor(nh, "tf_monitor");

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
    for (unsigned int i = 0; i < message.transforms.size(); i++)
    {
        bool new_frame = false;
        std::string frame = message.transforms[i].child_frame_id;
        std::string parent = message.transforms[i].header.frame_id;

        //Check if Frame is knowkn
        if(transforms_.count(frame) == 0){
            transforms_[frame] = TransformData();
            transforms_[frame].frame = frame;

            ROS_INFO("New Frame: %s", frame.c_str());
            new_frame = true;
        }

        //Static change
        if(transforms_[frame].is_static != is_static && !new_frame){
            ROS_WARN("TF_Monitor: static changed for frame: %s, old_parent: %d new_parent: %d", frame.c_str(), transforms_[frame].is_static, is_static);
            monitor_->addValue(parent+"/"+frame+"/static_changed", -1, "", 1.0);
        }
        transforms_[frame].is_static = is_static;

        //TF Parent Change =>
        if(transforms_[frame].parent != parent && !new_frame){
            ROS_WARN("TF_Monitor: parent changed for frame: %s, old_parent: %s new_parent: %s", frame.c_str(), transforms_[frame].parent.c_str(), parent.c_str());
            monitor_->addValue(parent+"/"+frame+"/parent_changed", -1, "", 1.0);

        }

        //Seperation detection
        if(new_frame){
           if(transforms_.count(transforms_[frame].parent)==0 ||transforms_[frame].parent !=base_parent_frame){
               //new base
               if(frame==base_parent_frame){
               base_parent_frame=transforms_[frame].parent;
               }
               else{
                 ROS_WARN("xxxxx");
                 monitor_->addValue();

               }

           }
        }

        while (transforms_[frame].parent!= parent && !new_frame) {
            if(transforms_[frame].parent==base_parent_frame){
                break;
            }
            else{

                if(transforms_.count(transforms_[active_frame_child].parent)==0){
                    ////Seperation
                    ROS_WARN("xxxxx");
                    monitor_->addValue();
                };

            }
        }
        transforms_[frame].parent = parent;

        //Autohrity change
        if(transforms_[frame].authority != authority && !new_frame){
            ROS_WARN("TF_Monitor: authority changed for frame: %s, old_authority: %s new_authority: %s", frame.c_str(), transforms_[frame].authority.c_str(), authority.c_str());
            monitor_->addValue(parent+"/"+frame+"/authority_changed", -1, "", 1.0);
        }
        transforms_[frame].authority = authority;



        //offset
        double offset;
        if (is_static)
        {
            offset = 0.0;
        }
        else
        {
            offset = (ros::Time::now() - message.transforms[i].header.stamp).toSec();
        }
        monitor_->addValue(parent+"/"+frame+"/timestamp_delay", offset, "s", 0.0);

        // If has parent get Transform data
        //Stamped Transform compare
        tf::StampedTransform current_transform;
        tf::transformStampedMsgToTF(message.transforms[i], current_transform);

        //calculate deltas
        if(!new_frame){
            double dx, dy, dz, droll, dpitch, dyaw;
            ros::Duration dTime;

            dx = current_transform.getOrigin().getX() - transforms_[frame].last_transform.getOrigin().getX();
            dy = current_transform.getOrigin().getY() - transforms_[frame].last_transform.getOrigin().getY();
            dz = current_transform.getOrigin().getZ() - transforms_[frame].last_transform.getOrigin().getZ();

            double current_roll, current_pitch, current_yaw;
            tf::Matrix3x3(current_transform.getRotation()).getRPY(current_roll, current_pitch, current_yaw);

            double last_roll, last_pitch, last_yaw;
            tf::Matrix3x3(transforms_[frame].last_transform.getRotation()).getRPY(current_roll, current_pitch, current_yaw);

            droll = current_roll - last_roll;
            dpitch = current_pitch - last_pitch;
            dyaw = current_yaw - last_yaw;

            dTime = current_transform.stamp_ - transforms_[frame].last_transform.stamp_;


            monitor_->addValue(parent+"/"+frame+"/dx", dx, "m", 0.0);
            monitor_->addValue(parent+"/"+frame+"/dy", dy, "m", 0.0);
            monitor_->addValue(parent+"/"+frame+"/dz", dz, "m", 0.0);

            monitor_->addValue(parent+"/"+frame+"/droll", droll, "degree", 0.0);
            monitor_->addValue(parent+"/"+frame+"/dpitch", dpitch, "degree", 0.0);
            monitor_->addValue(parent+"/"+frame+"/dyaw", dyaw, "degree", 0.0);

            monitor_->addValue(parent+"/"+frame+"/tdoa", dTime.toSec(), "s", 0.0);

            ROS_DEBUG("%s -> %s:  \t xyz [%f, %f, %f] rpy [%f, %f, %f] dtime: %f s", parent.c_str(), frame.c_str(), dx, dy, dz, droll, dpitch, dyaw, dTime.toSec());

        }
        transforms_[frame].last_transform = current_transform;


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

    ROS_INFO("init done");
    TFMonitor monitor;



    ros::spin();

    return 0;

}


