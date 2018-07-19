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
  bool NeedToCheckSperation;

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

#endif // TF_MONITOR_H
