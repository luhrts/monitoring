/*
 * userfaultdetection.cpp
 *
 *  Created on: Dec 4, 2017
 *      Author: matthias
 */

#include "userfaultdetection.h"

UserFaultDetection::UserFaultDetection(ros::NodeHandle& n)
{
  sub = n.subscribe("/monitoring/all", 1000, &UserFaultDetection::monitorCallback, this);
  pub = n.advertise<ros_monitoring::Error>("/monitoring/errors", 1000);

}

UserFaultDetection::~UserFaultDetection()
{
  // TODO Auto-generated destructor stub
}

void UserFaultDetection::monitorCallback(ros_monitoring::MonitoringInfo mi)
{
  ROS_INFO("CALLBACK");
  for (int i = 0; i < mi.values.size(); i++)
  {
    msgBuffer.push(mi.values[i]);
  }
}

void UserFaultDetection::load_config(ros::NodeHandle& n)
{
  std::vector < std::string > faultdetection;
  if (n.getParam("faultdetection", faultdetection))
  {
  }

  for (int i = 0; i < faultdetection.size(); i++)
  {
    ROS_INFO("%s", faultdetection[i].c_str());
    fdiconfig newConfig;
    if (!n.getParam(faultdetection[i] + "/operator", newConfig.op))
    {
      ROS_ERROR("The supplied Type \"%s\" is not available!", faultdetection[i].c_str());
      continue;
    }

    if (!n.getParam(faultdetection[i] + "/value", newConfig.value))
    {
      ROS_ERROR("value error");
    }
    if (!n.getParam(faultdetection[i] + "/error", newConfig.error))
    {
      ROS_ERROR("error error");
    }
    if (!n.getParam(faultdetection[i] + "/errorlevel", newConfig.errorlevel))
    {
      ROS_ERROR("errorlevel error");
    }
    std::string message;
    if (!n.getParam(faultdetection[i] + "/message", message))
    {
      ROS_ERROR("message error");
    }

    user_config[message] = newConfig;
  }

}

void UserFaultDetection::fdi()
{
  ROS_INFO("FDI:");
  while (!msgBuffer.empty())
  {
    ros_monitoring::KeyValue kv = msgBuffer.front();
//    ROS_INFO("Key: %s", kv.key.c_str());
    if (!(user_config.find(kv.key) == user_config.end()))
    {
      //TODO check the config and publish msg if error
      fdiconfig config = user_config[kv.key];
      //ROS_INFO("For message: \'%s\' following config", kv.key.c_str());
      std::string::size_type sz;
      float value = std::stof (kv.value,&sz);
      if(value>config.value) {
        ros_monitoring::Error er;
        er.key = config.error;
        er.value = kv.value;
        er.level = config.errorlevel;
        char hostname[30];
        getHostname(hostname);
        er.pc.Hostname = hostname;
        pub.publish(er);
      }

    }
    else {
//      ROS_INFO("No Key found");
    }

    msgBuffer.pop();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "user_fdi");
  ros::NodeHandle n("~");

  float freq = 1;
  if (!n.getParam("frequency", freq))
  {
    ROS_WARN("No frequency supplied. Working with %d Hz.", freq);
  }

  UserFaultDetection userFDI(n);

  ros::Rate loop_rate(freq);

  userFDI.load_config(n);
  while (ros::ok())
  {
    userFDI.fdi();
    loop_rate.sleep();
    ros::spinOnce();
  }

}

