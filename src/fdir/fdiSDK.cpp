/*
 * FdiSDK.cpp
 *
 *  Created on: Dec 4, 2017
 *      Author: matthias
 */


#include "fdiSDK.h"

FdiSDK::FdiSDK(ros::NodeHandle& n)
{
  sub = n.subscribe("/monitoring/all", 1000, &FdiSDK::monitorCallback, this);
  pub = n.advertise<ros_monitoring::Error>("/monitoring/errors", 1000);
}

FdiSDK::~FdiSDK()
{
  // TODO Auto-generated destructor stub
}

void FdiSDK::monitorCallback(ros_monitoring::MonitoringInfo mi)
{
  ROS_INFO("CALLBACK");
  for (int i = 0; i < mi.values.size(); i++)
  {
    msgBuffer.push(mi.values[i]);
  }
}

void FdiSDK::load_config(ros::NodeHandle& n)
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
    //fdiConfigList[message] = ;
  }
}

void FdiSDK::registerFDIObject(ConfigInterface object, std::string msg) {

  fdiConfigList[msg].push_back(object);

}


void FdiSDK::fdi()
{
  ROS_INFO("FDI:");
  while (!msgBuffer.empty())
  {
    ros_monitoring::KeyValue kv = msgBuffer.front();
//    ROS_INFO("Key: %s", kv.key.c_str());
    if (!(fdiConfigList.find(kv.key) == fdiConfigList.end()))
    {
      //TODO check the config and publish msg if error
      std::vector<ConfigInterface> fdiObjectList = fdiConfigList[kv.key]; //TODO austausch von config durch eine Liste welche die Objekte beinhaltet die die msgs brauchen. aufruf der check fkt mit der message
      ROS_INFO("Found %d obejcts that test this msg", fdiObjectList.size());

      for(int i=0;i<fdiObjectList.size();i++) {
        ROS_INFO("Working on %d obejct", i);
        fdiObjectList[i].check(kv);
      }
    } else {
//      ROS_INFO("No Key found");
    }
    msgBuffer.pop();
  }
}

