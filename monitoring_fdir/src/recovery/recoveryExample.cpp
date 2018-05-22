#include <ros/ros.h>

#include "monitoring_fdir/recovery/recoverysdk.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "recoveryExample");
  ros::NodeHandle n("~");

  float freq = 1;
  if (!n.getParam("frequency", freq))
  {
    freq = 1;
    ROS_WARN("No frequency supplied. Working with %f Hz.", freq);
  }

  ros::Rate loop_rate(freq);

  RecoverySDK example(n);

  RestartNodeHandler rnh;
  StopLaunchFile slf;
  OutputErrorMessage oem;
  ErrorToSpeech ets(n);

  example.registerErrorHandler(&rnh, "node dead");
  example.registerErrorHandler(&slf, "CPU Overheating");
  example.registerErrorHandler(&oem, "CPU Hot");
  example.registerErrorHandler(&ets, "CPU Hot");


  ros::spin();
}
