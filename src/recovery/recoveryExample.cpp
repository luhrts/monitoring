#include <ros/ros.h>

#include "recoverysdk.h"
#include "std_handler/restartnodehandler.h"
#include "std_handler/stoplaunchfile.h"

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

  example.registerErrorHandler(&rnh, "node dead");
  example.registerErrorHandler(&slf, "CPU Overheating");

  while (ros::ok())
  {
    example.checkErrors();
    loop_rate.sleep();
    ros::spinOnce();
  }

}
