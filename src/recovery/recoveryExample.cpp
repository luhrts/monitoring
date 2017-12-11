#include <ros/ros.h>

#include "recoverysdk.h"
#include "std_handler/restartnodehandler.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "recoveryExample");
  ros::NodeHandle nh;

  RecoverySDK example(nh);

  RestartNodeHandler rnh;
  example.registerErrorHandling(&rnh, "node dead");

  while(ros::ok()) {
    example.checkErrors();
    ros::spinOnce();
  }

}
