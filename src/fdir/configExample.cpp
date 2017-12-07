#include <ros/ros.h>
#include "fdiSDK.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "user_fdi");
  ros::NodeHandle n("~");
  float freq = 1;
  if (!n.getParam("frequency", freq))
  {
    ROS_WARN("No frequency supplied. Working with %d Hz.", freq);
  }
  ros::Publisher pub = n.advertise<ros_monitoring::Error>("/monitoring/errors", 1000);
  FdiSDK userFDI(n);
  ros::Rate loop_rate(freq);


  Max maxtemp(30,"CPU Overheating", 0.5, pub);

  userFDI.registerFDIObject(maxtemp, "CPU Temperatur");
  while (ros::ok())
  {
    userFDI.fdi();
    loop_rate.sleep();
    ros::spinOnce();
  }
}

