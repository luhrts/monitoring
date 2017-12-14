#include <ros/ros.h>
#include "fdiSDK.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "user_fdi");
  ros::NodeHandle n("~");
  float freq = 1;
  if (!n.getParam("frequency", freq))
  {
    freq = 1;
    ROS_WARN("No frequency supplied. Working with %f Hz.", freq);
  }
  ros::Publisher pub = n.advertise<ros_monitoring::Error>("/monitoring/errors", 1000);
  FdiSDK userFDI(n);
  ros::Rate loop_rate(freq);



  Between betweentemp(40, "CPU Hot", 0.4, 30, "Getting Cold", 0.1, pub);
  Max maxtemp(60,"CPU Overheating", 0.7, pub);
  Min mintemp(20,"CPU to Cold", 0.2, pub);

  userFDI.registerFDIObject(&maxtemp, "CPU Temperatur");
  userFDI.registerFDIObject(&betweentemp, "CPU Temperatur");
  userFDI.registerFDIObject(&mintemp, "CPU Temperatur");

  NodeAvailable nodefdi(0.5, pub);

  userFDI.registerFDIObject(&nodefdi, "node unavailable");



  while (ros::ok())
  {
    userFDI.checkForFDI();
    loop_rate.sleep();
    ros::spinOnce();
  }
}

