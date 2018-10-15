#include <ros/ros.h>
#include "monitoring_fdir/fdi/fdiSDK.h"


/*
 * this is a example how you can implement a fdi for your ros system.
 */
int main(int argc, char **argv)
{
  //Initializising ROS with a publisher and a configurable frequency
  ros::init(argc, argv, "user_fdi");
  ros::NodeHandle n("~");
  ros::Publisher pub = n.advertise<monitoring_msgs::Error>("/monitoring/errors", 1000);
  double freq = 1;
  if (!n.getParam("frequency", freq))
  {
    freq = 1;
    ROS_WARN("No frequency supplied. Working with %f Hz.", freq);
  }
  ros::Rate loop_rate(freq);

  //Initializising the FDI SDK.
  FdiSDK userFDI(n);

  //Initializising the validators
  Between betweentemp(40, "CPU Hot", 0.99, 30, "Getting Cold", 0.1, pub);
  Max maxtemp(70,"CPU Overheating", 1.0, pub);
  Min mintemp(20,"CPU to Cold", 0.2, pub);
  NodeAvailable nodefdi(0.5, pub);

  //Registering the validators
  userFDI.registerFDIObject(&maxtemp, "CPU Temperatur");
  userFDI.registerFDIObject(&betweentemp, "CPU Temperatur");
  userFDI.registerFDIObject(&mintemp, "CPU Temperatur");
  userFDI.registerFDIObject(&nodefdi, "node unavailable");


  //ROS loop for checking the messages.
  ros::spin();
}

