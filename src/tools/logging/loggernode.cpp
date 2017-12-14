/*
 * loggernode.cpp
 *
 *  Created on: Dec 14, 2017
 *      Author: matthias
 */

#include "datalogger.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "datalogger");
  ros::NodeHandle n("~");

  float freq = 1;
  if (!n.getParam("frequency", freq))
  {
    freq = 1;
    ROS_WARN("No frequency supplied. Working with %f Hz.", freq);
  }

  ros::Rate loop_rate(freq);

  DataLogger logger(testlog.log);

  while (ros::ok())
   {
     logger.writeData();
     loop_rate.sleep();
     ros::spinOnce();
   }

 }

