#include "min.h"


Min::Min(float value, std::string errormsg, float errorLevel, ros::Publisher& publisher)
  :minValue(value)
  ,msg(errormsg)
  ,errorlevel(errorLevel)
  ,ConfigInterface(publisher)
{

}

Min::~Min() {}

/**
 * checks if the keyvalue pair is under the configured value
 */
void Min::check(ros_monitoring::KeyValue newMsg)
{
  std::string::size_type sz;
  float value = std::stof (newMsg.value,&sz);
  if(minValue>=value) {
    ROS_WARN("ERROR: Value: %f is lower then expected (%f), Errorlevel to %f", value, minValue, errorlevel);
    ros_monitoring::Error errormsg;
    errormsg.header.stamp = ros::Time::now();
    errormsg.key = msg;
    errormsg.value = newMsg.value;
    errormsg.unit = newMsg.unit;
    errormsg.errorlevel = errorlevel;
    publishError(errormsg);
  }
}
