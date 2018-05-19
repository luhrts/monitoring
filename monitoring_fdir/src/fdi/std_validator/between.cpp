#include "ros_monitoring/fdi/std_validator/between.h"

Between::Between(float maxValue, std::string maxErrorMsg, float maxerrorLevel, float minValue, std::string minErrorMsg, float minerrorLevel, ros::Publisher& publisher)
  :maxValue(maxValue)
  ,maxmsg(maxErrorMsg)
  ,maxlevel(maxerrorLevel)
  ,minValue(minValue)
  ,minmsg(minErrorMsg)
  ,minlevel(minerrorLevel)
  ,ConfigInterface(publisher)
{

}
Between::~Between() {}

/**
 * checks the message if keyvalue pair is between the configured values(min/max), sends out a errormessage if not.
 */
void Between::check(ros_monitoring::KeyValue newMsg) {
//  ROS_INFO("Checking CPU TEMP %s", newMsg.value.c_str());

  std::string::size_type sz;
  float value = std::stof (newMsg.value,&sz);
  if(maxValue<value) {
    ROS_WARN("ERROR: Value: %f is higher then expected (%f), Errorlevel to %f", value, maxValue, maxlevel);
    ros_monitoring::Error maxerrormsg;
    maxerrormsg.header.stamp = ros::Time::now();
    maxerrormsg.key = maxmsg;
    maxerrormsg.value = newMsg.value;
    maxerrormsg.unit = newMsg.unit;
    maxerrormsg.errorlevel = maxlevel;
    publishError(maxerrormsg);

  } else if(minValue>=value) {
    ROS_WARN("ERROR: Value: %f is lower then expected (%f), Errorlevel to %f", value, minValue, minlevel);
    ros_monitoring::Error minerrormsg;
    minerrormsg.header.stamp = ros::Time::now();
    minerrormsg.key = minmsg;
    minerrormsg.value = newMsg.value;
    minerrormsg.unit = newMsg.unit;
    minerrormsg.errorlevel = minlevel;
    publishError(minerrormsg);
  }

}
