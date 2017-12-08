#include "between.h"

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


void Between::check(ros_monitoring::KeyValue newMsg) {
//  ROS_INFO("Checking CPU TEMP %s", newMsg.value.c_str());

  std::string::size_type sz;
  float value = std::stof (newMsg.value,&sz);
  if(maxValue<value) {
    ROS_ERROR("ERROR: Value: %f is higher then expected (%f), Errorlevel to %f", value, maxValue, maxlevel);
    ros_monitoring::Error maxerrormsg;
    maxerrormsg.key = maxmsg;
    maxerrormsg.value = newMsg.value;
    maxerrormsg.level = maxlevel;
    publishError(maxerrormsg);
  } else if(minValue>=value) {
    ROS_ERROR("ERROR: Value: %f is lower then expected (%f), Errorlevel to %f", value, minValue, minlevel);
    ros_monitoring::Error minerrormsg;
    minerrormsg.key = minmsg;
    minerrormsg.value = newMsg.value;
    minerrormsg.level = minlevel;
    publishError(minerrormsg);
  }

}
