#include "nodeavailable.h"

NodeAvailable::NodeAvailable(float errorwert, ros::Publisher pub)
  :ConfigInterface(pub)
  ,errorlevel(errorwert)
{

}

/**
 * redirects the keyvalue pair if the msg node unavailable is read, so the recovery system can restart it.
 */
void NodeAvailable::check(ros_monitoring::KeyValue newMsg)
{
  if(newMsg.key == "node unavailable") {
    ros_monitoring::Error errormsg;
    errormsg.header.stamp = ros::Time::now();
    errormsg.key = "node dead";
    errormsg.value = newMsg.value;
    errormsg.unit = newMsg.unit;
    errormsg.errorlevel = errorlevel;
    publishError(errormsg);
  }
}
